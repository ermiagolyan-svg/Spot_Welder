/*
  =====================================================================================
   SPOT WELDER FIRMWARE - ESP32-C3-SUPERMINI
   ZCD-synchronized integral-cycle triac control | Dual pulse w/ power% | Presets
   OLED + rotary encoder UI | Web panel (monitor/config only) | Safety watchdogs
  =====================================================================================

  PIN MAP (as given):
    GPIO4  SW1        - secondary/safety trigger switch (active LOW, pulled up)
    GPIO3  ENCODER A
    GPIO1  ENCODER SW  - encoder push button (active LOW, pulled up)
    GPIO0  ENCODER B   - NOTE: boot-strapping pin, fine to reuse after boot,
                         internal pullup keeps it HIGH at reset so it will not
                         interfere with normal boot.
    GPIO5  PC817 (ZCD) - mains zero-cross detector input
    GPIO6  WELD-SW     - main trigger switch (active LOW, pulled up)
    GPIO7  MOC3023 gate -> BTA41 triac -> MOT primary
    GPIO8  OLED SDA (I2C)  - also an ESP32-C3 strap pin; the I2C pull-up needed
                             anyway keeps it HIGH at boot, so this is fine.
    GPIO9  OLED SCL (I2C)  - same note as above (main boot-mode strap pin).
    GPIO10 BC547 -> buzzer

  REQUIRED LIBRARIES (Arduino Library Manager):
    - Adafruit GFX Library
    - Adafruit SSD1306
  (Preferences, WiFi, WebServer, esp_timer are all part of the ESP32 board core.)

  ASSUMPTIONS MADE (search "ASSUMPTION" to find/adjust each one):
    1. Trigger = SW1 AND WELD-SW closed together (dual-switch safety interlock).
       If SW1 is meant for something else in your build, edit triggerActive().
    2. OLED is a 128x64 SSD1306 at I2C address 0x3C.
    3. ZCD (PC817) output is a narrow HIGH pulse at each zero crossing -> using
       RISING edge. If firing looks wrong/halved, try FALLING instead.
    4. Web panel never fires a weld remotely - only physical switches can weld.
       This is a deliberate safety choice, not an oversight.

  This firmware switches MAINS VOLTAGE via a triac. It has NOT been compiled or
  bench-tested on real hardware. Review it, compile it, and test it first into a
  lamp/resistive dummy load (not the welding transformer) before trusting it on
  the real MOT. Start with LOW preset values and increase gradually while test
  welding scrap nickel strip - not directly on cells - until you're confident in
  the settings.
*/

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Preferences.h>
#include <WiFi.h>
#include <WebServer.h>
#include "esp_timer.h"

// ================================ Pin Definitions ================================
const uint8_t PIN_SW1      = 4;
const uint8_t PIN_ENC_A    = 3;
const uint8_t PIN_ENC_SW   = 1;
const uint8_t PIN_ENC_B    = 0;
const uint8_t PIN_ZCD      = 5;
const uint8_t PIN_WELD_SW  = 6;
const uint8_t PIN_TRIAC    = 7;
const uint8_t PIN_OLED_SDA = 8;
const uint8_t PIN_OLED_SCL = 9;
const uint8_t PIN_BUZZER   = 10;

// ================================ OLED ================================
#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT 64
#define OLED_ADDR     0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// ================================ WiFi ================================
// Fill STA_SSID in to have it join your existing WiFi first; leave blank to
// go straight to its own Access Point. Falls back to AP automatically if the
// STA connection doesn't succeed within a few seconds.
const char* STA_SSID = "";
const char* STA_PASS = "";
const char* AP_SSID  = "SpotWelder";
const char* AP_PASS  = "weld12345";
bool wifiIsAP = false;

// ================================ Safety / config constants ================================
const int           MAX_HALFCYCLES_PER_PULSE = 40;      // hard UI ceiling per pulse (~330-400ms)
const unsigned long ABSOLUTE_MAX_SEGMENT_US  = 1000000UL;// 1s absolute hard cap per pulse segment
const unsigned long ZCD_TIMEOUT_US           = 30000UL;  // no zero-cross for 30ms => assume lost
const unsigned long MIN_COOLDOWN_MS          = 800;      // minimum gap between welds
const unsigned long DEBOUNCE_MS              = 25;
const unsigned long LOCK_HOLD_MS             = 3000;     // hold encoder knob this long to lock/unlock
const int           NUM_PRESETS              = 6;
const float          HEAT_MAX                 = 100.0;   // thermal bucket ceiling (arbitrary units)
const float          HEAT_PER_HALFCYCLE       = 1.2;      // heat added per half-cycle actually fired
const float          HEAT_DECAY_PER_S         = 3.0;      // heat removed per second while idle

// ================================ State machine + error codes ================================
#define WS_IDLE        0
#define WS_PULSE1      1
#define WS_INTERDELAY  2
#define WS_PULSE2      3
#define WS_COOLDOWN    4
#define WS_LOCKED      5
#define WS_FAULT       6
int weldState = WS_IDLE;

#define ERR_NONE      0
#define ERR_ZCD_LOST  1
int lastError = ERR_NONE;

// ================================ UI screens ================================
#define SCR_HOME        0
#define SCR_PRESET_SEL  1
#define SCR_EDIT_P1     2
#define SCR_EDIT_PW1    3
#define SCR_EDIT_DELAY  4
#define SCR_EDIT_P2     5
#define SCR_EDIT_PW2    6
#define SCR_SAVE        7
#define SCR_WIFI_INFO   8
#define SCR_STATS       9
#define NUM_SCREENS     10
int  currentScreen = SCR_HOME;
bool inEditMode = false;

// ================================ Objects ================================
Preferences prefs;
WebServer   server(80);
esp_timer_handle_t safetyTimer;

// ================================ Presets (parallel arrays - no structs) ================================
int  presetPulse1[NUM_PRESETS];   // half-cycles
int  presetPower1[NUM_PRESETS];   // 10-100 %
int  presetDelay[NUM_PRESETS];    // half-cycles (non-energized gap)
int  presetPulse2[NUM_PRESETS];   // half-cycles, 0 = single-pulse mode
int  presetPower2[NUM_PRESETS];   // 10-100 %
char presetName[NUM_PRESETS][10];
int  currentPresetIndex = 0;

// ================================ Weld engine (shared with ISR/timer - volatile) ================================
volatile bool          weldActive = false;
volatile int           halfCyclesRemaining = 0;
volatile float         powerFraction = 1.0;
volatile float         powerAccum = 0.0;
volatile bool          pulseSegmentDone = false;
volatile unsigned long lastZcdMicros = 0;
volatile unsigned long zcdCount = 0;
volatile unsigned long weldStartMicros = 0;
volatile bool          zcdFaultFlag = false;
volatile unsigned long zcdPeriodAccumUs = 0;
volatile unsigned int  zcdPeriodSamples = 0;

float acFrequencyHz = 50.0;

// ================================ Weld sequencing / thermal ================================
float         heatAccum = 0.0;
unsigned long lastHeatUpdateMs = 0;
unsigned long weldEndMs = 0;
unsigned long interDelayStartMs = 0;
unsigned long totalWeldCount = 0;
unsigned int  weldsSinceSave = 0;
bool          deviceLocked = false;

// ================================ Encoder (interrupt-driven quadrature decode) ================================
volatile long  encoderTicks = 0;
volatile uint8_t encPrevState = 0;
const int8_t encTable[16] = {
   0, -1,  1,  0,
   1,  0,  0, -1,
  -1,  0,  0,  1,
   0,  1, -1,  0
};

// ================================ Buttons (debounced) ================================
bool sw1Stable = false, sw1Last = false;         unsigned long sw1LastChange = 0;
bool weldSwStable = false, weldSwLast = false;   unsigned long weldSwLastChange = 0;
bool encSwStable = false, encSwLast = false;     unsigned long encSwLastChange = 0;
unsigned long encSwPressStart = 0;
bool encSwLongHandled = false;

// ================================ Buzzer (non-blocking patterns) ================================
#define MAX_BUZZ_STEPS 6
int buzzSeq[MAX_BUZZ_STEPS];
int buzzLen = 0;
int buzzerPattern = 0;
int buzzerPatternStep = 0;
unsigned long buzzerPatternNextMs = 0;

// ================================ Display refresh timing ================================
unsigned long lastDisplayMs = 0;


// =====================================================================================
//  SETUP
// =====================================================================================
void setup() {
  // SAFETY FIRST: force the triac gate and buzzer to a known OFF state before
  // anything else (WiFi/OLED init, etc.) has a chance to run or stall.
  pinMode(PIN_TRIAC, OUTPUT);
  digitalWrite(PIN_TRIAC, LOW);
  pinMode(PIN_BUZZER, OUTPUT);
  digitalWrite(PIN_BUZZER, LOW);

  Serial.begin(115200);

  pinMode(PIN_SW1, INPUT_PULLUP);
  pinMode(PIN_WELD_SW, INPUT_PULLUP);
  pinMode(PIN_ENC_SW, INPUT_PULLUP);
  pinMode(PIN_ENC_A, INPUT_PULLUP);
  pinMode(PIN_ENC_B, INPUT_PULLUP);
  pinMode(PIN_ZCD, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(PIN_ENC_A), encoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_B), encoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ZCD), zcdISR, RISING); // ASSUMPTION #3

  Wire.begin(PIN_OLED_SDA, PIN_OLED_SCL);
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) { // ASSUMPTION #2
    Serial.println("OLED init failed - check wiring/address");
  }
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("Spot Welder");
  display.println("Booting...");
  display.display();

  prefs.begin("weldercfg", false);
  loadAllPresets();
  currentPresetIndex = prefs.getInt("lastPreset", 0);
  if (currentPresetIndex < 0 || currentPresetIndex >= NUM_PRESETS) currentPresetIndex = 0;
  totalWeldCount = prefs.getULong("weldCount", 0);
  deviceLocked   = prefs.getBool("locked", false);

  setupWiFi();
  setupWebServer();

  // Independent hardware-timer safety watchdog: runs every 5ms in its own
  // task context, regardless of what loop()/WiFi/webserver is doing.
  esp_timer_create_args_t timerArgs = {};
  timerArgs.callback = &safetyTimerCallback;
  timerArgs.arg = NULL;
  timerArgs.dispatch_method = ESP_TIMER_TASK;
  timerArgs.name = "safety_wd";
  esp_timer_create(&timerArgs, &safetyTimer);
  esp_timer_start_periodic(safetyTimer, 5000); // 5000us = 5ms

  lastHeatUpdateMs = millis();
  weldState = deviceLocked ? WS_LOCKED : WS_IDLE;

  beepPattern(1); // startup beep
}


// =====================================================================================
//  MAIN LOOP (non-blocking - no delay() in here or anything it calls)
// =====================================================================================
void loop() {
  server.handleClient();
  updateButtons();
  handleUI();
  handleWeldStateMachine();
  updateBuzzer();

  static unsigned long lastFreqCalc = 0;
  if (millis() - lastFreqCalc > 500) {
    updateFrequencyEstimate();
    lastFreqCalc = millis();
  }

  if (millis() - lastDisplayMs > 100) { // ~10Hz OLED refresh
    renderDisplay();
    lastDisplayMs = millis();
  }
}


// =====================================================================================
//  TRIGGER LOGIC (ASSUMPTION #1 - isolated here so it's a one-line change)
// =====================================================================================
bool triggerActive() {
  // Both switches must be closed together (dual-switch safety interlock).
  // Single-switch version (WELD-SW only):   return weldSwStable;
  return sw1Stable && weldSwStable;
}


// =====================================================================================
//  ZERO-CROSS ISR - integral-cycle (burst) firing with per-pulse power%
// =====================================================================================
void IRAM_ATTR zcdISR() {
  unsigned long now = micros();
  unsigned long period = now - lastZcdMicros;
  if (period < 3000) return; // ignore noise/bounce (<3ms - real half-cycle is ~8.3-10ms)
  lastZcdMicros = now;
  zcdCount++;

  if (period < 15000) { // sane range before feeding the frequency estimator
    zcdPeriodAccumUs += period;
    zcdPeriodSamples++;
  }

  if (weldActive && halfCyclesRemaining > 0) {
    // Time-proportioning: spreads ON half-cycles evenly across the pulse
    // rather than firing them all consecutively, per powerFraction (1.0=100%).
    powerAccum += powerFraction;
    if (powerAccum >= 1.0f) {
      digitalWrite(PIN_TRIAC, HIGH);
      powerAccum -= 1.0f;
    } else {
      digitalWrite(PIN_TRIAC, LOW);
    }
    halfCyclesRemaining--;
    if (halfCyclesRemaining <= 0) {
      weldActive = false;
      digitalWrite(PIN_TRIAC, LOW);
      pulseSegmentDone = true;
    }
  } else {
    digitalWrite(PIN_TRIAC, LOW);
  }
}


// =====================================================================================
//  SAFETY WATCHDOG - independent hardware timer, catches a lost ZCD signal or a
//  stuck-active weld even if loop()/ISR logic misbehaves. This is what protects
//  against the triac gate being left driven if zero-cross pulses stop arriving.
// =====================================================================================
void safetyTimerCallback(void* arg) {
  if (weldActive) {
    unsigned long nowUs = micros();
    if ((unsigned long)(nowUs - lastZcdMicros) > ZCD_TIMEOUT_US) {
      weldActive = false;
      digitalWrite(PIN_TRIAC, LOW);
      zcdFaultFlag = true;
    } else if ((unsigned long)(nowUs - weldStartMicros) > ABSOLUTE_MAX_SEGMENT_US) {
      weldActive = false;
      digitalWrite(PIN_TRIAC, LOW);
      zcdFaultFlag = true;
    }
  }
}


// =====================================================================================
//  ENCODER ISR - standard full-step quadrature decode
// =====================================================================================
void IRAM_ATTR encoderISR() {
  uint8_t a = digitalRead(PIN_ENC_A);
  uint8_t b = digitalRead(PIN_ENC_B);
  uint8_t state = (a << 1) | b;
  uint8_t idx = ((encPrevState & 0x03) << 2) | (state & 0x03);
  encoderTicks += encTable[idx & 0x0F];
  encPrevState = state;
}


// =====================================================================================
//  BUTTONS - simple debounce, polled once per loop()
// =====================================================================================
void updateButtons() {
  unsigned long now = millis();

  bool sw1Raw = (digitalRead(PIN_SW1) == LOW);
  if (sw1Raw != sw1Last) { sw1LastChange = now; sw1Last = sw1Raw; }
  if (now - sw1LastChange > DEBOUNCE_MS) sw1Stable = sw1Raw;

  bool weldRaw = (digitalRead(PIN_WELD_SW) == LOW);
  if (weldRaw != weldSwLast) { weldSwLastChange = now; weldSwLast = weldRaw; }
  if (now - weldSwLastChange > DEBOUNCE_MS) weldSwStable = weldRaw;

  bool encRaw = (digitalRead(PIN_ENC_SW) == LOW);
  if (encRaw != encSwLast) { encSwLastChange = now; encSwLast = encRaw; }
  if (now - encSwLastChange > DEBOUNCE_MS) {
    if (encRaw && !encSwStable) { encSwPressStart = now; encSwLongHandled = false; }
    encSwStable = encRaw;
  }
}


// =====================================================================================
//  WELD SEQUENCE STATE MACHINE
// =====================================================================================
void startWeld() {
  int hc1 = presetPulse1[currentPresetIndex];
  if (hc1 <= 0) return;
  if (hc1 > MAX_HALFCYCLES_PER_PULSE) hc1 = MAX_HALFCYCLES_PER_PULSE;

  powerFraction = constrain(presetPower1[currentPresetIndex], 10, 100) / 100.0f;
  powerAccum = 0;
  halfCyclesRemaining = hc1;
  weldStartMicros = micros();
  weldActive = true;
  weldState = WS_PULSE1;
  beepPattern(2);
}

void startPulse2() {
  int hc2 = presetPulse2[currentPresetIndex];
  if (hc2 > MAX_HALFCYCLES_PER_PULSE) hc2 = MAX_HALFCYCLES_PER_PULSE;
  powerFraction = constrain(presetPower2[currentPresetIndex], 10, 100) / 100.0f;
  powerAccum = 0;
  halfCyclesRemaining = hc2;
  weldStartMicros = micros();
  weldActive = true;
  weldState = WS_PULSE2;
}

void finishWeld() {
  weldEndMs = millis();
  weldState = WS_COOLDOWN;
  totalWeldCount++;
  weldsSinceSave++;
  heatAccum += HEAT_PER_HALFCYCLE * (presetPulse1[currentPresetIndex] + presetPulse2[currentPresetIndex]);
  if (weldsSinceSave >= 5) { // batch NVS writes to reduce flash wear
    prefs.putULong("weldCount", totalWeldCount);
    weldsSinceSave = 0;
  }
  beepPattern(4);
}

unsigned long halfCycleToMs(int halfCycles) {
  float halfPeriodMs = 1000.0 / (acFrequencyHz * 2.0);
  return (unsigned long)(halfCycles * halfPeriodMs);
}

void handleWeldStateMachine() {
  unsigned long now = millis();

  float dtS = (now - lastHeatUpdateMs) / 1000.0;
  if (dtS > 0) {
    heatAccum -= HEAT_DECAY_PER_S * dtS;
    if (heatAccum < 0) heatAccum = 0;
    lastHeatUpdateMs = now;
  }

  if (pulseSegmentDone) {
    pulseSegmentDone = false;
    if (weldState == WS_PULSE1) {
      if (presetPulse2[currentPresetIndex] > 0) {
        weldState = WS_INTERDELAY;
        interDelayStartMs = now;
      } else {
        finishWeld();
      }
    } else if (weldState == WS_PULSE2) {
      finishWeld();
    }
  }

  if (zcdFaultFlag) {
    zcdFaultFlag = false;
    weldState = WS_FAULT;
    lastError = ERR_ZCD_LOST;
    beepPattern(3);
  }

  switch (weldState) {
    case WS_IDLE:
      if (deviceLocked) { weldState = WS_LOCKED; break; }
      if (now - weldEndMs < MIN_COOLDOWN_MS) break;
      if (heatAccum > HEAT_MAX) break; // overheat lockout, cooling down
      if (triggerActive()) startWeld();
      break;

    case WS_LOCKED:
      if (!deviceLocked) weldState = WS_IDLE;
      break;

    case WS_INTERDELAY: {
      unsigned long delayMs = halfCycleToMs(presetDelay[currentPresetIndex]);
      if (now - interDelayStartMs >= delayMs) startPulse2();
      break;
    }

    case WS_COOLDOWN:
      if (now - weldEndMs >= MIN_COOLDOWN_MS) {
        weldState = deviceLocked ? WS_LOCKED : WS_IDLE;
      }
      break;

    case WS_FAULT:
      // stays here until the user acknowledges with a short encoder press
      break;

    default:
      break;
  }
}


// =====================================================================================
//  MAINS FREQUENCY ESTIMATE (informational; firing itself doesn't need this)
// =====================================================================================
void updateFrequencyEstimate() {
  noInterrupts();
  unsigned long accum = zcdPeriodAccumUs;
  unsigned int samples = zcdPeriodSamples;
  zcdPeriodAccumUs = 0;
  zcdPeriodSamples = 0;
  interrupts();

  if (samples >= 10) {
    float avgPeriodUs = (float)accum / samples;
    float freq = 1000000.0 / (avgPeriodUs * 2.0);
    if (freq > 40 && freq < 70) acFrequencyHz = freq; // sane-range guard
  }
}


// =====================================================================================
//  PRESET STORAGE (Preferences / NVS)
// =====================================================================================
void loadAllPresets() {
  for (int i = 0; i < NUM_PRESETS; i++) {
    String base = "p" + String(i) + "_";
    presetPulse1[i] = prefs.getInt((base + "p1").c_str(), 2);
    presetPower1[i] = prefs.getInt((base + "pw1").c_str(), 100);
    presetDelay[i]  = prefs.getInt((base + "dl").c_str(), 3);
    presetPulse2[i] = prefs.getInt((base + "p2").c_str(), 3);
    presetPower2[i] = prefs.getInt((base + "pw2").c_str(), 100);
    String nm = prefs.getString((base + "nm").c_str(), "P" + String(i + 1));
    nm.toCharArray(presetName[i], 10);
  }
}

void savePreset(int idx) {
  if (idx < 0 || idx >= NUM_PRESETS) return;
  String base = "p" + String(idx) + "_";
  prefs.putInt((base + "p1").c_str(), presetPulse1[idx]);
  prefs.putInt((base + "pw1").c_str(), presetPower1[idx]);
  prefs.putInt((base + "dl").c_str(), presetDelay[idx]);
  prefs.putInt((base + "p2").c_str(), presetPulse2[idx]);
  prefs.putInt((base + "pw2").c_str(), presetPower2[idx]);
  prefs.putString((base + "nm").c_str(), String(presetName[idx]));
}


// =====================================================================================
//  UI / MENU (encoder navigation)
// =====================================================================================
void handleUI() {
  static long lastProcessedTicks = 0;
  noInterrupts();
  long ticksNow = encoderTicks;
  interrupts();
  int delta = (int)((ticksNow - lastProcessedTicks) / 4); // 4 ticks per detent, typical EC11
  lastProcessedTicks += (long)delta * 4;

  bool shortPress = false, longPress = false;
  static bool wasPressed = false;
  if (encSwStable && !wasPressed) {
    wasPressed = true;
  }
  if (!encSwStable && wasPressed) {
    wasPressed = false;
    if (!encSwLongHandled) shortPress = true;
  }
  if (encSwStable && !encSwLongHandled && (millis() - encSwPressStart > LOCK_HOLD_MS)) {
    longPress = true;
    encSwLongHandled = true;
  }

  if (longPress) {
    deviceLocked = !deviceLocked;
    prefs.putBool("locked", deviceLocked);
    weldState = deviceLocked ? WS_LOCKED : WS_IDLE;
    beepPattern(4);
    return;
  }

  if (weldState == WS_FAULT) {
    if (shortPress) {
      lastError = ERR_NONE;
      weldState = deviceLocked ? WS_LOCKED : WS_IDLE;
      beepPattern(5);
    }
    return; // no menu navigation while faulted
  }

  if (delta != 0) beepPattern(5);

  if (!inEditMode) {
    if (delta > 0) currentScreen = (currentScreen + 1) % NUM_SCREENS;
    if (delta < 0) currentScreen = (currentScreen - 1 + NUM_SCREENS) % NUM_SCREENS;

    if (shortPress) {
      switch (currentScreen) {
        case SCR_PRESET_SEL:
        case SCR_EDIT_P1:
        case SCR_EDIT_PW1:
        case SCR_EDIT_DELAY:
        case SCR_EDIT_P2:
        case SCR_EDIT_PW2:
          inEditMode = true;
          break;
        case SCR_SAVE:
          savePreset(currentPresetIndex);
          prefs.putInt("lastPreset", currentPresetIndex);
          beepPattern(4);
          break;
        default:
          break;
      }
    }
  } else {
    switch (currentScreen) {
      case SCR_PRESET_SEL:
        currentPresetIndex = constrain(currentPresetIndex + delta, 0, NUM_PRESETS - 1);
        break;
      case SCR_EDIT_P1:
        presetPulse1[currentPresetIndex] = constrain(presetPulse1[currentPresetIndex] + delta, 1, MAX_HALFCYCLES_PER_PULSE);
        break;
      case SCR_EDIT_PW1:
        // Note: with very few half-cycles, a low power% may round down to zero
        // actual conduction (time-proportioning needs a few cycles to average
        // over). Keep power high for very short pulses.
        presetPower1[currentPresetIndex] = constrain(presetPower1[currentPresetIndex] + delta, 10, 100);
        break;
      case SCR_EDIT_DELAY:
        presetDelay[currentPresetIndex] = constrain(presetDelay[currentPresetIndex] + delta, 1, MAX_HALFCYCLES_PER_PULSE);
        break;
      case SCR_EDIT_P2:
        presetPulse2[currentPresetIndex] = constrain(presetPulse2[currentPresetIndex] + delta, 0, MAX_HALFCYCLES_PER_PULSE);
        break;
      case SCR_EDIT_PW2:
        presetPower2[currentPresetIndex] = constrain(presetPower2[currentPresetIndex] + delta, 10, 100);
        break;
      default:
        break;
    }
    if (shortPress) inEditMode = false;
  }
}


// =====================================================================================
//  BUZZER - non-blocking pattern playback
// =====================================================================================
void beepPattern(int pattern) {
  buzzerPatternStep = 0;
  switch (pattern) {
    case 1: buzzLen = 1; buzzSeq[0] = 70; break;                                       // startup
    case 2: buzzLen = 1; buzzSeq[0] = 20; break;                                       // weld start
    case 3: buzzLen = 6; for (int i = 0; i < 6; i++) buzzSeq[i] = 80; break;            // error: 3 beeps
    case 4: buzzLen = 3; buzzSeq[0] = 40; buzzSeq[1] = 60; buzzSeq[2] = 40; break;      // complete: 2 beeps
    case 5: buzzLen = 1; buzzSeq[0] = 12; break;                                       // UI click
    default: buzzLen = 0; return;
  }
  buzzerPattern = pattern;
  buzzerPatternNextMs = millis();
}

void updateBuzzer() {
  if (buzzerPattern == 0) return;
  unsigned long now = millis();
  if ((long)(now - buzzerPatternNextMs) < 0) return;

  if (buzzerPatternStep >= buzzLen) {
    buzzerPattern = 0;
    digitalWrite(PIN_BUZZER, LOW);
    return;
  }
  bool isOn = (buzzerPatternStep % 2 == 0);
  digitalWrite(PIN_BUZZER, isOn ? HIGH : LOW);
  buzzerPatternNextMs = now + buzzSeq[buzzerPatternStep];
  buzzerPatternStep++;
}


// =====================================================================================
//  OLED RENDERING
// =====================================================================================
const char* stateName() {
  switch (weldState) {
    case WS_IDLE:       return "READY";
    case WS_PULSE1:      return "WELD P1";
    case WS_INTERDELAY:  return "DELAY";
    case WS_PULSE2:      return "WELD P2";
    case WS_COOLDOWN:    return "COOLDOWN";
    case WS_LOCKED:      return "LOCKED";
    case WS_FAULT:       return "FAULT";
    default:             return "?";
  }
}

void renderDisplay() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);

  if (weldState == WS_FAULT) {
    display.setTextSize(2);
    display.println("FAULT!");
    display.setTextSize(1);
    if (lastError == ERR_ZCD_LOST) display.println("ZCD signal lost");
    display.println("Press knob to clear");
    display.display();
    return;
  }

  display.print(stateName());
  display.print("  ");
  display.print(acFrequencyHz, 1);
  display.println("Hz");
  display.print("Welds: ");
  display.println(totalWeldCount);
  display.drawLine(0, 18, 127, 18, SSD1306_WHITE);
  display.setCursor(0, 22);

  switch (currentScreen) {
    case SCR_HOME:
      display.print("Preset: "); display.println(presetName[currentPresetIndex]);
      display.print("P1:"); display.print(presetPulse1[currentPresetIndex]);
      display.print("hc@"); display.print(presetPower1[currentPresetIndex]); display.println("%");
      display.print("Dl:"); display.println(presetDelay[currentPresetIndex]);
      display.print("P2:"); display.print(presetPulse2[currentPresetIndex]);
      display.print("hc@"); display.print(presetPower2[currentPresetIndex]); display.println("%");
      if (deviceLocked) display.println("** LOCKED **");
      break;

    case SCR_PRESET_SEL:
      display.println(inEditMode ? "> Select preset" : "  Select preset");
      display.setTextSize(2);
      display.println(presetName[currentPresetIndex]);
      break;

    case SCR_EDIT_P1:
      display.println(inEditMode ? "> Pulse 1 (hc)" : "  Pulse 1 (hc)");
      display.setTextSize(2);
      display.println(presetPulse1[currentPresetIndex]);
      break;

    case SCR_EDIT_PW1:
      display.println(inEditMode ? "> Power 1 (%)" : "  Power 1 (%)");
      display.setTextSize(2);
      display.println(presetPower1[currentPresetIndex]);
      break;

    case SCR_EDIT_DELAY:
      display.println(inEditMode ? "> Delay (hc)" : "  Delay (hc)");
      display.setTextSize(2);
      display.println(presetDelay[currentPresetIndex]);
      break;

    case SCR_EDIT_P2:
      display.println(inEditMode ? "> Pulse 2 (hc)" : "  Pulse 2 (hc)");
      display.setTextSize(2);
      display.println(presetPulse2[currentPresetIndex]);
      break;

    case SCR_EDIT_PW2:
      display.println(inEditMode ? "> Power 2 (%)" : "  Power 2 (%)");
      display.setTextSize(2);
      display.println(presetPower2[currentPresetIndex]);
      break;

    case SCR_SAVE:
      display.println("Press knob to");
      display.println("SAVE preset");
      break;

    case SCR_WIFI_INFO:
      display.println(wifiIsAP ? "Mode: AP" : "Mode: STA");
      display.println(wifiIsAP ? AP_SSID : STA_SSID);
      display.print("IP: ");
      display.println(wifiIsAP ? WiFi.softAPIP().toString() : WiFi.localIP().toString());
      break;

    case SCR_STATS:
      display.print("Total welds: "); display.println(totalWeldCount);
      display.print("Heat: "); display.print((int)heatAccum); display.println("%");
      break;

    default:
      break;
  }
  display.display();
}


// =====================================================================================
//  WIFI + WEB SERVER (monitor & configure only - no remote firing, by design)
// =====================================================================================
void setupWiFi() {
  if (strlen(STA_SSID) > 0) {
    WiFi.mode(WIFI_STA);
    WiFi.begin(STA_SSID, STA_PASS);
    unsigned long start = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - start < 8000) {
      delay(200); // setup() only, one-time, not in loop()
    }
  }
  if (WiFi.status() == WL_CONNECTED) {
    wifiIsAP = false;
    Serial.println("WiFi STA connected: " + WiFi.localIP().toString());
  } else {
    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASS);
    wifiIsAP = true;
    Serial.println("WiFi AP started: " + WiFi.softAPIP().toString());
  }
}

const char INDEX_HTML[] PROGMEM = R"HTMLPAGE(
<!DOCTYPE html><html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Spot Welder</title>
<style>
body{font-family:sans-serif;background:#111;color:#eee;margin:0;padding:16px;}
h1{font-size:20px;}
.card{background:#1c1c1c;border-radius:8px;padding:12px;margin-bottom:12px;}
label{display:block;margin-top:8px;font-size:13px;color:#aaa;}
input,select{width:100%;padding:8px;margin-top:4px;background:#222;color:#eee;border:1px solid #444;border-radius:4px;box-sizing:border-box;}
button{margin-top:14px;padding:10px;width:100%;background:#2a72d4;color:#fff;border:none;border-radius:4px;font-size:15px;}
.stat{display:flex;justify-content:space-between;padding:4px 0;font-size:14px;}
.note{font-size:13px;color:#ccc;}
</style></head><body>
<h1>Spot Welder</h1>
<div class="card">
  <div class="stat"><span>State</span><span id="state">-</span></div>
  <div class="stat"><span>Mains freq</span><span id="freq">-</span></div>
  <div class="stat"><span>Total welds</span><span id="count">-</span></div>
  <div class="stat"><span>Heat</span><span id="heat">-</span></div>
  <div class="stat"><span>WiFi</span><span id="wifi">-</span></div>
</div>
<div class="card note">
  <b>This panel cannot trigger a weld.</b> Firing only happens from the
  physical switches on the device, for safety.
</div>
<div class="card">
  <label>Preset</label>
  <select id="preset" onchange="selectPreset()"></select>
  <label>Name</label><input type="text" id="pname" maxlength="9">
  <label>Pulse 1 (half-cycles)</label><input type="number" id="p1" min="1" max="40">
  <label>Power 1 (%)</label><input type="number" id="pw1" min="10" max="100">
  <label>Delay (half-cycles)</label><input type="number" id="dl" min="1" max="40">
  <label>Pulse 2 (half-cycles, 0=off)</label><input type="number" id="p2" min="0" max="40">
  <label>Power 2 (%)</label><input type="number" id="pw2" min="10" max="100">
  <button type="button" onclick="saveCfg()">Save to preset</button>
</div>
<script>
let presets = [];
async function refresh() {
  const r = await fetch('/status'); const d = await r.json();
  document.getElementById('state').textContent = d.state;
  document.getElementById('freq').textContent = d.freq.toFixed(1) + ' Hz';
  document.getElementById('count').textContent = d.count;
  document.getElementById('heat').textContent = Math.round(d.heat) + '%';
  document.getElementById('wifi').textContent = d.wifiMode + ' ' + d.ip;
  presets = d.presets;
  const sel = document.getElementById('preset');
  if (sel.options.length !== presets.length) {
    sel.innerHTML = '';
    presets.forEach((p,i)=>{ const o=document.createElement('option'); o.value=i; o.textContent=p.name; sel.appendChild(o); });
  }
  sel.value = d.current;
  const focused = document.activeElement.tagName === 'INPUT';
  const p = presets[d.current];
  if (p && !focused) {
    document.getElementById('pname').value = p.name;
    document.getElementById('p1').value = p.p1;
    document.getElementById('pw1').value = p.pw1;
    document.getElementById('dl').value = p.dl;
    document.getElementById('p2').value = p.p2;
    document.getElementById('pw2').value = p.pw2;
  }
}
function selectPreset() {
  const idx = document.getElementById('preset').value;
  const fd = new URLSearchParams(); fd.append('idx', idx);
  fetch('/preset', {method:'POST', body: fd}).then(refresh);
}
function saveCfg() {
  const fd = new URLSearchParams();
  fd.append('idx', document.getElementById('preset').value);
  fd.append('name', document.getElementById('pname').value);
  fd.append('p1', document.getElementById('p1').value);
  fd.append('pw1', document.getElementById('pw1').value);
  fd.append('dl', document.getElementById('dl').value);
  fd.append('p2', document.getElementById('p2').value);
  fd.append('pw2', document.getElementById('pw2').value);
  fetch('/save', {method:'POST', body: fd}).then(refresh);
}
setInterval(refresh, 1000);
refresh();
</script>
</body></html>
)HTMLPAGE";

void handleRoot() {
  server.send_P(200, "text/html", INDEX_HTML);
}

void handleStatus() {
  String ip = wifiIsAP ? WiFi.softAPIP().toString() : WiFi.localIP().toString();
  String json = "{";
  json += "\"state\":\"" + String(stateName()) + "\",";
  json += "\"freq\":" + String(acFrequencyHz, 1) + ",";
  json += "\"count\":" + String(totalWeldCount) + ",";
  json += "\"heat\":" + String(heatAccum, 1) + ",";
  json += "\"current\":" + String(currentPresetIndex) + ",";
  json += "\"locked\":" + String(deviceLocked ? "true" : "false") + ",";
  json += "\"wifiMode\":\"" + String(wifiIsAP ? "AP" : "STA") + "\",";
  json += "\"ip\":\"" + ip + "\",";
  json += "\"presets\":[";
  for (int i = 0; i < NUM_PRESETS; i++) {
    if (i > 0) json += ",";
    json += "{\"name\":\"" + String(presetName[i]) + "\",";
    json += "\"p1\":" + String(presetPulse1[i]) + ",";
    json += "\"pw1\":" + String(presetPower1[i]) + ",";
    json += "\"dl\":" + String(presetDelay[i]) + ",";
    json += "\"p2\":" + String(presetPulse2[i]) + ",";
    json += "\"pw2\":" + String(presetPower2[i]) + "}";
  }
  json += "]}";
  server.send(200, "application/json", json);
}

void handleSelectPreset() {
  if (server.hasArg("idx")) {
    int idx = server.arg("idx").toInt();
    if (idx >= 0 && idx < NUM_PRESETS) {
      currentPresetIndex = idx;
      prefs.putInt("lastPreset", idx);
    }
  }
  server.send(200, "text/plain", "OK");
}

void handleSave() {
  if (server.hasArg("idx")) {
    int idx = server.arg("idx").toInt();
    if (idx >= 0 && idx < NUM_PRESETS) {
      if (server.hasArg("name")) {
        String nm = server.arg("name");
        if (nm.length() == 0) nm = "P" + String(idx + 1);
        nm.toCharArray(presetName[idx], 10);
      }
      if (server.hasArg("p1"))  presetPulse1[idx] = constrain(server.arg("p1").toInt(), 1, MAX_HALFCYCLES_PER_PULSE);
      if (server.hasArg("pw1")) presetPower1[idx] = constrain(server.arg("pw1").toInt(), 10, 100);
      if (server.hasArg("dl"))  presetDelay[idx]  = constrain(server.arg("dl").toInt(), 1, MAX_HALFCYCLES_PER_PULSE);
      if (server.hasArg("p2"))  presetPulse2[idx] = constrain(server.arg("p2").toInt(), 0, MAX_HALFCYCLES_PER_PULSE);
      if (server.hasArg("pw2")) presetPower2[idx] = constrain(server.arg("pw2").toInt(), 10, 100);
      savePreset(idx);
      currentPresetIndex = idx;
      prefs.putInt("lastPreset", idx);
    }
  }
  server.send(200, "text/plain", "OK");
}

void setupWebServer() {
  server.on("/", HTTP_GET, handleRoot);
  server.on("/status", HTTP_GET, handleStatus);
  server.on("/save", HTTP_POST, handleSave);
  server.on("/preset", HTTP_POST, handleSelectPreset);
  server.begin();
}
