#include "UI.h"
#include "WeldEngine.h" // برای دسترسی به وضعیت موتور جوش و متغیرهای حرارت/تعداد

// ================================ OLED Object ================================
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// ================================ External Variables ================================
// این متغیر در ماژول WebPanel (وای‌فای) تعریف خواهد شد
extern bool wifiIsAP;

// ================================ UI Internal State ================================
int  currentScreen = SCR_HOME;
bool inEditMode = false;

// ================================ Display refresh timing ================================
unsigned long lastDisplayMs = 0;

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
    case WS_IDLE:        return "READY";
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