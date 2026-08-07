#include "WeldEngine.h"

// ارجاع به متغیرها و توابعی که در بخش UI و دکمه‌ها تعریف خواهند شد
extern bool sw1Stable;
extern bool weldSwStable;
extern void beepPattern(int pattern);

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
volatile unsigned long expectedHalfPeriodUs = 10000; // adaptive noise-reject reference, starts at 50Hz
volatile bool          halfCycleParity = false; // false=1st half of an AC full-cycle pair, true=2nd
volatile bool          currentPairFire = false; // fire decision made once per pair, reused for both halves

float acFrequencyHz = 50.0;

// ================================ Weld sequencing / thermal ================================
float         heatAccum = 0.0;
unsigned long lastHeatUpdateMs = 0;
unsigned long weldEndMs = 0;
unsigned long interDelayStartMs = 0;
unsigned long totalWeldCount = 0;
bool          deviceLocked = false;

int weldState = WS_IDLE;
int lastError = ERR_NONE;

// =====================================================================================
//  TRIGGER LOGIC (ASSUMPTION #1 - isolated here so it's a one-line change)
// =====================================================================================
bool triggerActive() {
  // Both switches must be closed together (dual-switch safety interlock).
  // Single-switch version (WELD-SW only):   return weldSwStable;
  return sw1Stable && weldSwStable;
}

void IRAM_ATTR zcdISR() {
  unsigned long now = micros();
  unsigned long period = now - lastZcdMicros;

  // Adaptive noise-reject: a real half-cycle runs ~8.3ms(60Hz)-10ms(50Hz).
  // Reject anything under half the currently-expected half-period as mains
  // noise or contact bounce (e.g. from our own triac switching). The
  // threshold self-calibrates instead of using one fixed guess, so it
  // tracks whichever of 50/60Hz is actually present.
  unsigned long minAccept = expectedHalfPeriodUs / 2;
  if (period < minAccept) return;

  lastZcdMicros = now;
  zcdCount++;

  if (period < 15000) { // sane range before trusting it
    zcdPeriodAccumUs += period;
    zcdPeriodSamples++;
    expectedHalfPeriodUs = (expectedHalfPeriodUs * 3 + period) / 4; // fast-adapting running average
  }

  if (weldActive && halfCyclesRemaining > 0) {
    // Fire in matched POSITIVE+NEGATIVE half-cycle PAIRS - never a lone
    // half-cycle. The fire/skip decision is made once per pair (on the
    // first half) and reused unchanged for the second half. This is what
    // guarantees the triac never applies more positive than negative
    // half-cycles to the transformer: an imbalance is exactly what walks
    // a MOT core into DC saturation and destructive inrush current.
    if (!halfCycleParity) {
      powerAccum += powerFraction;
      if (powerAccum >= 1.0f) {
        currentPairFire = true;
        powerAccum -= 1.0f;
      } else {
        currentPairFire = false;
      }
    }
    digitalWrite(PIN_TRIAC, currentPairFire ? HIGH : LOW);

    halfCyclesRemaining--;
    halfCycleParity = !halfCycleParity;

    if (halfCyclesRemaining <= 0) {
      weldActive = false;
      digitalWrite(PIN_TRIAC, LOW);
      pulseSegmentDone = true;
      halfCycleParity = false; // next segment always starts on a fresh pair
    }
  } else {
    digitalWrite(PIN_TRIAC, LOW);
  }
}

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

void startWeld() {
  int hc1 = presetPulse1[currentPresetIndex];
  if (hc1 <= 0) return;
  if (hc1 > MAX_HALFCYCLES_PER_PULSE) hc1 = MAX_HALFCYCLES_PER_PULSE;
  if (hc1 % 2 != 0) hc1++; // always a whole number of full AC cycles - see zcdISR

  powerFraction = constrain(presetPower1[currentPresetIndex], 10, 100) / 100.0f;
  powerAccum = 0;
  halfCycleParity = false;
  halfCyclesRemaining = hc1;
  weldStartMicros = micros();
  weldActive = true;
  weldState = WS_PULSE1;
  beepPattern(2);
}

void startPulse2() {
  int hc2 = presetPulse2[currentPresetIndex];
  if (hc2 > MAX_HALFCYCLES_PER_PULSE) hc2 = MAX_HALFCYCLES_PER_PULSE;
  if (hc2 % 2 != 0) hc2++; // always a whole number of full AC cycles - see zcdISR
  powerFraction = constrain(presetPower2[currentPresetIndex], 10, 100) / 100.0f;
  powerAccum = 0;
  halfCycleParity = false;
  halfCyclesRemaining = hc2;
  weldStartMicros = micros();
  weldActive = true;
  weldState = WS_PULSE2;
}

void finishWeld() {
  weldEndMs = millis();
  weldState = WS_COOLDOWN;
  totalWeldCount++;
  heatAccum += HEAT_PER_HALFCYCLE * (presetPulse1[currentPresetIndex] + presetPulse2[currentPresetIndex]);
  prefs.putULong("weldCount", totalWeldCount); // every weld - NVS wear isn't a real concern at this write rate
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