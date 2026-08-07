#include <Arduino.h>
#include <Wire.h>
#include "Config.h"
#include "WeldEngine.h"
#include "UI.h"
#include "Storage.h"
#include "WebPanel.h"

// =====================================================================================
//  SETUP
// =====================================================================================
void setup() {
  // SAFETY FIRST
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
  attachInterrupt(digitalPinToInterrupt(PIN_ZCD), zcdISR, RISING);

  Wire.begin(PIN_OLED_SDA, PIN_OLED_SCL);
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
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

  // Watchdog Timer
  esp_timer_create_args_t timerArgs = {};
  timerArgs.callback = &safetyTimerCallback;
  timerArgs.arg = NULL;
  timerArgs.dispatch_method = ESP_TIMER_TASK;
  timerArgs.name = "safety_wd";
  esp_timer_create(&timerArgs, &safetyTimer);
  esp_timer_start_periodic(safetyTimer, 5000); 

  lastHeatUpdateMs = millis();
  weldState = deviceLocked ? WS_LOCKED : WS_IDLE;

  beepPattern(1); // startup beep
}

// =====================================================================================
//  MAIN LOOP
// =====================================================================================
void loop() {
  server.handleClient();
  updateWifiHealth();
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