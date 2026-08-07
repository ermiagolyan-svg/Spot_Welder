#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>
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

// ================================ WiFi ================================
constexpr const char* STA_SSID = "";
constexpr const char* STA_PASS = "";
constexpr const char* AP_SSID  = "SpotWelder";
constexpr const char* AP_PASS  = "weld12345";

// ================================ Safety / config constants ================================
const int           MAX_HALFCYCLES_PER_PULSE = 40;
const unsigned long ABSOLUTE_MAX_SEGMENT_US  = 1000000UL;
const unsigned long ZCD_TIMEOUT_US           = 30000UL;
const unsigned long MIN_COOLDOWN_MS          = 800;
const unsigned long DEBOUNCE_MS              = 25;
const unsigned long LOCK_HOLD_MS             = 3000;
const int           NUM_PRESETS              = 6;
const float         HEAT_MAX                 = 100.0;
const float         HEAT_PER_HALFCYCLE       = 1.2;
const float         HEAT_DECAY_PER_S         = 3.0;

// ================================ State machine + error codes ================================
#define WS_IDLE        0
#define WS_PULSE1      1
#define WS_INTERDELAY  2
#define WS_PULSE2      3
#define WS_COOLDOWN    4
#define WS_LOCKED      5
#define WS_FAULT       6

#define ERR_NONE      0
#define ERR_ZCD_LOST  1

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

#endif // CONFIG_H