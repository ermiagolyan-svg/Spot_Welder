#ifndef UI_H
#define UI_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "Config.h"

// ================================ Display Object ================================
extern Adafruit_SSD1306 display;

// ================================ Shared Variables ================================
// این متغیرها توسط WeldEngine و فایل اصلی خوانده می‌شوند
extern bool sw1Stable;
extern bool weldSwStable;
extern unsigned long lastDisplayMs;

// ================================ Functions ================================
void IRAM_ATTR encoderISR();
void updateButtons();
void handleUI();
void beepPattern(int pattern);
void updateBuzzer();
void renderDisplay();
const char* stateName();

// ================================ External Dependencies ================================
// این توابع در فایل اصلی یا ماژول‌های دیگر پیاده‌سازی می‌شوند
extern void savePreset(int idx);

#endif // UI_H