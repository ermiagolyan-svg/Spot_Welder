#ifndef WELD_ENGINE_H
#define WELD_ENGINE_H

#include <Arduino.h>
#include <Preferences.h>
#include <WebServer.h>
#include "esp_timer.h"
#include "Config.h"

// ================================ Objects ================================
extern Preferences prefs;
extern WebServer   server;
extern esp_timer_handle_t safetyTimer;

// ================================ Presets ================================
extern int  presetPulse1[NUM_PRESETS];
extern int  presetPower1[NUM_PRESETS];
extern int  presetDelay[NUM_PRESETS];
extern int  presetPulse2[NUM_PRESETS];
extern int  presetPower2[NUM_PRESETS];
extern char presetName[NUM_PRESETS][10];
extern int  currentPresetIndex;

// ================================ Engine State Variables ================================
extern int weldState;
extern int lastError;
extern float acFrequencyHz;
extern float heatAccum;
extern unsigned long totalWeldCount;
extern bool deviceLocked;
extern unsigned long lastHeatUpdateMs; // <--- این خط اضافه شد

// ================================ Functions ================================
bool triggerActive();
void IRAM_ATTR zcdISR();
void safetyTimerCallback(void* arg);
void startWeld();
void startPulse2();
void finishWeld();
unsigned long halfCycleToMs(int halfCycles);
void handleWeldStateMachine();
void updateFrequencyEstimate();

#endif // WELD_ENGINE_H