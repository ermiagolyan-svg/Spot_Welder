#include <Arduino.h>
#include "Storage.h"
#include "WeldEngine.h" // برای دسترسی به آرایه‌های پریست و prefs

void loadAllPresets() {
  // مقادیر پیش‌فرض بر اساس ضخامت ورق نیکل
  const char* defNames[] = {"0.10mm", "0.12mm", "0.15mm", "0.20mm", "0.25mm", "Custom"};
  const int defP1[]  = {2, 2, 3, 4, 5, 2};     // پالس پیش‌جوش
  const int defP2[]  = {4, 6, 10, 15, 25, 0};  // پالس اصلی (پالس صفر یعنی حالت تک‌پالس)
  const int defDly[] = {5, 5, 5, 5, 5, 3};     // تاخیر بین دو پالس

  for (int i = 0; i < NUM_PRESETS; i++) {
    String base = "p" + String(i) + "_";
    presetPulse1[i] = prefs.getInt((base + "p1").c_str(), defP1[i]);
    presetPower1[i] = prefs.getInt((base + "pw1").c_str(), 100);
    presetDelay[i]  = prefs.getInt((base + "dl").c_str(), defDly[i]);
    presetPulse2[i] = prefs.getInt((base + "p2").c_str(), defP2[i]);
    presetPower2[i] = prefs.getInt((base + "pw2").c_str(), 100);
    String nm = prefs.getString((base + "nm").c_str(), defNames[i]);
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