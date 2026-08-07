#include <Arduino.h>
#include "Storage.h"
#include "WeldEngine.h" // برای دسترسی به آرایه‌های پریست و prefs

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