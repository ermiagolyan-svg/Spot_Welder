#include "UI.h"
#include "WeldEngine.h"

// ================================ OLED Object ================================
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// ================================ External Variables ================================
extern bool wifiIsAP;

// ================================ UI Internal State ================================
int  currentScreen = SCR_HOME;
bool inEditMode = false;
unsigned long lastDisplayMs = 0;

// ================================ Encoder ================================
volatile long  encoderTicks = 0;
volatile uint8_t encPrevState = 0;
const int8_t encTable[16] = {
   0, -1,  1,  0,
   1,  0,  0, -1,
  -1,  0,  0,  1,
   0,  1, -1,  0
};

// ================================ Buttons ================================
bool sw1Stable = false, sw1Last = false;         unsigned long sw1LastChange = 0;
bool weldSwStable = false, weldSwLast = false;   unsigned long weldSwLastChange = 0;
bool encSwStable = false, encSwLast = false;     unsigned long encSwLastChange = 0;
unsigned long encSwPressStart = 0;
bool encSwLongHandled = false;

// ================================ Buzzer ================================
#define MAX_BUZZ_STEPS 6
int buzzSeq[MAX_BUZZ_STEPS];
int buzzLen = 0;
int buzzerPattern = 0;
int buzzerPatternStep = 0;
unsigned long buzzerPatternNextMs = 0;

// =====================================================================================
void IRAM_ATTR encoderISR() {
  uint8_t a = digitalRead(PIN_ENC_A);
  uint8_t b = digitalRead(PIN_ENC_B);
  uint8_t state = (a << 1) | b;
  uint8_t idx = ((encPrevState & 0x03) << 2) | (state & 0x03);
  encoderTicks += encTable[idx & 0x0F];
  encPrevState = state;
}

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
void handleUI() {
  static long lastProcessedTicks = 0;
  noInterrupts();
  long ticksNow = encoderTicks;
  interrupts();
  int delta = (int)((ticksNow - lastProcessedTicks) / 4);
  lastProcessedTicks += (long)delta * 4;

  bool shortPress = false, longPress = false;
  static bool wasPressed = false;
  
  if (encSwStable && !wasPressed) { wasPressed = true; }
  if (!encSwStable && wasPressed) {
    wasPressed = false;
    if (!encSwLongHandled) shortPress = true;
  }
  
  if (encSwStable && !encSwLongHandled && (millis() - encSwPressStart > LOCK_HOLD_MS)) {
    longPress = true;
    encSwLongHandled = true;
  }

  // قفل کردن دستگاه
  if (longPress) {
    deviceLocked = !deviceLocked;
    prefs.putBool("locked", deviceLocked);
    weldState = deviceLocked ? WS_LOCKED : WS_IDLE;
    return; // بوق منو حذف شد
  }

  // خروج از حالت خطا
  if (weldState == WS_FAULT) {
    if (shortPress) {
      lastError = ERR_NONE;
      weldState = deviceLocked ? WS_LOCKED : WS_IDLE;
    }
    return;
  }

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
          // بوق ذخیره منو حذف شد
          break;
      }
    }
  } else {
    switch (currentScreen) {
      case SCR_PRESET_SEL: currentPresetIndex = constrain(currentPresetIndex + delta, 0, NUM_PRESETS - 1); break;
      case SCR_EDIT_P1:    presetPulse1[currentPresetIndex] = constrain(presetPulse1[currentPresetIndex] + delta, 1, MAX_HALFCYCLES_PER_PULSE); break;
      case SCR_EDIT_PW1:   presetPower1[currentPresetIndex] = constrain(presetPower1[currentPresetIndex] + delta, 10, 100); break;
      case SCR_EDIT_DELAY: presetDelay[currentPresetIndex] = constrain(presetDelay[currentPresetIndex] + delta, 1, MAX_HALFCYCLES_PER_PULSE); break;
      case SCR_EDIT_P2:    presetPulse2[currentPresetIndex] = constrain(presetPulse2[currentPresetIndex] + delta, 0, MAX_HALFCYCLES_PER_PULSE); break;
      case SCR_EDIT_PW2:   presetPower2[currentPresetIndex] = constrain(presetPower2[currentPresetIndex] + delta, 10, 100); break;
    }
    if (shortPress) inEditMode = false;
  }
}

// =====================================================================================
void beepPattern(int pattern) {
  buzzerPatternStep = 0;
  switch (pattern) {
    case 1: buzzLen = 0; return;                                            // بوق استارت‌آپ حذف شد
    case 2: buzzLen = 1; buzzSeq[0] = 30; break;                            // شروع جوش (یک بوق بسیار کوتاه 30ms)
    case 3: buzzLen = 6; for(int i=0; i<6; i++) buzzSeq[i] = 80; break;     // خطای ZCD (هشدار ایمنی حفظ شده است)
    case 4: buzzLen = 1; buzzSeq[0] = 40; break;                            // پایان جوش (یک بوق کوتاه 40ms)
    case 5: buzzLen = 0; return;                                            // تمام بوق‌های رابط کاربری و انکودر غیرفعال شدند
    default: buzzLen = 0; return;
  }
  if (buzzLen > 0) {
    buzzerPattern = pattern;
    buzzerPatternNextMs = millis();
  }
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
// توابع کمکی برای رسم گرافیک
const char* stateName() {
  switch (weldState) {
    case WS_IDLE:        return "READY";
    case WS_PULSE1:      return "WELD P1";
    case WS_INTERDELAY:  return "DELAY";
    case WS_PULSE2:      return "WELD P2";
    case WS_COOLDOWN:    return "COOLING";
    case WS_LOCKED:      return "LOCKED";
    case WS_FAULT:       return "FAULT";
    default:             return "?";
  }
}

void drawEditScreen(const char* title, int value, const char* unit) {
  display.setTextSize(1);
  display.setCursor(0, 16);
  display.print(inEditMode ? ">> Edit " : "   Edit ");
  display.print(title);
  
  // نمایش مقدار با فونت بسیار بزرگ در مرکز
  display.setTextSize(3);
  display.setCursor(24, 32);
  display.printf("%02d", value);
  
  display.setTextSize(1);
  display.setCursor(display.getCursorX() + 4, 46);
  display.print(unit);
}

// =====================================================================================
void renderDisplay() {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  // پیام خطای تمام صفحه
  if (weldState == WS_FAULT) {
    display.setTextSize(2); display.setCursor(0, 10); display.println(" FAULT! ");
    display.setTextSize(1); display.setCursor(0, 35); display.println(lastError == ERR_ZCD_LOST ? " ZCD Signal Lost" : " Error");
    display.setCursor(0, 50); display.println(" Press Knob to Clear");
    display.display();
    return;
  }

  // --- نوار وضعیت بالای صفحه ---
  display.setTextSize(1);
  display.setCursor(0, 0);
  if (deviceLocked) display.print("LOCKED");
  else display.print(stateName());
  
  // فرکانس در گوشه بالا سمت راست
  display.setCursor(100, 0);
  display.print((int)acFrequencyHz); display.print("Hz");
  
  display.drawLine(0, 10, 127, 10, SSD1306_WHITE);
  // -----------------------------

  switch (currentScreen) {
    case SCR_HOME:
      // طراحی جدید صفحه اصلی: اعداد بزرگ و خوانا
      
      // Pulse 1
      display.setTextSize(1); display.setCursor(0, 16); display.print("P1:");
      display.setTextSize(2); display.setCursor(22, 16); display.printf("%02d", presetPulse1[currentPresetIndex]);
      display.setTextSize(1); display.print("hc ");
      display.printf("%03d%%", presetPower1[currentPresetIndex]);

      // Pulse 2
      display.setTextSize(1); display.setCursor(0, 36); display.print("P2:");
      display.setTextSize(2); display.setCursor(22, 36); 
      if (presetPulse2[currentPresetIndex] == 0) {
        display.print("OFF");
      } else {
        display.printf("%02d", presetPulse2[currentPresetIndex]);
        display.setTextSize(1); display.print("hc ");
        display.printf("%03d%%", presetPower2[currentPresetIndex]);
      }

      // اطلاعات پایین صفحه
      display.setTextSize(1); display.setCursor(0, 56);
      display.printf("[%s] Dly: %02dhc", presetName[currentPresetIndex], presetDelay[currentPresetIndex]);
      break;

    case SCR_PRESET_SEL:
      display.setTextSize(1); display.setCursor(0, 16);
      display.print(inEditMode ? ">> Preset" : "   Preset");
      display.setTextSize(2); display.setCursor(16, 36);
      display.print(presetName[currentPresetIndex]);
      break;

    case SCR_EDIT_P1:    drawEditScreen("Pulse 1", presetPulse1[currentPresetIndex], "hc"); break;
    case SCR_EDIT_PW1:   drawEditScreen("Power 1", presetPower1[currentPresetIndex], "%"); break;
    case SCR_EDIT_DELAY: drawEditScreen("Delay", presetDelay[currentPresetIndex], "hc"); break;
    case SCR_EDIT_P2:    drawEditScreen("Pulse 2", presetPulse2[currentPresetIndex], "hc"); break;
    case SCR_EDIT_PW2:   drawEditScreen("Power 2", presetPower2[currentPresetIndex], "%"); break;

    case SCR_SAVE:
      display.setTextSize(1); display.setCursor(0, 16); display.print("   Save Settings?");
      display.setTextSize(2); display.setCursor(24, 36); display.print("PRESS");
      break;

    case SCR_WIFI_INFO:
      display.setTextSize(1); display.setCursor(0, 14); display.print("   WiFi Status");
      display.setCursor(0, 28); display.printf("Mode: %s\n", wifiIsAP ? "AP" : "STA");
      display.setCursor(0, 40); display.printf("SSID: %s\n", wifiIsAP ? AP_SSID : STA_SSID);
      display.setCursor(0, 52); display.print("IP: "); display.print(wifiIsAP ? WiFi.softAPIP() : WiFi.localIP());
      break;

    case SCR_STATS:
      display.setTextSize(1); display.setCursor(0, 14); display.print("   Statistics");
      display.setCursor(0, 32); display.printf("Welds: %lu\n", totalWeldCount);
      display.setCursor(0, 46); display.printf("Heat : %d%%\n", (int)heatAccum);
      break;
  }
  
  display.display();
}