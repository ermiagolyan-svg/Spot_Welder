#include <Arduino.h>
#include <WiFi.h>
#include "WebPanel.h"
#include "WeldEngine.h"
#include "UI.h"
#include "Storage.h"
#include "Config.h"

bool wifiIsAP = false;
unsigned long lastWifiCheckMs = 0;
const unsigned long WIFI_RECONNECT_INTERVAL_MS = 20000;

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

void updateWifiHealth() {
  if (wifiIsAP) return;
  if (strlen(STA_SSID) == 0) return;
  unsigned long now = millis();
  if (now - lastWifiCheckMs < WIFI_RECONNECT_INTERVAL_MS) return;
  lastWifiCheckMs = now;
  if (WiFi.status() != WL_CONNECTED) {
    WiFi.disconnect();
    WiFi.begin(STA_SSID, STA_PASS); // non-blocking
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