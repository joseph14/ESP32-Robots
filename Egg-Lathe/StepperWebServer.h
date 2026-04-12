#pragma once

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <math.h>
#include <stdarg.h>

class StepperWebServer {
 public:
  explicit StepperWebServer(uint16_t port = 80)
      : _server(port) {}

  void beginAP(const char* ssid, const char* password = nullptr) {
    WiFi.mode(WIFI_AP);

    bool ok;
    if (password && strlen(password) >= 8) {
      ok = WiFi.softAP(ssid, password);
    } else {
      ok = WiFi.softAP(ssid);
    }

    _apIP = WiFi.softAPIP();

    Serial.println("[WebUI] ==============================");
    Serial.printf("[WebUI] softAP start: %s\n", ok ? "OK" : "FAILED");
    Serial.printf("[WebUI] SSID: %s\n", ssid);
    Serial.printf("[WebUI] Password: %s\n",
                  (password && strlen(password) >= 8) ? password : "(open)");
    Serial.printf("[WebUI] AP IP: %s\n", _apIP.toString().c_str());
    Serial.println("[WebUI] Open http://192.168.4.1");
    Serial.println("[WebUI] ==============================");

    setupRoutes();
    _server.begin();

    Serial.println("[WebUI] HTTP server started");
  }

  void setDebug(bool v) { _debug = v; }

  void loop() {
    _server.handleClient();

    if (_debug) {
      unsigned long now = millis();
      if (now - _lastStationLogMs >= 2000) {
        _lastStationLogMs = now;
        int count = WiFi.softAPgetStationNum();
        if (count != _lastStationCount) {
          _lastStationCount = count;
          Serial.printf("[WebUI] connected stations: %d\n", count);
        }
      }
    }
  }

  void setStepsPerRev(float v) { _stepsPerRev = v; }
  void setBaseSpeedStepsPerSec(float v) { _baseSpeedStepsPerSec = v; }
  void setSpeedMultiplier(float v) { _speedMultiplier = constrain(v, 0.0f, _maxSpeedMultiplier); }
  void setDirection(float v) { _direction = (v < 0.0f) ? -1.0f : 1.0f; }

  void setSelectedMoveAngleDeg(float v) { _selectedMoveAngleDeg = sanitizeMoveAngle(v); }
  void setMaxSpeedMultiplier(float v) { _maxSpeedMultiplier = max(0.01f, v); }

  float direction() const { return _direction; }
  float angleDeg() const { return _stepperAngleDeg; }
  float speedMultiplier() const { return _speedMultiplier; }
  float currentSpeedStepsPerSec() const { return _baseSpeedStepsPerSec * _speedMultiplier; }
  float selectedMoveAngleDeg() const { return _selectedMoveAngleDeg; }
  float maxSpeedMultiplier() const { return _maxSpeedMultiplier; }

  bool consumeTargetChanged() {
    bool v = _targetChanged;
    _targetChanged = false;
    return v;
  }

  bool consumeStopRequested() {
    bool v = _stopRequested;
    _stopRequested = false;
    return v;
  }

  long targetSteps() const {
    return lroundf(_stepperAngleDeg * _stepsPerRev / 360.0f);
  }

  IPAddress apIP() const { return _apIP; }

 private:
  WebServer _server;
  DNSServer _dns;
  IPAddress _apIP = IPAddress(192, 168, 4, 1);

  bool _debug = true;
  unsigned long _lastStationLogMs = 0;
  int _lastStationCount = -1;

  float _stepsPerRev = 4096.0f;
  float _baseSpeedStepsPerSec = 100.0f;
  float _speedMultiplier = 1.0f;
  float _maxSpeedMultiplier = 4.0f;
  float _direction = 1.0f;
  float _stepperAngleDeg = 0.0f;
  float _selectedMoveAngleDeg = 45.0f;

  bool _targetChanged = false;
  bool _stopRequested = false;

  static float sanitizeMoveAngle(float v) {
    const float allowed[] = {45.0f, 60.0f, 90.0f,180.0f, 360.0f};
    for (float a : allowed) {
      if (fabsf(v - a) < 0.5f) return a;
    }
    return 45.0f;
  }

  void markTargetChanged() {
    _targetChanged = true;
  }

  void logf(const char* fmt, ...) {
    if (!_debug) return;

    char buf[256];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    Serial.println(buf);
  }

  String clientIPString() {
    return _server.client().remoteIP().toString();
  }

  void logRequestSummary(const char* tag) {
    if (!_debug) return;

    Serial.printf(
      "[WebUI][%s] method=%s uri=%s host=%s client=%s args=%d\n",
      tag,
      (_server.method() == HTTP_GET) ? "GET" :
      (_server.method() == HTTP_POST) ? "POST" : "OTHER",
      _server.uri().c_str(),
      _server.hostHeader().c_str(),
      clientIPString().c_str(),
      _server.args()
    );

    for (int i = 0; i < _server.args(); ++i) {
      Serial.printf("  arg[%d] %s = %s\n",
                    i,
                    _server.argName(i).c_str(),
                    _server.arg(i).c_str());
    }
  }

  void logState(const char* prefix) {
    if (!_debug) return;

    Serial.printf(
      "[WebUI][%s] dir=%s moveAngle=%.2f angleDeg=%.2f targetSteps=%ld speedMul=%.4f speed=%.2f targetChanged=%d stopRequested=%d\n",
      prefix,
      (_direction > 0.0f) ? "Forward" : "Reverse",
      _selectedMoveAngleDeg,
      _stepperAngleDeg,
      targetSteps(),
      _speedMultiplier,
      currentSpeedStepsPerSec(),
      _targetChanged,
      _stopRequested
    );
  }

  void setupRoutes() {
    _server.on("/", HTTP_GET, [this]() {
      logRequestSummary("ROOT");
      _server.send(200, "text/html", htmlPage());
      logf("[WebUI][ROOT] served control page to %s", clientIPString().c_str());
    });

    _server.on("/cmd", HTTP_POST, [this]() {
      logRequestSummary("CMD");

      if (!_server.hasArg("name")) {
        logf("[WebUI][CMD] missing 'name' arg");
        _server.send(400, "text/plain", "Missing command name");
        return;
      }

      const String cmd = _server.arg("name");
      logf("[WebUI][CMD] received command: %s", cmd.c_str());
      handleCommand(cmd);
    });

    _server.on("/state", HTTP_GET, [this]() {
      logRequestSummary("STATE");

      String json = "{";
      json += "\"direction\":\"";
      json += (_direction > 0.0f) ? "Forward" : "Reverse";
      json += "\",";
      json += "\"directionSign\":";
      json += String(_direction > 0.0f ? 1 : -1);
      json += ",";
      json += "\"selectedMoveAngleDeg\":";
      json += String(_selectedMoveAngleDeg, 0);
      json += ",";
      json += "\"angleDeg\":";
      json += String(_stepperAngleDeg, 2);
      json += ",";
      json += "\"targetSteps\":";
      json += String(targetSteps());
      json += ",";
      json += "\"speedMultiplier\":";
      json += String(_speedMultiplier, 4);
      json += ",";
      json += "\"maxSpeedMultiplier\":";
      json += String(_maxSpeedMultiplier, 4);
      json += ",";
      json += "\"speedStepsPerSec\":";
      json += String(currentSpeedStepsPerSec(), 2);
      json += "}";

      _server.send(200, "application/json", json);
      logState("STATE_RESPONSE");
    });

    _server.onNotFound([this]() {
      logRequestSummary("404");
      logf("[WebUI][404] uri not found");
      _server.send(404, "text/plain", "Not found");
    });
  }

  void handleCommand(const String& cmd) {
    logState("CMD_BEFORE");

    bool ok = true;

    if (cmd == "move") {
      _stepperAngleDeg += _selectedMoveAngleDeg * _direction;
      markTargetChanged();
      logf("[WebUI][CMD] move applied");
    } else if (cmd == "dirForward") {
      _direction = 1.0f;
      logf("[WebUI][CMD] dirForward applied");
    } else if (cmd == "dirReverse") {
      _direction = -1.0f;
      logf("[WebUI][CMD] dirReverse applied");
    } else if (cmd == "setAngle") {
      if (!_server.hasArg("value")) {
        ok = false;
      } else {
        _selectedMoveAngleDeg = sanitizeMoveAngle(_server.arg("value").toFloat());
        logf("[WebUI][CMD] setAngle applied -> %.2f", _selectedMoveAngleDeg);
      }
    } else if (cmd == "setSpeedMultiplier") {
      if (!_server.hasArg("value")) {
        ok = false;
      } else {
        _speedMultiplier = constrain(_server.arg("value").toFloat(), 0.0f, _maxSpeedMultiplier);
        logf("[WebUI][CMD] setSpeedMultiplier applied -> %.4f", _speedMultiplier);
      }
    } else if (cmd == "stop") {
      _stopRequested = true;
      logf("[WebUI][CMD] stop applied");
    } else {
      ok = false;
      logf("[WebUI][CMD] unknown command: %s", cmd.c_str());
    }

    if (!ok) {
      _server.send(400, "text/plain", "Invalid command");
      return;
    }

    logState("CMD_AFTER");

    String json = "{";
    json += "\"ok\":true,";
    json += "\"command\":\"" + cmd + "\",";
    json += "\"direction\":\"";
    json += (_direction > 0.0f) ? "Forward" : "Reverse";
    json += "\",";
    json += "\"directionSign\":";
    json += String(_direction > 0.0f ? 1 : -1);
    json += ",";
    json += "\"selectedMoveAngleDeg\":";
    json += String(_selectedMoveAngleDeg, 0);
    json += ",";
    json += "\"angleDeg\":";
    json += String(_stepperAngleDeg, 2);
    json += ",";
    json += "\"targetSteps\":";
    json += String(targetSteps());
    json += ",";
    json += "\"speedMultiplier\":";
    json += String(_speedMultiplier, 4);
    json += ",";
    json += "\"maxSpeedMultiplier\":";
    json += String(_maxSpeedMultiplier, 4);
    json += ",";
    json += "\"speedStepsPerSec\":";
    json += String(currentSpeedStepsPerSec(), 2);
    json += "}";

    _server.send(200, "application/json", json);
    logf("[WebUI][CMD] response sent");
  }

  String htmlPage() const {
    return R"HTML(
<!doctype html>
<html>
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width,initial-scale=1"/>
  <title>Stepper Control</title>
<style>
  :root {
    --bg: #f4f5f7;
    --card: #ffffff;
    --border: #d9dde3;
    --text: #1f2937;
    --muted: #6b7280;
    --primary: #2563eb;
    --primary-soft: #dbeafe;
    --danger: #dc2626;
    --danger-soft: #fee2e2;
    --button: #eef2f7;
    --button-active: #dbeafe;
    --shadow: 0 2px 8px rgba(0,0,0,0.06);
    --radius: 16px;
  }

  * { box-sizing: border-box; }

  body {
    margin: 0;
    padding: 14px;
    font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif;
    background: var(--bg);
    color: var(--text);
  }

  .wrap {
    max-width: 520px;
    margin: 0 auto;
  }

  h1 {
    margin: 0 0 4px;
    font-size: 30px;
    line-height: 1.1;
  }

  .subtitle {
    margin: 0 0 12px;
    color: var(--muted);
    font-size: 14px;
  }

  .card {
    background: var(--card);
    border: 1px solid var(--border);
    border-radius: var(--radius);
    padding: 14px;
    margin-bottom: 12px;
    box-shadow: var(--shadow);
  }

  .card h2 {
    margin: 0 0 10px;
    font-size: 20px;
  }

  button, select, input[type="range"] {
    width: 100%;
  }

  button, select {
    border: 1px solid var(--border);
    border-radius: 14px;
    background: var(--button);
    color: var(--text);
    font-size: 18px;
    font-weight: 600;
    padding: 14px 12px;
    min-height: 58px;
    -webkit-tap-highlight-color: transparent;
  }

  button:active {
    transform: scale(0.99);
  }

  .primary {
    background: var(--primary-soft);
    border-color: #93c5fd;
    color: #1d4ed8;
    min-height: 76px;
    font-size: 28px;
  }

  .danger {
    background: var(--danger-soft);
    border-color: #fca5a5;
    color: var(--danger);
    min-height: 64px;
    font-size: 24px;
  }

  .control-row {
    display: grid;
    grid-template-columns: 1fr;
    gap: 12px;
  }

  .label {
    font-size: 14px;
    color: var(--muted);
    margin: 0 0 8px;
  }

  .move-top-row {
    display: grid;
    grid-template-columns: 72px minmax(0, 1fr);
    gap: 10px;
    align-items: center;
    margin-bottom: 12px;
  }

  .move-top-row select {
    min-width: 0;
    width: 100%;
    min-height: 52px;
    font-size: 18px;
    padding: 10px 12px;
  }

  .move-btn {
    width: 100%;
  }

  .dir-switch {
    display: flex;
    align-items: center;
    justify-content: center;
    width: 72px;
    height: 52px;
    margin: 0;
  }

  .dir-switch input {
    display: none;
  }

  .dir-slider {
    position: relative;
    width: 60px;
    height: 34px;
    border-radius: 999px;
    background: #dbeafe;
    border: 1px solid #93c5fd;
    transition: 0.2s ease;
    cursor: pointer;
  }

  .dir-slider::before {
    content: "";
    position: absolute;
    top: 2px;
    left: 2px;
    width: 28px;
    height: 28px;
    border-radius: 50%;
    background: #2563eb;
    transition: 0.2s ease;
  }

  .dir-switch input:checked + .dir-slider {
    background: #fee2e2;
    border-color: #fca5a5;
  }

  .dir-switch input:checked + .dir-slider::before {
    transform: translateX(26px);
    background: #dc2626;
  }

  .speed-line {
    display: flex;
    justify-content: space-between;
    align-items: center;
    gap: 8px;
    margin-top: 8px;
    font-size: 14px;
    color: var(--muted);
  }

  input[type="range"] {
    accent-color: var(--primary);
    min-height: 38px;
    margin: 0;
  }

  .status-card {
    padding-top: 10px;
    padding-bottom: 10px;
  }

  .status-line {
    display: flex;
    flex-wrap: wrap;
    gap: 8px 14px;
    font-size: 14px;
    color: var(--text);
  }

  .status-chip {
    padding: 6px 10px;
    border: 1px solid var(--border);
    border-radius: 999px;
    background: #f8fafc;
  }

  .offline {
    color: var(--danger);
  }
</style>
</head>
<body>
  <div class="wrap">


<div class="card">
  <h2>Move</h2>

  <div class="move-top-row">
    <label class="dir-switch" title="Forward / Reverse">
      <input id="dirToggle" type="checkbox" onchange="setDirectionFromToggle()" />
      <span class="dir-slider"></span>
    </label>

    <select id="angleSelect" onchange="setAngleFromUi()">
      <option value="45">45°</option>
      <option value="60">60°</option>
      <option value="90">90°</option>
      <option value="180">180°</option>
      <option value="360">360°</option>
    </select>
  </div>

  <button class="primary move-btn" onclick="sendCmd('move')">Move</button>
</div>

    <div class="card">
      <h2>Speed</h2>
      <div class="label">0 to max speed</div>
      <input id="speedSlider" type="range" min="0" max="1000" step="1" value="250" oninput="setSpeedFromUi()" />
      <div class="speed-line">
        <span>0</span>
        <span id="speedText">--</span>
        <span>Max</span>
      </div>
    </div>

    <div class="card">
      <button class="danger" onclick="sendCmd('stop')">Stop</button>
    </div>

    <div class="card status-card">
      <div class="status-line">
        <div class="status-chip" id="statusDir">Dir: --</div>
        <div class="status-chip" id="statusMove">Move: --</div>
        <div class="status-chip" id="statusSpeed">Speed: --</div>
        <div class="status-chip" id="statusAngle">Target: --</div>
        <div class="status-chip" id="statusSteps">Steps: --</div>
      </div>
    </div>
  </div>

<script>
let suppressSliderSend = false;

async function postCommand(name, value) {
  const body = new URLSearchParams();
  body.append("name", name);
  if (value !== undefined && value !== null) {
    body.append("value", String(value));
  }

  const r = await fetch("/cmd", {
    method: "POST",
    headers: { "Content-Type": "application/x-www-form-urlencoded" },
    body
  });

  if (!r.ok) {
    throw new Error("HTTP " + r.status);
  }
  return await r.json();
}

async function sendCmd(name) {
  try {
    await postCommand(name);
  } catch (e) {
    console.log(e);
  }
  refreshState();
}

async function setAngleFromUi() {
  const angle = document.getElementById("angleSelect").value;
  try {
    await postCommand("setAngle", angle);
  } catch (e) {
    console.log(e);
  }
  refreshState();
}

async function setSpeedFromUi() {
  if (suppressSliderSend) return;

  const slider = document.getElementById("speedSlider");
  const raw = Number(slider.value);
  const maxRaw = Number(slider.max);
  const multiplier = maxRaw > 0 ? (raw / maxRaw) * window._maxSpeedMultiplier : 0;

  document.getElementById("speedText").textContent = raw + " / " + maxRaw;

  try {
    await postCommand("setSpeedMultiplier", multiplier.toFixed(4));
  } catch (e) {
    console.log(e);
  }
}

async function setDirectionFromToggle() {
  const toggle = document.getElementById("dirToggle");
  const cmd = toggle.checked ? "dirReverse" : "dirForward";

  try {
    await postCommand(cmd);
  } catch (e) {
    console.log(e);
  }

  refreshState();
}

async function refreshState() {
  try {
    const r = await fetch("/state");
    const s = await r.json();

    window._maxSpeedMultiplier = Number(s.maxSpeedMultiplier || 4.0);

    document.getElementById("btnForward").classList.toggle("selected", s.directionSign > 0);
    document.getElementById("btnReverse").classList.toggle("selected", s.directionSign < 0);

    document.getElementById("angleSelect").value = String(Number(s.selectedMoveAngleDeg).toFixed(0));

    const slider = document.getElementById("speedSlider");
    const sliderMax = Number(slider.max);
    const maxMul = Number(s.maxSpeedMultiplier || 1);
    const mul = Number(s.speedMultiplier || 0);
    const sliderValue = maxMul > 0 ? Math.round((mul / maxMul) * sliderMax) : 0;

    suppressSliderSend = true;
    slider.value = String(sliderValue);
    suppressSliderSend = false;

    document.getElementById("speedText").textContent =
      Number(s.speedStepsPerSec).toFixed(1) + " steps/s";

    document.getElementById("statusDir").textContent = "Dir: " + s.direction;
    document.getElementById("statusMove").textContent = "Move: " + Number(s.selectedMoveAngleDeg).toFixed(0) + "°";
    document.getElementById("statusSpeed").textContent = "Speed: " + Number(s.speedStepsPerSec).toFixed(1);
    document.getElementById("statusAngle").textContent = "Target: " + Number(s.angleDeg).toFixed(1) + "°";
    document.getElementById("statusSteps").textContent = "Steps: " + s.targetSteps;
  } catch (e) {
    document.getElementById("statusDir").textContent = "Disconnected";
    document.getElementById("statusDir").classList.add("offline");
    document.getElementById("statusMove").textContent = "Move: --";
    document.getElementById("statusSpeed").textContent = "Speed: --";
    document.getElementById("statusAngle").textContent = "Target: --";
    document.getElementById("statusSteps").textContent = "Steps: --";
    document.getElementById("speedText").textContent = "--";
  }
}

setInterval(refreshState, 800);
refreshState();
</script>
</body>
</html>
)HTML";
  }
};