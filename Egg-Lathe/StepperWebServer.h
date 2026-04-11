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
  Serial.printf("[WebUI] Password: %s\n", (password && strlen(password) >= 8) ? password : "(open)");
  Serial.printf("[WebUI] AP IP: %s\n", _apIP.toString().c_str());
  Serial.println("[WebUI] Open http://192.168.4.1");
  Serial.println("[WebUI] ==============================");

  setupRoutes();
  _server.begin();

  Serial.println("[WebUI] HTTP server started");
}

  void setDebug(bool v) { _debug = v; }

  unsigned long _lastStationLogMs = 0;
int _lastStationCount = -1;

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
  void setSpeedMultiplier(float v) { _speedMultiplier = v; }
  void setDirection(float v) { _direction = (v < 0.0f) ? -1.0f : 1.0f; }

  float direction() const { return _direction; }
  float angleDeg() const { return _stepperAngleDeg; }
  float speedMultiplier() const { return _speedMultiplier; }
  float currentSpeedStepsPerSec() const { return _baseSpeedStepsPerSec * _speedMultiplier; }

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


bool _debug = true;


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
    Serial.printf(
      "  arg[%d] %s = %s\n",
      i,
      _server.argName(i).c_str(),
      _server.arg(i).c_str()
    );
  }
}

void logState(const char* prefix) {
  if (!_debug) return;

  Serial.printf(
    "[WebUI][%s] dir=%s angleDeg=%.2f targetSteps=%ld speedMul=%.4f speed=%.2f targetChanged=%d stopRequested=%d\n",
    prefix,
    (_direction > 0.0f) ? "Forward" : "Reverse",
    _stepperAngleDeg,
    targetSteps(),
    _speedMultiplier,
    currentSpeedStepsPerSec(),
    _targetChanged,
    _stopRequested
  );
}

  WebServer _server;
  DNSServer _dns;
  IPAddress _apIP = IPAddress(192, 168, 4, 1);

  float _stepsPerRev = 4096.0f;
  float _baseSpeedStepsPerSec = 100.0f;
  float _speedMultiplier = 1.0f;
  float _direction = 1.0f;
  float _stepperAngleDeg = 0.0f;

  bool _targetChanged = false;
  bool _stopRequested = false;

  void markTargetChanged() {
    _targetChanged = true;
  }

  bool captivePortal() {
    String host = _server.hostHeader();

    if (host.length() == 0) return false;
    if (host == _apIP.toString()) return false;
    if (host == String(_apIP.toString()) + ":80") return false;

    _server.sendHeader("Location", String("http://") + _apIP.toString() + "/", true);
    _server.send(302, "text/plain", "");
    return true;
  }

  void sendPortalPage() {
    _server.send(200, "text/html", htmlPage());
  }

  void sendPlainPortalPage() {
    _server.send(200, "text/html", htmlPage());
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
    json += "\"angleDeg\":";
    json += String(_stepperAngleDeg, 2);
    json += ",";
    json += "\"targetSteps\":";
    json += String(targetSteps());
    json += ",";
    json += "\"speedMultiplier\":";
    json += String(_speedMultiplier, 4);
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

  if (cmd == "move360") {
    _stepperAngleDeg += 360.0f * _direction;
    markTargetChanged();
    logf("[WebUI][CMD] move360 applied");
  } else if (cmd == "move90") {
    _stepperAngleDeg += 90.0f * _direction;
    markTargetChanged();
    logf("[WebUI][CMD] move90 applied");
  } else if (cmd == "move60") {
    _stepperAngleDeg += 60.0f * _direction;
    markTargetChanged();
    logf("[WebUI][CMD] move60 applied");
  } else if (cmd == "move45") {
    _stepperAngleDeg += 45.0f * _direction;
    markTargetChanged();
    logf("[WebUI][CMD] move45 applied");
  } else if (cmd == "speedUp") {
    _speedMultiplier *= 1.1f;
    logf("[WebUI][CMD] speedUp applied");
  } else if (cmd == "speedDown") {
    _speedMultiplier /= 1.1f;
    logf("[WebUI][CMD] speedDown applied");
  } else if (cmd == "dirForward") {
    _direction = 1.0f;
    logf("[WebUI][CMD] dirForward applied");
  } else if (cmd == "dirReverse") {
    _direction = -1.0f;
    logf("[WebUI][CMD] dirReverse applied");
  } else if (cmd == "zeroAngle") {
    _stepperAngleDeg = 0.0f;
    markTargetChanged();
    logf("[WebUI][CMD] zeroAngle applied");
  } else if (cmd == "stop") {
    _stopRequested = true;
    logf("[WebUI][CMD] stop applied");
  } else {
    ok = false;
    logf("[WebUI][CMD] unknown command: %s", cmd.c_str());
  }

  if (!ok) {
    _server.send(400, "text/plain", "Unknown command");
    return;
  }

  logState("CMD_AFTER");

  String json = "{";
  json += "\"ok\":true,";
  json += "\"command\":\"" + cmd + "\",";
  json += "\"direction\":\"";
  json += (_direction > 0.0f) ? "Forward" : "Reverse";
  json += "\",";
  json += "\"angleDeg\":";
  json += String(_stepperAngleDeg, 2);
  json += ",";
  json += "\"targetSteps\":";
  json += String(targetSteps());
  json += ",";
  json += "\"speedMultiplier\":";
  json += String(_speedMultiplier, 4);
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
      padding: 16px;
      font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif;
      background: var(--bg);
      color: var(--text);
    }

    .wrap {
      max-width: 720px;
      margin: 0 auto;
    }

    h1 {
      margin: 0 0 6px;
      font-size: 32px;
      line-height: 1.1;
    }

    .subtitle {
      margin: 0 0 16px;
      color: var(--muted);
      font-size: 15px;
    }

    .card {
      background: var(--card);
      border: 1px solid var(--border);
      border-radius: var(--radius);
      padding: 16px;
      margin-bottom: 14px;
      box-shadow: var(--shadow);
    }

    .card h2 {
      margin: 0 0 12px;
      font-size: 22px;
    }

    .status-grid {
      display: grid;
      grid-template-columns: 1fr 1fr;
      gap: 10px;
    }

    .stat {
      background: #f8fafc;
      border: 1px solid var(--border);
      border-radius: 12px;
      padding: 12px;
    }

    .stat-label {
      font-size: 13px;
      color: var(--muted);
      margin-bottom: 6px;
    }

    .stat-value {
      font-size: 22px;
      font-weight: 700;
    }

    .button-grid {
      display: grid;
      grid-template-columns: 1fr 1fr;
      gap: 12px;
    }

    button {
      width: 100%;
      min-height: 64px;
      border: 1px solid var(--border);
      border-radius: 14px;
      background: var(--button);
      color: var(--text);
      font-size: 18px;
      font-weight: 600;
      padding: 12px;
      -webkit-tap-highlight-color: transparent;
    }

    button:active {
      transform: scale(0.99);
    }

    .selected {
      background: var(--button-active);
      border-color: var(--primary);
      color: var(--primary);
    }

    .primary {
      background: var(--primary-soft);
      border-color: #93c5fd;
      color: #1d4ed8;
    }

    .danger {
      background: var(--danger-soft);
      border-color: #fca5a5;
      color: var(--danger);
    }

    .wide {
      grid-column: span 2;
    }

    .pill-row {
      display: grid;
      grid-template-columns: 1fr 1fr;
      gap: 12px;
    }

    .hint {
      margin-top: 8px;
      font-size: 13px;
      color: var(--muted);
    }
  </style>
</head>
<body>
  <div class="wrap">
    <h1>Stepper Control</h1>
    <p class="subtitle">Connected to 192.168.4.1</p>

    <div class="card">
      <h2>Status</h2>
      <div class="status-grid">
        <div class="stat">
          <div class="stat-label">Direction</div>
          <div class="stat-value" id="dirValue">Forward</div>
        </div>
        <div class="stat">
          <div class="stat-label">Speed</div>
          <div class="stat-value" id="speedValue">--</div>
        </div>
        <div class="stat">
          <div class="stat-label">Target Angle</div>
          <div class="stat-value" id="angleValue">0&deg;</div>
        </div>
        <div class="stat">
          <div class="stat-label">Target Steps</div>
          <div class="stat-value" id="stepsValue">0</div>
        </div>
      </div>
    </div>

    <div class="card">
      <h2>Direction</h2>
      <div class="pill-row">
        <button id="btnForward" onclick="sendCmd('dirForward')">Forward</button>
        <button id="btnReverse" onclick="sendCmd('dirReverse')">Reverse</button>
      </div>
      <div class="hint">Direction affects the next rotate command.</div>
    </div>

    <div class="card">
      <h2>Rotate</h2>
      <div class="button-grid">
        <button class="primary" onclick="sendCmd('move45')">45&deg;</button>
        <button class="primary" onclick="sendCmd('move60')">60&deg;</button>
        <button class="primary" onclick="sendCmd('move90')">90&deg;</button>
        <button class="primary" onclick="sendCmd('move360')">360&deg;</button>
      </div>
    </div>

    <div class="card">
      <h2>Speed</h2>
      <div class="button-grid">
        <button onclick="sendCmd('speedDown')">Slower</button>
        <button onclick="sendCmd('speedUp')">Faster</button>
      </div>
    </div>

    <div class="card">
      <h2>Actions</h2>
      <div class="button-grid">
        <button onclick="sendCmd('zeroAngle')">Zero Target</button>
        <button class="danger" onclick="sendCmd('stop')">Stop</button>
      </div>
    </div>
  </div>

<script>
async function sendCmd(name) {
  const body = new URLSearchParams();
  body.append("name", name);

  try {
    await fetch("/cmd", {
      method: "POST",
      headers: { "Content-Type": "application/x-www-form-urlencoded" },
      body
    });
  } catch (e) {
    console.log(e);
  }

  refreshState();
}

async function refreshState() {
  try {
    const r = await fetch("/state");
    const s = await r.json();

    document.getElementById("dirValue").textContent = s.direction;
    document.getElementById("speedValue").textContent = Number(s.speedStepsPerSec).toFixed(1);
    document.getElementById("angleValue").innerHTML = Number(s.angleDeg).toFixed(0) + "&deg;";
    document.getElementById("stepsValue").textContent = s.targetSteps;

    document.getElementById("btnForward").classList.toggle("selected", s.directionSign > 0);
    document.getElementById("btnReverse").classList.toggle("selected", s.directionSign < 0);
  } catch (e) {
    document.getElementById("dirValue").textContent = "Disconnected";
    document.getElementById("speedValue").textContent = "--";
    document.getElementById("angleValue").innerHTML = "--";
    document.getElementById("stepsValue").textContent = "--";
  }
}

setInterval(refreshState, 1000);
refreshState();
</script>
</body>
</html>
)HTML";
}
};