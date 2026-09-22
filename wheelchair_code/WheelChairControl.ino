#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include "esp_log.h"

#define SLAVE_ADDR 0x41
#define CENTER_DEADBAND_SQ 22500

const char* AP_SSID = "Wheelchair-Joystick";
const char* AP_PASSWORD = "DriveSafe123";

// H-Bridge PWM Pins
const int PIN_AN1 = 6;  // Motor Left
const int PIN_AN2 = 7;  // Motor Right

// Locked Anti-Phase Constants
const int PWM_STOP = 128;
const int PWM_FWD = 160;  // 25% Speed Forward (Hardware definition)
const int PWM_REV = 96;   // 25% Speed Reverse (Hardware definition)
const uint32_t WEB_COMMAND_TIMEOUT_MS = 300;
const uint8_t BASELINE_SAMPLE_COUNT = 25;

int16_t baseline[3] = {0, 0, 0};
int32_t baseline_accumulator[3] = {0, 0, 0};
bool baseline_established = false;
uint8_t baseline_samples_collected = 0;
uint32_t last_web_command_ms = 0;
WebServer server(80);

enum WebCommandState {
    WEB_STOP,
    WEB_FORWARD,
    WEB_BACKWARD,
    WEB_LEFT,
    WEB_RIGHT
};

WebCommandState web_command = WEB_STOP;

// Empirical Kinematic Centroids
const int16_t CENTROIDS[4][3] = {
    {130, 208, 477}, // 0: FORWARD
    {443, 457, 171}, // 1: BACKWARD
    {201, 443, 416}, // 2: LEFT
    {391, 144, 191}  // 3: RIGHT
};
const char* STATE_LABELS[4] = {"FORWARD", "BACKWARD", "LEFT", "RIGHT"};

const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1, viewport-fit=cover">
  <title>Wheelchair Joystick</title>
  <style>
    :root {
      --bg: #eef3ea;
      --panel: #fcfdf8;
      --ink: #16301f;
      --accent: #2c8a57;
      --accent-strong: #1f6a41;
      --danger: #c95a3d;
      --line: #c9d8c7;
    }
    * { box-sizing: border-box; }
    body {
      margin: 0;
      min-height: 100vh;
      font-family: Verdana, Geneva, sans-serif;
      background:
        radial-gradient(circle at top, #ffffff 0%, #eef3ea 45%, #d9e7d5 100%);
      color: var(--ink);
      display: grid;
      place-items: center;
      padding: 20px;
    }
    .panel {
      width: min(420px, 100%);
      background: rgba(252, 253, 248, 0.95);
      border: 1px solid var(--line);
      border-radius: 24px;
      box-shadow: 0 18px 40px rgba(22, 48, 31, 0.12);
      padding: 22px;
    }
    h1 {
      margin: 0 0 8px;
      font-size: 1.8rem;
      text-align: center;
    }
    p {
      margin: 0 0 18px;
      text-align: center;
      line-height: 1.4;
    }
    .status {
      border: 1px solid var(--line);
      border-radius: 16px;
      padding: 12px 14px;
      background: #f5f8f2;
      text-align: center;
      margin-bottom: 18px;
      font-weight: bold;
    }
    .pad {
      display: grid;
      grid-template-columns: repeat(3, 1fr);
      gap: 12px;
    }
    button {
      border: none;
      border-radius: 18px;
      min-height: 92px;
      font-size: 1.05rem;
      font-weight: bold;
      color: white;
      background: var(--accent);
      touch-action: manipulation;
      box-shadow: 0 10px 18px rgba(44, 138, 87, 0.18);
    }
    button:active,
    button.active {
      background: var(--accent-strong);
      transform: scale(0.98);
    }
    .empty {
      visibility: hidden;
    }
    .stop {
      background: var(--danger);
      box-shadow: 0 10px 18px rgba(201, 90, 61, 0.18);
    }
    .recenter {
      width: 100%;
      margin-top: 14px;
      min-height: 58px;
      background: #557868;
      box-shadow: 0 10px 18px rgba(85, 120, 104, 0.18);
    }
    .note {
      margin-top: 12px;
      text-align: center;
      font-size: 0.95rem;
      color: #476050;
      line-height: 1.4;
    }
  </style>
</head>
<body>
  <main class="panel">
    <h1>E-Joystick</h1>
    <p>Physical joystick has priority whenever it is pushed.</p>
    <div class="status" id="status">Connecting...</div>
    <div class="pad">
      <div class="empty"></div>
      <button data-command="forward">Forward</button>
      <div class="empty"></div>
      <button data-command="left">Left</button>
      <button class="stop" data-command="stop">Stop</button>
      <button data-command="right">Right</button>
      <div class="empty"></div>
      <button data-command="backward">Backward</button>
      <div class="empty"></div>
    </div>
    <button class="recenter" id="recenterButton" type="button">Recenter Physical Joystick</button>
    <div class="note">Use recenter only while the physical joystick is untouched and resting at neutral.</div>
  </main>
  <script>
    const statusEl = document.getElementById('status');
    const buttons = Array.from(document.querySelectorAll('button[data-command]'));
    const recenterButton = document.getElementById('recenterButton');
    let sendTimer = null;

    async function sendCommand(command) {
      buttons.forEach((button) => {
        button.classList.toggle('active', button.dataset.command === command && command !== 'stop');
      });
      statusEl.textContent = 'Command: ' + command.toUpperCase();
      try {
        const response = await fetch('/command?dir=' + encodeURIComponent(command), { cache: 'no-store' });
        const text = await response.text();
        statusEl.textContent = text;
      } catch (error) {
        statusEl.textContent = 'Connection lost. Timeout will stop motors.';
      }
    }

    function startHold(command) {
      sendCommand(command);
      if (sendTimer) {
        clearInterval(sendTimer);
      }
      sendTimer = setInterval(() => sendCommand(command), 120);
    }

    function endHold() {
      if (sendTimer) {
        clearInterval(sendTimer);
        sendTimer = null;
      }
      sendCommand('stop');
    }

    function bindHold(button) {
      const command = button.dataset.command;
      const start = (event) => {
        event.preventDefault();
        startHold(command);
      };
      const end = (event) => {
        event.preventDefault();
        endHold();
      };
      button.addEventListener('pointerdown', start);
      button.addEventListener('pointerup', end);
      button.addEventListener('pointerleave', end);
      button.addEventListener('pointercancel', end);
    }

    buttons.forEach(bindHold);
    window.addEventListener('beforeunload', endHold);
    window.addEventListener('blur', endHold);

    recenterButton.addEventListener('click', async () => {
      endHold();
      try {
        const response = await fetch('/recalibrate', { cache: 'no-store' });
        statusEl.textContent = await response.text();
      } catch (error) {
        statusEl.textContent = 'Could not start recalibration.';
      }
    });

    fetch('/status')
      .then((response) => response.text())
      .then((text) => { statusEl.textContent = text; })
      .catch(() => { statusEl.textContent = 'Ready'; });
  </script>
</body>
</html>
)rawliteral";

const char* web_state_label(WebCommandState state) {
    switch (state) {
        case WEB_FORWARD: return "FORWARD";
        case WEB_BACKWARD: return "BACKWARD";
        case WEB_LEFT: return "LEFT";
        case WEB_RIGHT: return "RIGHT";
        case WEB_STOP:
        default: return "STOP";
    }
}

WebCommandState parse_web_command(const String& value) {
    if (value == "forward") return WEB_FORWARD;
    if (value == "backward") return WEB_BACKWARD;
    if (value == "left") return WEB_LEFT;
    if (value == "right") return WEB_RIGHT;
    return WEB_STOP;
}

void stop_motors() {
    analogWrite(PIN_AN1, PWM_STOP);
    analogWrite(PIN_AN2, PWM_STOP);
}

void reset_baseline_calibration() {
    baseline_established = false;
    baseline_samples_collected = 0;
    baseline_accumulator[0] = 0;
    baseline_accumulator[1] = 0;
    baseline_accumulator[2] = 0;
}

bool sample_is_valid(int16_t p0, int16_t p2, int16_t p3) {
    return !(p0 == 0 && p2 == 0 && p3 == 0);
}

bool update_baseline_calibration(int16_t p0, int16_t p2, int16_t p3) {
    if (!sample_is_valid(p0, p2, p3)) {
        if (baseline_samples_collected != 0) {
            Serial.println("[BASELINE] Invalid sample detected. Restarting neutral calibration.");
        }
        reset_baseline_calibration();
        return false;
    }

    baseline_accumulator[0] += p0;
    baseline_accumulator[1] += p2;
    baseline_accumulator[2] += p3;
    baseline_samples_collected++;

    if (baseline_samples_collected < BASELINE_SAMPLE_COUNT) {
        if (baseline_samples_collected == 1 || baseline_samples_collected % 5 == 0) {
            Serial.printf("[BASELINE] Collecting neutral samples: %u/%u\n",
                          baseline_samples_collected, BASELINE_SAMPLE_COUNT);
        }
        return false;
    }

    baseline[0] = baseline_accumulator[0] / BASELINE_SAMPLE_COUNT;
    baseline[1] = baseline_accumulator[1] / BASELINE_SAMPLE_COUNT;
    baseline[2] = baseline_accumulator[2] / BASELINE_SAMPLE_COUNT;
    baseline_established = true;

    Serial.printf("[BASELINE LOCKED] Neutral calibrated at p0=%d p2=%d p3=%d\n",
                  baseline[0], baseline[1], baseline[2]);

    baseline_samples_collected = 0;
    baseline_accumulator[0] = 0;
    baseline_accumulator[1] = 0;
    baseline_accumulator[2] = 0;
    return true;
}

void handle_root() {
    server.send_P(200, "text/html", INDEX_HTML);
}

void handle_status() {
    String message;
    if (!baseline_established) {
        message = "Calibrating neutral joystick position. Leave the physical joystick untouched.";
        server.send(200, "text/plain", message);
        return;
    }

    message = "Current web command: ";
    message += web_state_label(web_command);
    server.send(200, "text/plain", message);
}

void handle_command() {
    const String dir = server.hasArg("dir") ? server.arg("dir") : "stop";
    web_command = parse_web_command(dir);
    last_web_command_ms = millis();

    String message = "Accepted: ";
    message += web_state_label(web_command);
    message += " | Physical joystick still overrides when active";
    server.send(200, "text/plain", message);
}

void handle_recalibrate() {
    web_command = WEB_STOP;
    last_web_command_ms = 0;
    stop_motors();
    reset_baseline_calibration();
    Serial.println("[BASELINE] Recalibration requested from web UI.");
    server.send(200, "text/plain", "Recalibrating neutral. Leave the physical joystick untouched for about one second.");
}

void handle_not_found() {
    server.send(404, "text/plain", "Not found");
}

void start_web_joystick() {
    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASSWORD);

    server.on("/", HTTP_GET, handle_root);
    server.on("/status", HTTP_GET, handle_status);
    server.on("/command", HTTP_GET, handle_command);
    server.on("/recalibrate", HTTP_GET, handle_recalibrate);
    server.onNotFound(handle_not_found);
    server.begin();

    Serial.println("[WEB JOYSTICK] Hotspot started.");
    Serial.print("[WEB JOYSTICK] SSID: ");
    Serial.println(AP_SSID);
    Serial.print("[WEB JOYSTICK] Password: ");
    Serial.println(AP_PASSWORD);
    Serial.print("[WEB JOYSTICK] Open: http://");
    Serial.println(WiFi.softAPIP());
}

void setup() {
    Serial.begin(115200);
    esp_log_level_set("i2c.master", ESP_LOG_NONE);
    esp_log_level_set("i2c", ESP_LOG_NONE);

    Wire.begin((int)SDA, (int)SCL);
    Wire.setClock(100000); 

    pinMode(PIN_AN1, OUTPUT);
    pinMode(PIN_AN2, OUTPUT);
    
    // Safety Startup
    stop_motors();

    start_web_joystick();
    reset_baseline_calibration();
    
    Serial.println("\n[SYSTEM ARMED] Physical joystick priority active. Web joystick enabled when centered.");
    Serial.println("[BASELINE] Leave the physical joystick untouched while neutral is calibrated.");
}

void feed_slave_watchdog() {
    Wire.beginTransmission(SLAVE_ADDR);
    Wire.write(0x2D); Wire.write(0x09); Wire.write(0x84);
    Wire.endTransmission();
}

void loop() {
    uint8_t current_map[256];
    server.handleClient();
    feed_slave_watchdog();

    // 1. Maintain established ASIL-B duty cycle
    for (int reg = 0; reg <= 255; reg++) {
        Wire.beginTransmission(SLAVE_ADDR);
        Wire.write(reg);
        if (Wire.endTransmission() == 0) {
            Wire.requestFrom(SLAVE_ADDR, 1);
            current_map[reg] = Wire.available() ? Wire.read() : 0x00;
        } else {
            current_map[reg] = 0x00; 
        }
    }

    // 2. Extract 16-bit primary spatial pipelines
    int16_t p0 = (int16_t)((current_map[0x16] << 8) | current_map[0x17]);
    int16_t p2 = (int16_t)((current_map[0x1A] << 8) | current_map[0x1B]);
    int16_t p3 = (int16_t)((current_map[0x1E] << 8) | current_map[0x1F]);

    if (!baseline_established) {
        stop_motors();
        update_baseline_calibration(p0, p2, p3);
        delay(20);
        return;
    }

    // 3. Vector Analysis
    int32_t d0 = (int32_t)p0 - baseline[0];
    int32_t d2 = (int32_t)p2 - baseline[1];
    int32_t d3 = (int32_t)p3 - baseline[2];

    int32_t magnitude_sq = (d0 * d0) + (d2 * d2) + (d3 * d3);
    String position = "CENTER";
    const bool physical_joystick_active = magnitude_sq > CENTER_DEADBAND_SQ;

    if ((millis() - last_web_command_ms) > WEB_COMMAND_TIMEOUT_MS) {
        web_command = WEB_STOP;
    }

    if (physical_joystick_active) {
        int best_match = 0;
        int32_t min_dist_sq = 2147483647; 

        for (int i = 0; i < 4; i++) {
            int32_t err0 = d0 - CENTROIDS[i][0];
            int32_t err2 = d2 - CENTROIDS[i][1];
            int32_t err3 = d3 - CENTROIDS[i][2];
            int32_t dist_sq = (err0 * err0) + (err2 * err2) + (err3 * err3);
            
            if (dist_sq < min_dist_sq) {
                min_dist_sq = dist_sq;
                best_match = i;
            }
        }
        position = STATE_LABELS[best_match];
    } else {
        if (web_command == WEB_FORWARD) {
            position = "FORWARD";
        } else if (web_command == WEB_BACKWARD) {
            position = "BACKWARD";
        } else if (web_command == WEB_LEFT) {
            position = "LEFT";
        } else if (web_command == WEB_RIGHT) {
            position = "RIGHT";
        }
    }

    // 4. Kinematic Discrete Routing (Y-Axis Hardware Inverted)
    uint8_t left_pwm = PWM_STOP;
    uint8_t right_pwm = PWM_STOP;

    if (position == "FORWARD") {
        left_pwm = PWM_REV;
        right_pwm = PWM_REV;
    } 
    else if (position == "BACKWARD") {
        left_pwm = PWM_FWD;
        right_pwm = PWM_FWD;
    } 
    else if (position == "LEFT") {
        left_pwm = PWM_REV; 
        right_pwm = PWM_FWD; 
    } 
    else if (position == "RIGHT") {
        left_pwm = PWM_FWD; 
        right_pwm = PWM_REV;
    } 
    else if (position == "CENTER") {
        left_pwm = PWM_STOP;
        right_pwm = PWM_STOP;
    }

    // 5. Hardware Actuation
    analogWrite(PIN_AN1, left_pwm);
    analogWrite(PIN_AN2, right_pwm);

    Serial.printf("[t=%05lu] STATE: %-10s | L_PWM: %03d | R_PWM: %03d\n", 
                  millis(), position.c_str(), left_pwm, right_pwm);

    delay(20); // 50Hz control loop limit
}
