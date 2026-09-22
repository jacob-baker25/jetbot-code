#include <WiFi.h>
#include <WebServer.h>

namespace {
const char* AP_SSID = "Wheelchair-Joystick";
const char* AP_PASSWORD = "DriveSafe123";

const int PIN_AN1 = 6;
const int PIN_AN2 = 7;

const int PWM_STOP = 128;
const int PWM_FWD = 160;
const int PWM_REV = 96;

constexpr uint32_t COMMAND_TIMEOUT_MS = 300;
constexpr uint32_t STATUS_PRINT_MS = 1000;

enum CommandState {
  CMD_STOP,
  CMD_FORWARD,
  CMD_BACKWARD,
  CMD_LEFT,
  CMD_RIGHT
};

WebServer server(80);
CommandState activeCommand = CMD_STOP;
uint32_t lastCommandAtMs = 0;
uint32_t lastStatusAtMs = 0;

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
    .footer {
      margin-top: 16px;
      font-size: 0.92rem;
      color: #476050;
    }
  </style>
</head>
<body>
  <main class="panel">
    <h1>E-Joystick</h1>
    <p>Press and hold a direction. Releasing the button sends stop automatically.</p>
    <div class="status" id="status">Connecting...</div>
    <div class="pad">
      <div class="empty"></div>
      <button data-command="forward">Forward</button>
      <div class="empty"></div>
      <button data-command="left">Left</button>
      <button class="stop" id="stopButton" data-command="stop">Stop</button>
      <button data-command="right">Right</button>
      <div class="empty"></div>
      <button data-command="backward">Backward</button>
      <div class="empty"></div>
    </div>
    <div class="footer">Safety: if the page stops sending commands, the controller times out to stop.</div>
  </main>
  <script>
    const statusEl = document.getElementById('status');
    const buttons = Array.from(document.querySelectorAll('button[data-command]'));
    let heldCommand = 'stop';
    let sendTimer = null;

    async function sendCommand(command) {
      heldCommand = command;
      buttons.forEach((button) => {
        button.classList.toggle('active', button.dataset.command === command && command !== 'stop');
      });
      statusEl.textContent = 'Command: ' + command.toUpperCase();
      try {
        await fetch('/command?dir=' + encodeURIComponent(command), { cache: 'no-store' });
      } catch (error) {
        statusEl.textContent = 'Connection lost. Command timeout will stop motors.';
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

    fetch('/status')
      .then((response) => response.text())
      .then((text) => { statusEl.textContent = text; })
      .catch(() => { statusEl.textContent = 'Ready. Press and hold a direction.'; });
  </script>
</body>
</html>
)rawliteral";

const char* commandToText(CommandState command) {
  switch (command) {
    case CMD_FORWARD:
      return "FORWARD";
    case CMD_BACKWARD:
      return "BACKWARD";
    case CMD_LEFT:
      return "LEFT";
    case CMD_RIGHT:
      return "RIGHT";
    case CMD_STOP:
    default:
      return "STOP";
  }
}

void applyMotorCommand(CommandState command) {
  int leftPwm = PWM_STOP;
  int rightPwm = PWM_STOP;

  if (command == CMD_FORWARD) {
    leftPwm = PWM_REV;
    rightPwm = PWM_REV;
  } else if (command == CMD_BACKWARD) {
    leftPwm = PWM_FWD;
    rightPwm = PWM_FWD;
  } else if (command == CMD_LEFT) {
    leftPwm = PWM_REV;
    rightPwm = PWM_FWD;
  } else if (command == CMD_RIGHT) {
    leftPwm = PWM_FWD;
    rightPwm = PWM_REV;
  }

  analogWrite(PIN_AN1, leftPwm);
  analogWrite(PIN_AN2, rightPwm);
}

CommandState parseCommand(const String& value) {
  if (value == "forward") {
    return CMD_FORWARD;
  }
  if (value == "backward") {
    return CMD_BACKWARD;
  }
  if (value == "left") {
    return CMD_LEFT;
  }
  if (value == "right") {
    return CMD_RIGHT;
  }
  return CMD_STOP;
}

void handleRoot() {
  server.send_P(200, "text/html", INDEX_HTML);
}

void handleStatus() {
  server.send(200, "text/plain", String("Current command: ") + commandToText(activeCommand));
}

void handleCommand() {
  const String dir = server.hasArg("dir") ? server.arg("dir") : "stop";
  activeCommand = parseCommand(dir);
  lastCommandAtMs = millis();
  applyMotorCommand(activeCommand);

  server.send(200, "text/plain", String("OK ") + commandToText(activeCommand));
}

void handleNotFound() {
  server.send(404, "text/plain", "Not found");
}

void printAccessInfo() {
  Serial.println();
  Serial.println(F("ESP32 web joystick ready"));
  Serial.print(F("Connect phone/laptop to Wi-Fi SSID: "));
  Serial.println(AP_SSID);
  Serial.print(F("Password: "));
  Serial.println(AP_PASSWORD);
  Serial.print(F("Open this address in a browser: http://"));
  Serial.println(WiFi.softAPIP());
  Serial.println(F("Press and hold a direction. Releasing sends STOP."));
  Serial.println();
}
}  // namespace

void setup() {
  Serial.begin(115200);
  delay(500);

  pinMode(PIN_AN1, OUTPUT);
  pinMode(PIN_AN2, OUTPUT);
  applyMotorCommand(CMD_STOP);

  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASSWORD);

  server.on("/", HTTP_GET, handleRoot);
  server.on("/status", HTTP_GET, handleStatus);
  server.on("/command", HTTP_GET, handleCommand);
  server.onNotFound(handleNotFound);
  server.begin();

  printAccessInfo();
  lastCommandAtMs = millis();
}

void loop() {
  server.handleClient();

  const uint32_t now = millis();
  if (activeCommand != CMD_STOP && (now - lastCommandAtMs) > COMMAND_TIMEOUT_MS) {
    activeCommand = CMD_STOP;
    applyMotorCommand(activeCommand);
    Serial.println(F("Command timeout reached. Motors set to STOP."));
  }

  if ((now - lastStatusAtMs) > STATUS_PRINT_MS) {
    lastStatusAtMs = now;
    Serial.print(F("Active command: "));
    Serial.println(commandToText(activeCommand));
  }
}
