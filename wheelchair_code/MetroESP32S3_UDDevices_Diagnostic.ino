#include <WiFi.h>

namespace {
const char* WIFI_SSID = "UD Devices";

constexpr uint32_t SERIAL_WAIT_MS = 5000;
constexpr uint32_t CONNECT_TIMEOUT_MS = 30000;
constexpr uint32_t RECONNECT_DELAY_MS = 10000;
constexpr uint32_t STATUS_PRINT_INTERVAL_MS = 1000;

uint32_t lastReconnectAttemptMs = 0;
uint32_t lastStatusPrintMs = 0;
bool hasPrintedConnectionDetails = false;

const char* statusToText(wl_status_t status) {
  switch (status) {
    case WL_IDLE_STATUS:
      return "WL_IDLE_STATUS";
    case WL_NO_SSID_AVAIL:
      return "WL_NO_SSID_AVAIL";
    case WL_SCAN_COMPLETED:
      return "WL_SCAN_COMPLETED";
    case WL_CONNECTED:
      return "WL_CONNECTED";
    case WL_CONNECT_FAILED:
      return "WL_CONNECT_FAILED";
    case WL_CONNECTION_LOST:
      return "WL_CONNECTION_LOST";
    case WL_DISCONNECTED:
      return "WL_DISCONNECTED";
    default:
      return "WL_UNKNOWN";
  }
}

void printDivider() {
  Serial.println(F("--------------------------------------------------"));
}

void printNetworkDetails() {
  printDivider();
  Serial.println(F("Connected to UD Devices"));
  Serial.print(F("Wi-Fi MAC: "));
  Serial.println(WiFi.macAddress());
  Serial.print(F("Local IP: "));
  Serial.println(WiFi.localIP());
  Serial.print(F("Gateway: "));
  Serial.println(WiFi.gatewayIP());
  Serial.print(F("Subnet: "));
  Serial.println(WiFi.subnetMask());
  Serial.print(F("DNS #1: "));
  Serial.println(WiFi.dnsIP(0));
  Serial.print(F("DNS #2: "));
  Serial.println(WiFi.dnsIP(1));
  Serial.print(F("RSSI: "));
  Serial.print(WiFi.RSSI());
  Serial.println(F(" dBm"));
  printDivider();
}

void printRegistrationInstructions() {
  printDivider();
  Serial.println(F("UD Devices setup process"));
  Serial.println(F("1. Copy the Wi-Fi MAC printed below."));
  Serial.println(F("2. On another device, sign in to UD ClearPass."));
  Serial.println(F("3. Create a device entry using this Wi-Fi MAC."));
  Serial.println(F("4. Select UD Devices for the network."));
  Serial.println(F("5. Wait a few minutes, then reset this board."));
  Serial.println(F("6. This sketch will report whether DHCP succeeds."));
  printDivider();
}

bool connectToUdDevices() {
  Serial.print(F("Attempting Wi-Fi connection to SSID: "));
  Serial.println(WIFI_SSID);

  WiFi.disconnect();
  delay(250);
  WiFi.begin(WIFI_SSID);

  const uint32_t startedAt = millis();
  wl_status_t lastStatus = WiFi.status();

  while ((millis() - startedAt) < CONNECT_TIMEOUT_MS) {
    const wl_status_t status = WiFi.status();
    if (status != lastStatus || (millis() - lastStatusPrintMs) >= STATUS_PRINT_INTERVAL_MS) {
      Serial.print(F("Status: "));
      Serial.println(statusToText(status));
      lastStatus = status;
      lastStatusPrintMs = millis();
    }

    if (status == WL_CONNECTED) {
      return true;
    }

    delay(250);
  }

  Serial.println(F("Connection attempt timed out."));
  return false;
}
}  // namespace

void setup() {
  Serial.begin(115200);

  const uint32_t serialStart = millis();
  while (!Serial && (millis() - serialStart) < SERIAL_WAIT_MS) {
    delay(10);
  }

  Serial.println();
  Serial.println(F("MetroESP32-S3 diagnostic sketch for UD Devices"));

  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);

  printRegistrationInstructions();

  Serial.print(F("Wi-Fi MAC for ClearPass registration: "));
  Serial.println(WiFi.macAddress());

  const bool connected = connectToUdDevices();
  if (connected) {
    printNetworkDetails();
    hasPrintedConnectionDetails = true;
  } else {
    Serial.println(F("Not connected. Double-check ClearPass registration, 2.4 GHz coverage, and timing."));
  }
}

void loop() {
  const wl_status_t status = WiFi.status();

  if (status == WL_CONNECTED) {
    if (!hasPrintedConnectionDetails) {
      printNetworkDetails();
      hasPrintedConnectionDetails = true;
    }

    delay(1000);
    return;
  }

  hasPrintedConnectionDetails = false;

  if ((millis() - lastReconnectAttemptMs) >= RECONNECT_DELAY_MS) {
    lastReconnectAttemptMs = millis();
    Serial.println(F("Wi-Fi disconnected. Retrying connection to UD Devices..."));
    connectToUdDevices();
  }

  delay(250);
}
