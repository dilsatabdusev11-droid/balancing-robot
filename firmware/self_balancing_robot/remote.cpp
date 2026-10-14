#include "remote.h"

#include <Arduino.h>

#include "config.h"

#if REMOTE_MODE == REMOTE_BLUETOOTH
#include "BluetoothSerial.h"
#elif REMOTE_MODE == REMOTE_WIFI
#include <WebServer.h>
#include <WiFi.h>

#include "remote_page.h"
#else
#error "REMOTE_MODE must be REMOTE_BLUETOOTH or REMOTE_WIFI (see config.h)"
#endif

namespace {

RemoteCommandHandler commandHandler = nullptr;

#if REMOTE_MODE == REMOTE_BLUETOOTH

BluetoothSerial bluetooth;

}  // namespace

void remoteBegin(const char* robotName, RemoteCommandHandler handler) {
  commandHandler = handler;
  if (!bluetooth.begin(robotName)) {
    Serial.println("! Bluetooth failed to start.");
    return;
  }
  Serial.printf("Bluetooth is on. Pair your phone with '%s' and use a pad app\n", robotName);
  Serial.println("(for example \"Arduino Bluetooth Controller\", Android only).");
}

void remotePoll(const RemoteStatus&) {
  while (bluetooth.available() > 0) {
    commandHandler(static_cast<char>(toupper(bluetooth.read())));
  }
}

#else  // REMOTE_WIFI

WebServer server(80);
RemoteStatus latestStatus = {"?", 0.0f, 0.0f, 0.0f};

void handleCommand() {
  const String command = server.arg("c");
  if (command.length() != 1) {
    server.send(400, "text/plain", "expected ?c=<one letter>");
    return;
  }
  commandHandler(static_cast<char>(toupper(command[0])));
  server.send(204);
}

void handleStatus() {
  char json[128];
  snprintf(json, sizeof(json), "{\"state\":\"%s\",\"angle\":%.2f,\"speed\":%.2f,\"battery\":%.2f}",
           latestStatus.state, latestStatus.angle, latestStatus.speed, latestStatus.batteryVolts);
  server.send(200, "application/json", json);
}

}  // namespace

void remoteBegin(const char* robotName, RemoteCommandHandler handler) {
  commandHandler = handler;
  WiFi.mode(WIFI_AP);
  if (!WiFi.softAP(robotName, WIFI_PASSWORD)) {
    Serial.println("! Wi-Fi access point failed to start (password needs 8+ characters).");
    return;
  }
  server.on("/", [] { server.send(200, "text/html", kRemotePage); });
  server.on("/cmd", handleCommand);
  server.on("/status", handleStatus);
  server.onNotFound([] { server.send(404, "text/plain", "not found"); });
  server.begin();
  Serial.printf("Wi-Fi is on. Connect the phone to '%s' (password '%s')\n", robotName, WIFI_PASSWORD);
  Serial.printf("and open http://%s in the browser.\n", WiFi.softAPIP().toString().c_str());
}

void remotePoll(const RemoteStatus& status) {
  latestStatus = status;
  server.handleClient();
}

#endif
