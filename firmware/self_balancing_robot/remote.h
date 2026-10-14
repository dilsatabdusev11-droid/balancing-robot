// Phone remote: classic Bluetooth (Android pad apps) or a Wi-Fi web page that
// works in any phone browser, including iPhone. Chosen with REMOTE_MODE in
// config.h. Both deliver the same one-letter commands:
//   W/S/A/D drive, L stop driving, X emergency stop, G go.
#pragma once

struct RemoteStatus {
  const char* state;
  float angle;         // deg
  float speed;         // m/s, 0 without encoders
  float batteryVolts;  // 0 when the monitor is off
};

using RemoteCommandHandler = void (*)(char command);

// robotName becomes the Bluetooth name or the Wi-Fi network name.
void remoteBegin(const char* robotName, RemoteCommandHandler handler);
// Reads pending commands (and serves the web page). Never blocks for long.
void remotePoll(const RemoteStatus& status);
