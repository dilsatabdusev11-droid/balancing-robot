// USB Serial console (second tab of the sketch: the Arduino IDE joins all .ino
// files of the folder, so the globals of self_balancing_robot.ino are shared).

// ---------------------------------------------------------------------------
// USB Serial commands. Characters are collected into a line without waiting,
// so typing a command never pauses the balance loop (Serial.parseFloat()
// used to block for up to 1 s and the robot fell over).
// ---------------------------------------------------------------------------
void handleCommandLine(const char* line) {
  Command cmd;
  switch (parseCommand(line, cmd)) {
    case ParseStatus::Empty: return;
    case ParseStatus::BadName:
    case ParseStatus::BadValue:
      Serial.printf("! Cannot understand '%s'. Type 'help'.\n", line);
      return;
    case ParseStatus::Ok: break;
  }

  if (handleActionCommand(cmd)) return;

  // "save" keeps the learned auto-trim correction as part of baseAngle.
  if (strcmp(cmd.name, "save") == 0) {
    settings.baseAngle = clampf(settings.baseAngle + autoTrim.offset, -45.0f, 45.0f);
    autoTrim.reset();
  }
  const float oldKi = settings.ki;
  const float oldBase = settings.baseAngle;
  const bool oldAutoTrim = settings.autoTrim;
  if (!handleSettingsCommand(cmd, settings, Serial)) {
    Serial.printf("! Unknown command '%s'. Type 'help'.\n", cmd.name);
    return;
  }
  if (settings.ki != oldKi) pid.reset();  // a big Ki change must not cause a kick
  if (settings.baseAngle != oldBase || settings.autoTrim != oldAutoTrim) autoTrim.reset();
}

// Commands that act on the robot rather than change a setting.
bool handleActionCommand(const Command& cmd) {
  const char* name = cmd.name;
  const bool balancing = arming.state == RobotState::Balancing;
  if (strcmp(name, "stop") == 0) {
    requestStop(true);
  } else if (strcmp(name, "go") == 0) {
    requestStop(false);
  } else if (strcmp(name, "calib") == 0) {
    if (balancing) {
      Serial.println("! Stop the robot first ('stop'), then 'calib'.");
    } else {
      startGyroCalibration();
    }
  } else if (strcmp(name, "zero") == 0) {
    if (balancing) {
      Serial.println("! Stop the robot first ('stop'), hold it at its balance point, then 'zero'.");
    } else {
      settings.baseAngle = clampf(currentAngle, -45.0f, 45.0f);
      autoTrim.reset();
      Serial.printf("-> base = %.2f (current angle). Type 'save' to keep it.\n", settings.baseAngle);
    }
  } else if (strcmp(name, "log") == 0) {
    logMode = cmd.hasValue ? static_cast<LogMode>(static_cast<int>(clampf(cmd.value, 0.0f, 2.0f)))
                           : LogMode::Text;
    if (logMode == LogMode::Plotter) Serial.println("-> Plotter output: open Tools > Serial Plotter.");
  } else if (strcmp(name, "encoders") == 0) {
    if (!ENCODERS_ENABLED) {
      Serial.println("! Encoders are off: set ENCODERS_ENABLED in config.h.");
    } else {
      int32_t left = 0, right = 0;
      encodersReadTotals(left, right);
      Serial.printf("-> Counts since last check: left %ld, right %ld (one wheel turn = %.0f).\n",
                    static_cast<long>(left), static_cast<long>(right), ENCODER_COUNTS_PER_WHEEL_REV);
      Serial.println("   Roll the robot FORWARD and check again: both must be positive.");
    }
  } else if (strcmp(name, "motortest") == 0) {
    if (balancing) {
      Serial.println("! Stop the robot first ('stop') and hold it with the wheels in the air.");
    } else {
      Serial.println("-> Motor test: left wheel, pause, right wheel.");
      motorTest.start();
    }
  } else {
    return false;
  }
  return true;
}

void requestStop(bool stop) {
  if (stop == stopRequested) return;
  stopRequested = stop;
  Serial.println(stop ? "-> Emergency stop: motors off. 'go' (or G on the phone) to continue."
                      : "-> Go: hold the robot upright to start balancing.");
}

void readSerialCommands() {
  while (Serial.available() > 0) {
    switch (serialLine.push(static_cast<char>(Serial.read()))) {
      case LineStatus::Ready: handleCommandLine(serialLine.line()); break;
      case LineStatus::Overflow: Serial.println("! Line too long, ignored."); break;
      case LineStatus::Pending: break;
    }
  }
}
