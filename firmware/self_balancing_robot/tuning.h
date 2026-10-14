// Serial console commands that read and change Settings.
#pragma once

#include <Print.h>

#include "command.h"
#include "settings.h"

// Handles parameter commands ("kp 15", "p15", ...), "autotrim", "show",
// "save", "defaults" and "help". Returns false for any other command so the
// caller can handle it.
bool handleSettingsCommand(const Command& cmd, Settings& settings, Print& out);

void printSettings(const Settings& settings, Print& out);
void printHelp(Print& out);
