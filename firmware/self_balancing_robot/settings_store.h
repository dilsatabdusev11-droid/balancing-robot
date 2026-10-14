// Keeps Settings in the ESP32 flash (NVS) so tuned values survive a reboot.
#pragma once

#include "settings.h"

// Loads saved settings. Returns false (and leaves `settings` untouched) when
// nothing compatible is stored.
bool loadSettings(Settings& settings);
bool saveSettings(const Settings& settings);
// Forgets the saved settings; the next boot starts from the defaults.
bool clearSavedSettings();
