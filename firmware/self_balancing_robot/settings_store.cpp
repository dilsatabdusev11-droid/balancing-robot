#include "settings_store.h"

#include <Preferences.h>

namespace {
constexpr const char* kNamespace = "balbot";
constexpr const char* kVersionKey = "ver";
constexpr const char* kDataKey = "cfg";
}  // namespace

bool loadSettings(Settings& settings) {
  Preferences prefs;
  if (!prefs.begin(kNamespace, true)) return false;  // nothing saved yet
  Settings stored;
  const bool compatible = prefs.getUInt(kVersionKey, 0) == kSettingsVersion &&
                          prefs.getBytesLength(kDataKey) == sizeof(Settings) &&
                          prefs.getBytes(kDataKey, &stored, sizeof(Settings)) == sizeof(Settings);
  prefs.end();
  if (!compatible) return false;
  sanitize(stored);
  settings = stored;
  return true;
}

bool saveSettings(const Settings& settings) {
  Preferences prefs;
  if (!prefs.begin(kNamespace, false)) return false;
  const bool ok = prefs.putBytes(kDataKey, &settings, sizeof(Settings)) == sizeof(Settings) &&
                  prefs.putUInt(kVersionKey, kSettingsVersion) > 0;
  prefs.end();
  return ok;
}

bool clearSavedSettings() {
  Preferences prefs;
  if (!prefs.begin(kNamespace, false)) return false;
  const bool ok = prefs.clear();
  prefs.end();
  return ok;
}
