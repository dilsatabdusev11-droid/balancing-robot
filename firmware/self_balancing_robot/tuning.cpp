#include "tuning.h"

#include <string.h>

#include "settings_store.h"

namespace {

bool is(const Command& cmd, const char* name) { return strcmp(cmd.name, name) == 0; }

void printParam(const Settings& settings, const ParamInfo& param, Print& out) {
  out.printf("  %-9s (%s) = %8.3f   [%g .. %g] %s\n", param.name, param.alias,
             settings.*(param.field), param.minValue, param.maxValue, param.description);
}

}  // namespace

void printSettings(const Settings& settings, Print& out) {
  size_t count = 0;
  const ParamInfo* params = paramTable(count);
  out.println("Settings:");
  for (size_t i = 0; i < count; ++i) printParam(settings, params[i], out);
  out.printf("  autotrim  (a) = %s\n", settings.autoTrim ? "on" : "off");
}

void printHelp(Print& out) {
  out.println("Commands (type and press Enter):");
  out.println("  <name> <value>   set a parameter, e.g. 'kp 15', 'kp=15' or 'p15'");
  out.println("  autotrim 1|0     auto-trim on/off (also 'a1' / 'a0')");
  out.println("  show  (or ?)     list all parameters and their ranges");
  out.println("  save             keep the current values after a reboot");
  out.println("  defaults         back to factory values (also forgets saved ones)");
  out.println("  help             this list");
}

bool handleSettingsCommand(const Command& cmd, Settings& settings, Print& out) {
  if (is(cmd, "help")) {
    printHelp(out);
    return true;
  }
  if (is(cmd, "show") || is(cmd, "?")) {
    printSettings(settings, out);
    return true;
  }
  if (is(cmd, "save")) {
    out.println(saveSettings(settings) ? "-> Settings saved." : "! Could not save settings.");
    return true;
  }
  if (is(cmd, "defaults")) {
    settings = Settings();
    out.println(clearSavedSettings() ? "-> Factory settings restored."
                                     : "! Defaults applied, but saved settings could not be erased.");
    return true;
  }
  if (is(cmd, "a") || is(cmd, "autotrim")) {
    if (!cmd.hasValue) {
      out.println("! Usage: autotrim 1 (on) or autotrim 0 (off)");
      return true;
    }
    settings.autoTrim = cmd.value != 0.0f;
    out.printf("-> Auto-trim: %s\n", settings.autoTrim ? "ON" : "OFF");
    return true;
  }

  const ParamInfo* param = findParam(cmd.name);
  if (param == nullptr) return false;
  if (!cmd.hasValue) {
    printParam(settings, *param, out);
    return true;
  }
  if (setParam(settings, *param, cmd.value) == SetResult::Clamped) {
    out.printf("! %s limited to its range [%g .. %g]\n", param->name, param->minValue, param->maxValue);
  }
  out.printf("-> %s = %.3f\n", param->name, settings.*(param->field));
  return true;
}
