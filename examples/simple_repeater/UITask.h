#pragma once

#include <helpers/ui/DisplayDriver.h>
#include <helpers/CommonCLI.h>

#ifdef WITH_MT_BEACON
class MtBeaconControl;   // fwd decl; full include only in UITask.cpp
#endif

class UITask {
  DisplayDriver* _display;
  unsigned long _next_read, _next_refresh, _auto_off;
  int _prevBtnState;
  NodePrefs* _node_prefs;
  char _version_info[32];
#ifdef WITH_MT_BEACON
  MtBeaconControl* _beacon = nullptr;
#endif

  void renderCurrScreen();
public:
  UITask(DisplayDriver& display) : _display(&display) { _next_read = _next_refresh = 0; }
  void begin(NodePrefs* node_prefs, const char* build_date, const char* firmware_version);
#ifdef WITH_MT_BEACON
  void setBeacon(MtBeaconControl* b) { _beacon = b; }
#endif

  void loop();
};