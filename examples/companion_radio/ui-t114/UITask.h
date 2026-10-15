#pragma once

// Companion UI for the Heltec T114's 240x135 colour screen (ST7789ColorDisplay).
// Same public interface as ui-new's UITask, so main.cpp is unchanged; the
// pages are laid out for the native resolution, with a status bar on every
// page and an on-device message history.

#include <MeshCore.h>
#include <helpers/ui/DisplayDriver.h>
#include <helpers/ui/ST7789ColorDisplay.h>
#include <helpers/ui/UIScreen.h>
#include <helpers/SensorManager.h>
#include <helpers/MultiSerialInterface.h>
#include <Arduino.h>

#ifdef PIN_BUZZER
  #include <helpers/ui/buzzer.h>
#endif
#ifdef PIN_VIBRATION
  #include <helpers/ui/GenericVibration.h>
#endif

#include "../AbstractUITask.h"
#include "../NodePrefs.h"

#ifndef UI_DISCOVER_SCREEN
  #define UI_DISCOVER_SCREEN  1
#endif

#define DISCOVERED_NODES_TABLE_SIZE  6

struct DiscoveredNode {
  float snr_in;
  float snr_out;
  uint8_t pubkey_prefix[9];
  uint8_t type;
  char name[32];
};

// On-device message history, independent of the phone's offline queue (the
// app syncing does not empty it). RAM only: cleared on reboot.
#ifndef UI_MSG_HISTORY
  #define UI_MSG_HISTORY  16
#endif

struct UIMessage {
  uint32_t timestamp;
  uint8_t  hops;        // 0xFF = direct
  bool     channel;
  bool     unread;
  char     from[32];
  char     text[164];
};

class UIMessageStore {
  UIMessage _msgs[UI_MSG_HISTORY];
  int _head = -1;       // index of the newest message
  int _count = 0;
public:
  void add(uint32_t ts, uint8_t hops, bool channel, const char* from, const char* text);
  int count() const { return _count; }
  int unread() const;
  // i = 0 is the newest message
  UIMessage* get(int i);
  void markAllRead();
};

// Car nodes heard recently (from the FEAT2 advert marker). RAM only.
#define UI_MAX_CARS  4

struct UICar {
  uint8_t  key[8];      // pub_key prefix
  char     name[32];
  bool     has_loc;
  int32_t  lat, lon;    // last parked position, 6 decimal places
  bool     parked;      // still parked there (as of its last advert)
  uint8_t  hops;
  uint32_t heard_at;    // our clock
};

class UITask : public AbstractUITask {
  DisplayDriver* _display;
  SensorManager* _sensors;
#ifdef PIN_BUZZER
  genericBuzzer buzzer;
#endif
#ifdef PIN_VIBRATION
  GenericVibration vibration;
#endif
  unsigned long _next_refresh, _auto_off;
  NodePrefs* _node_prefs;
  char _alert[80];
  unsigned long _alert_expiry;
  int _msgcount;
  unsigned long ui_started_at, next_batt_chck;

  UIScreen* splash;
  UIScreen* home;
  UIScreen* reader;
  UIScreen* curr;

  void userLedHandler();

  char checkDisplayOn(char c);
  char handleLongPress(char c);
  char handleDoubleClick(char c);
  char handleTripleClick(char c);

  void setCurrScreen(UIScreen* c);
  void onNewMessage();

public:
  UIMessageStore messages;
  UICar cars[UI_MAX_CARS];
  int num_cars = 0;

  UITask(mesh::MainBoard* board, MultiSerialInterface* serial) : AbstractUITask(board, serial), _display(NULL), _sensors(NULL) {
    next_batt_chck = _next_refresh = 0;
    ui_started_at = 0;
    _msgcount = 0;
    curr = NULL;
  }
  void begin(DisplayDriver* display, SensorManager* sensors, NodePrefs* node_prefs);

  void gotoHomeScreen();
  void openReader(int index);
  void showAlert(const char* text, int duration_millis);
  int  getMsgCount() const { return _msgcount; }
  bool hasDisplay() const { return _display != NULL; }
  bool isButtonPressed() const;
  NodePrefs* getNodePrefs() { return _node_prefs; }

  bool isBuzzerQuiet() {
#ifdef PIN_BUZZER
    return buzzer.isQuiet();
#else
    return true;
#endif
  }

  void toggleBuzzer();
  bool getGPSState();
  void toggleGPS();

  // MyMesh::Listener
  void onMessageRecv(mesh::Packet *pkt, const ContactInfo &from, uint8_t txt_type, uint32_t sender_timestamp, const char* text) override;
  void onChannelMessageRecv(mesh::Packet *pkt, ChannelDetails& channel_details, const char* text) override;
  void onQueueSizeChanged(int offline_queue_size) override;
  void onDiscoveredContact(ContactInfo &contact, bool is_new, uint8_t path_len, const uint8_t* path) override;
  void onControlDataRecv(const mesh::Packet* pkt) override;
  void onCarNodeAdvert(const uint8_t* pub_key, const char* name, bool has_loc,
                       int32_t lat_e6, int32_t lon_e6, bool parked, uint8_t hops) override;

  // where this companion is: GPS fix, else the location set in the app
  bool getOwnPosition(double& lat, double& lon);

  // from AbstractUITask
  void notify(UIEventType t = UIEventType::none) override;
  void loop() override;

  void shutdown(bool restart = false);
};
