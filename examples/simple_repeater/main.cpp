#include "dualmode_rename.h"   // must precede MyMesh.h (renames classes under -D DUALMODE)
#include <Arduino.h>   // needed for PlatformIO
#include <Mesh.h>

#include "MyMesh.h"

#if defined(DISPLAY_CLASS) && !defined(DUALMODE)
  #include "UITask.h"
  static UITask ui_task(board, display);
#endif

#ifdef ETHERNET_ENABLED
  #define ETHERNET_CLI_BANNER "MeshCore Repeater CLI"
  #include <helpers/nrf52/EthernetCLI.h>
#endif

#ifdef NRF52_PLATFORM
  #include <helpers/nrf52/SafeInternalFS.h>
#endif

#if defined(ENABLE_BITCHAT) && (defined(ESP32) || defined(NRF52_PLATFORM))
  #include <new>
  #include <helpers/bitchat/BitchatBridge.h>
  #ifdef ESP32
    #include <esp_task_wdt.h>
  #endif
  static BitchatBridge* bitchat_bridge = nullptr;
#endif

static StdRNG fast_rng;
static SimpleMeshTables tables;

static MyMesh the_mesh(board, radio_driver, *new ArduinoMillis(), fast_rng, rtc_clock, tables);

static void halt() {
  while (1) ;
}

static char command[160];
#ifdef ETHERNET_ENABLED
static char ethernet_command[160];
#endif

// For power saving
unsigned long POWERSAVING_FIRSTSLEEP_SECS = 120; // The first sleep (if enabled) from boot

#if defined(PIN_USER_BTN) && defined(_SEEED_SENSECAP_SOLAR_H_)
static unsigned long userBtnDownAt = 0;
#define USER_BTN_HOLD_OFF_MILLIS 1500
#endif

#ifdef DUALMODE
void rpt_setup() {
#else
void setup() {
#endif
  Serial.begin(115200);
  delay(1000);

  board.begin();

#ifdef NRF52_PLATFORM
  // Hard-hang insurance: if loop() ever stalls (wedged radio wait, runaway
  // spin), the hardware watchdog reboots the node instead of leaving it dead
  // until someone drives out and power-cycles it. Fed at the top of loop().
  board.startWatchdog(90);
  // Breadcrumb for post-mortems ("Watchdog" here = the WDT above fired last
  // boot). Also remotely queryable via 'get pwrmgt.bootreason'.
  Serial.print("Reset reason: ");
  Serial.println(board.getResetReasonString(board.getResetReason()));
#endif

#ifdef HAS_EXTERNAL_WATCHDOG
  external_watchdog.begin();
#endif

#if defined(MESH_DEBUG) && defined(NRF52_PLATFORM)
  // give some extra time for serial to settle so
  // boot debug messages can be seen on terminal
  delay(5000);
#endif

#if defined(DISPLAY_CLASS) && !defined(DUALMODE)
  if (display.begin()) {
    display.startFrame();
    display.setCursor(0, 0);
    display.print("Please wait...");
    display.endFrame();
  }
#endif

  if (!radio_init()) {
    MESH_DEBUG_PRINTLN("Radio init failed!");
    halt();
  }

  fast_rng.begin(radio_driver.getRngSeed());

  FILESYSTEM* fs;
#if defined(NRF52_PLATFORM) || defined(STM32_PLATFORM)
 #if defined(NRF52_PLATFORM)
  if (!safeInternalFSBegin()) haltFSMountFailed();  // never auto-format existing data
 #else
  InternalFS.begin();
 #endif
  fs = &InternalFS;
  IdentityStore store(InternalFS, "");
#elif defined(ESP32)
  SPIFFS.begin(true);
  fs = &SPIFFS;
  IdentityStore store(SPIFFS, "/identity");
#elif defined(RP2040_PLATFORM)
  LittleFS.begin();
  fs = &LittleFS;
  IdentityStore store(LittleFS, "/identity");
  store.begin();
#else
  #error "need to define filesystem"
#endif
  if (!store.load("_main", the_mesh.self_id)) {
    MESH_DEBUG_PRINTLN("Generating new keypair");
    the_mesh.self_id = radio_new_identity();   // create new random identity
    int count = 0;
    while (count < 10 && (the_mesh.self_id.pub_key[0] == 0x00 || the_mesh.self_id.pub_key[0] == 0xFF)) {  // reserved id hashes
      the_mesh.self_id = radio_new_identity(); count++;
    }
    store.save("_main", the_mesh.self_id);
  }

  Serial.print("Repeater ID: ");
  mesh::Utils::printHex(Serial, the_mesh.self_id.pub_key, PUB_KEY_SIZE); Serial.println();

  command[0] = 0;
#ifdef ETHERNET_ENABLED
  ethernet_command[0] = 0;
#endif

  sensors.begin();

  the_mesh.begin(fs);

#if defined(DISPLAY_CLASS) && !defined(DUALMODE)
  ui_task.begin(the_mesh.getNodePrefs(), FIRMWARE_BUILD_DATE, FIRMWARE_VERSION);
#ifdef WITH_MT_BEACON
  ui_task.setBeacon(the_mesh.getBeacon());
#endif
#ifdef WITH_CAR_NODE
  ui_task.setCarNode(the_mesh.getCarNode());
#endif
#endif

#ifdef WITH_CAR_NODE
  user_btn.begin();   // hold-to-hibernate uses the shared MomentaryButton
#endif

#ifdef ETHERNET_ENABLED
  ethernet_start_task();
#endif

#if defined(ENABLE_BITCHAT) && (defined(ESP32) || defined(NRF52_PLATFORM))
  // BitChat bridge: the repeater (mode) has no other BLE user, so the bridge
  // owns the BLE stack (standalone mode — the configuration the upstream
  // author found reliable). In a DUALMODE build the companion half's phone BLE
  // never conflicts: only one mode runs per boot. Must start after
  // the_mesh.begin() so the channel registry and node name are loaded.
  // nothrow: if the ~45KB bridge object doesn't fit, run as a plain repeater
  bitchat_bridge = new (std::nothrow) BitchatBridge(the_mesh, the_mesh.self_id, the_mesh.getNodeName());
  if (bitchat_bridge == nullptr) {
    Serial.println("ERROR: BitchatBridge allocation failed - bridge disabled");
  }
  if (bitchat_bridge != nullptr) {
    bitchat_bridge->begin();
    if (bitchat_bridge->beginStandalone(the_mesh.getNodeName())) {
      Serial.println("Bitchat BLE service started (standalone mode)");
    } else {
      Serial.println("ERROR: Failed to start Bitchat BLE service!");
    }
    the_mesh.initBitchat(bitchat_bridge);
  }

  #ifdef ESP32
  // Hang insurance, ESP32 edition: the freeze this port is chasing presented
  // as a whole node dead until power cycle. If the loop task ever wedges (BLE
  // stack deadlock, blocked serial write), the task watchdog reboots the node
  // instead. Fed at the top of loop(). 60s is far beyond any legitimate pass.
  // (nRF52 already runs the 90s hardware watchdog started above.)
  esp_task_wdt_init(60, true);
  esp_task_wdt_add(NULL);
  #endif
#endif

  // send out initial zero hop Advertisement to the mesh
#if ENABLE_ADVERT_ON_BOOT == 1
  the_mesh.sendSelfAdvertisement(16000, false);
#endif

  board.onBootComplete();
}

#ifdef DUALMODE
void rpt_loop() {
#else
void loop() {
#endif
#ifdef NRF52_PLATFORM
  board.feedWatchdog();   // every pass through loop() proves we're alive
#endif
#if defined(ENABLE_BITCHAT) && (defined(ESP32) || defined(NRF52_PLATFORM))
  #ifdef ESP32
  esp_task_wdt_reset();   // every pass through loop() proves we're alive
  #endif
  if (bitchat_bridge != nullptr) {
    bitchat_bridge->loop();
  }
#endif

  // Handle Serial CLI
  int len = strlen(command);
  while (Serial.available() && len < sizeof(command)-1) {
    char c = Serial.read();
    if (c != '\n') {
      command[len++] = c;
      command[len] = 0;
      Serial.print(c);
    }
    if (c == '\r') break;
  }
  if (len == sizeof(command)-1) {  // command buffer full
    command[sizeof(command)-1] = '\r';
  }

  if (len > 0 && command[len - 1] == '\r') {  // received complete line
    Serial.print('\n');
    command[len - 1] = 0;  // replace newline with C string null terminator
    char reply[160];
    reply[0] = 0;
#ifdef ETHERNET_ENABLED
    if (!ethernet_handle_command(command, reply)) {
      the_mesh.handleCommand(0, command, reply);
    }
#else
    the_mesh.handleCommand(0, command, reply);  // NOTE: there is no sender_timestamp via serial!
#endif
    if (reply[0]) {
      Serial.print("  -> "); Serial.println(reply);
    }

    command[0] = 0;  // reset command buffer
  }

#ifdef ETHERNET_ENABLED
  ethernet_loop_maintain();
  if (ethernet_read_line(ethernet_command, sizeof(ethernet_command))) {
    char reply[160];
    reply[0] = 0;
    if (!ethernet_handle_command(ethernet_command, reply)) {
      the_mesh.handleCommand(0, ethernet_command, reply);
    }
    ethernet_send_reply(reply);
    ethernet_command[0] = 0;
  }
#endif

#if defined(PIN_USER_BTN) && defined(_SEEED_SENSECAP_SOLAR_H_) && !defined(DISPLAY_CLASS)
  // Hold the user button to power off the SenseCAP Solar repeater.
  int btnState = digitalRead(PIN_USER_BTN);
  if (btnState == LOW) {
    if (userBtnDownAt == 0) {
      userBtnDownAt = millis();
    } else if ((unsigned long)(millis() - userBtnDownAt) >= USER_BTN_HOLD_OFF_MILLIS) {
      Serial.println("Powering off...");
      board.powerOff();  // does not return
    }
  } else {
    userBtnDownAt = 0;
  }
#endif

#ifdef WITH_CAR_NODE
  // Hold the user button for CAR_NODE_HOLD_OFF_MILLIS (~3 s) to hibernate the
  // car node via board.hibernateButtonWake(): the board enters its lowest-power
  // state, waking only on the user button, and does not return.
  //   V4   -> ESP32 deep sleep (button-only wake, stays asleep through LoRa RX).
  //   T114 -> nRF52 SYSTEMOFF with a button SENSE wake (SD-aware); wake reboots.
  // On both, tap the button to wake (on V4 don't hold GPIO0 through reset).
  #ifndef CAR_NODE_HOLD_OFF_MILLIS
    #define CAR_NODE_HOLD_OFF_MILLIS 3000
  #endif
  {
    static unsigned long carnode_btn_down_at = 0;
    if (user_btn.isPressed()) {
      if (carnode_btn_down_at == 0) {
        carnode_btn_down_at = millis();
      } else if ((unsigned long)(millis() - carnode_btn_down_at) >= CAR_NODE_HOLD_OFF_MILLIS) {
        Serial.println("CarNode: hibernating (button held)...");
      #if defined(DISPLAY_CLASS) && !defined(DUALMODE)
        if (display.isOn()) {
          display.startFrame();
          display.setCursor(0, 0);
          display.print("Hibernating...");
          display.endFrame();
        }
      #endif
        board.hibernateButtonWake(PIN_USER_BTN);   // wake on button only; does not return
      }
    } else {
      carnode_btn_down_at = 0;
    }
  }
#endif

  the_mesh.loop();
  sensors.loop();
#if defined(DISPLAY_CLASS) && !defined(DUALMODE)
  ui_task.loop();
#endif
  rtc_clock.tick();

#ifdef HAS_EXTERNAL_WATCHDOG
  external_watchdog.loop();
#endif
  bool may_sleep = the_mesh.getNodePrefs()->powersaving_enabled && !the_mesh.hasPendingWork();
#if defined(ENABLE_BITCHAT) && defined(ESP32)
  // ESP32 light sleep stops the BLE radio — phones would lose the BitChat
  // connection every time the node dozed. Bridge running == no sleep.
  if (bitchat_bridge != nullptr) may_sleep = false;
#endif
#ifdef DUALMODE
  // Power-saving sleep stays enabled in dual-mode, but is suppressed during a
  // user-button press burst so the launcher can poll the 5x mode-switch at full
  // speed (the burst's final event dispatches ~500ms after the last release).
  // A button edge wakes us from sleep via the GPIO interrupt armed in the
  // launcher, so idle battery life is preserved between presses.
  extern bool dualmode_button_active();
  if (dualmode_button_active()) may_sleep = false;
#endif
  if (may_sleep) {
#if defined(NRF52_PLATFORM)
    board.sleep(0); // nrf ignores seconds param, sleeps whenever possible
#else
    if (the_mesh.millisHasNowPassed(POWERSAVING_FIRSTSLEEP_SECS * 1000)) { // To check if it is time to sleep
      board.sleep(30); // Sleep. Wake up after a while or when receiving a LoRa packet
    }
#endif
  }
}
