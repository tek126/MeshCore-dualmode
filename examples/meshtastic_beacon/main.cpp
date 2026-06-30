// Meshtastic beacon demo.
//
// A stripped-down repeater-style firmware that, every BEACON_INTERVAL, hops to
// the Meshtastic LongFast modem preset and transmits a "MeshCore repeater in
// range" text packet onto the default public channel, then hops back. Between
// beacons it just idles on the MeshCore PHY — fold the transmit call into a
// real repeater loop (e.g. simple_repeater) to make it useful.
//
// It reuses the variant's board/radio globals from target.h, so build it for a
// LoRa board the same way the other examples are built (add a PlatformIO env
// pointing src_dir here; see README).

#include <Arduino.h>
#include <Mesh.h>
#include "target.h"               // board, radio_driver, radio_init(), rtc_clock
#include "MeshtasticBeacon.h"

// The concrete RadioLib radio is defined (non-static) in the variant target.cpp
// but only radio_driver is declared in target.h; re-declare it here.
extern RADIO_CLASS radio;

// ---- MeshCore "home" PHY to restore after each beacon ----
// These mirror the compile-time LORA_* defaults the firmware booted with.
#ifndef LORA_CR
  #define LORA_CR 5
#endif
#ifndef LORA_TX_POWER
  #define LORA_TX_POWER 22
#endif
// RadioLib's 1-byte private LoRa sync word (0x12) — what MeshCore uses for all
// its LoRa chips. Override if your board/build differs.
#ifndef MESHCORE_SYNC_WORD
  #define MESHCORE_SYNC_WORD 0x12
#endif

// ---- beacon config ----
static const meshtastic::ModemPreset& MT_PRESET = meshtastic::LONGFAST_US;
static const char* BEACON_TEXT = "MeshCore repeater in range";
static const uint32_t BEACON_INTERVAL_MS = 5UL * 60UL * 1000UL;  // every 5 min
static const int8_t MT_TX_POWER = LORA_TX_POWER;

static uint32_t g_node_num;     // our Meshtastic node number (stable per device)
static uint32_t g_packet_id;    // incremented per beacon (must be nonzero)
static uint8_t  g_chan_hash;    // default-channel hash (0x08)
static uint32_t g_next_beacon;

static void halt() { while (1) ; }

static void sendBeacon() {
  uint8_t pkt[256];
  uint32_t id = ++g_packet_id;
  int len = meshtastic::buildTextPacket(
      pkt, sizeof(pkt),
      g_node_num, id, BEACON_TEXT,
      meshtastic::DEFAULT_KEY, sizeof(meshtastic::DEFAULT_KEY),
      g_chan_hash);
  if (len <= 0) { MESH_DEBUG_PRINTLN("beacon: build failed"); return; }

  bool ok = meshtastic::transmitOnce(
      radio_driver, radio, MT_PRESET, pkt, len,
      LORA_FREQ, LORA_BW, LORA_SF, LORA_CR, MESHCORE_SYNC_WORD,
      MT_TX_POWER, LORA_TX_POWER);

  Serial.printf("beacon: %s id=%lu len=%d\n", ok ? "sent" : "FAILED",
                (unsigned long)id, len);
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  board.begin();
  if (!radio_init()) {
    MESH_DEBUG_PRINTLN("Radio init failed!");
    halt();
  }

  // Derive a Meshtastic node number from the radio RNG seed. A real build
  // should derive it from the device identity / EUI so it survives reboots;
  // for the demo a per-boot value is fine.
  g_node_num = ((uint32_t)radio_driver.getRngSeed() << 8) ^ radio_driver.getRngSeed();
  if (g_node_num == 0 || g_node_num == 0xFFFFFFFF) g_node_num = 0x0DECAFED;
  g_packet_id = radio_driver.getRngSeed();   // random starting id

  g_chan_hash = meshtastic::channelHash(
      meshtastic::DEFAULT_CHANNEL_NAME,
      meshtastic::DEFAULT_KEY, sizeof(meshtastic::DEFAULT_KEY));

  Serial.printf("Meshtastic beacon ready: node=!%08lx chanHash=0x%02x (expect 0x08)\n",
                (unsigned long)g_node_num, g_chan_hash);

  board.onBootComplete();
  g_next_beacon = millis();   // beacon once shortly after boot
}

void loop() {
  radio_driver.loop();        // keep the radio serviced on the MeshCore PHY

  if ((int32_t)(millis() - g_next_beacon) >= 0) {
    sendBeacon();
    g_next_beacon = millis() + BEACON_INTERVAL_MS;
  }
}
