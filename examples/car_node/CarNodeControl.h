#pragma once

// CarNodeControl — runtime-configurable, park-driven location beacon for a
// MOBILE (in-vehicle) MeshCore repeater.
//
// This is a personal fork of meshtastic_beacon's MtBeaconControl, kept separate
// from that project. It reuses the on-air-verified interop math from the
// meshtastic_beacon folder (MeshtasticProto.h / MeshtasticBeacon.h).
//
// Location model (deliberately NOT live-while-driving):
//   * The vehicle is considered PARKED once its GPS fix stays within
//     `stop_radius_m` for `park_secs` (default 5 min).
//   * On that single park event, ONE unified update fires:
//       - a Meshtastic beacon burst (NodeInfo + Position [+ text]), AND
//       - a request for the repeater to update its MeshCore advert location and
//         flood re-advert  -- both with the same fix, at the same time.
//   * Between park events, a LIGHT periodic presence (base-mtbeacon style) keeps
//     the node from aging out of Meshtastic node lists: every `interval_mins` it
//     sends one packet — Position while parked (refreshes the pin at the parked
//     spot), NodeInfo only while driving or without a fix, so the map pin never
//     wanders off to some random point mid-drive. 0 = park-only (old behaviour).
//   * Location, then, still updates ONLY at park time. Re-parking in the same
//     spot does not re-broadcast (dedup by `stop_radius_m`); `carnode send`
//     forces an update.
//   * Optional QUIET ZONES: `carnode zone add <name>` stores the current spot
//     (up to 4 of them, e.g. home / work). Parking inside one turns repeating
//     off immediately (same switch as the park sleep) and skips the park
//     broadcast + periodic presence, so that place never goes on the air.
//     Driving away restores everything. `carnode home` is shorthand for the
//     zone named "home", which is what this used to be before v0.2.6.
//
// The repeater drives the MeshCore side: after tick(), it calls takeReadvert()
// and, if a location update is pending, writes it into NodePrefs and re-adverts.
//
// Header-only; drop into simple_repeater behind -D WITH_CAR_NODE. Include it
// AFTER the platform filesystem header (uses FILESYSTEM / File), which MyMesh.h
// already pulls in. Requires -I examples/meshtastic_beacon for the shared
// interop headers below.

#include <Arduino.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "MeshtasticBeacon.h"   // shared interop math (meshtastic_beacon folder)

// RadioLib's 1-byte private LoRa sync word (0x12) — what MeshCore uses on its
// LoRa chips; restored after each beacon. Override per board if needed.
#ifndef MESHCORE_SYNC_WORD
  #define MESHCORE_SYNC_WORD 0x12
#endif

#ifndef CAR_NODE_FILE
  #define CAR_NODE_FILE "/carnode"
#endif

// Meshtastic HardwareModel reported in NodeInfo. Set per board via -D MT_HW_MODEL
// (e.g. 110=Heltec V4). Defaults to 255 (PRIVATE_HW) for boards with no
// Meshtastic equivalent.
#ifndef MT_HW_MODEL
  #define MT_HW_MODEL 255
#endif

// The fix must sit outside `stop_radius_m` for this long, continuously, before
// it counts as driving. Without this, a single multipath outlier re-anchors and
// resets the parked/sleep clock — one bad fix per `park` window keeps a parked
// car in "driving" forever.
#ifndef CAR_NODE_MOVE_CONFIRM_MS
  #define CAR_NODE_MOVE_CONFIRM_MS 15000UL
#endif

// Median fix filter: the park logic (and the broadcast position) uses the
// component-wise median of the last few fixes instead of the instantaneous
// one, so a single teleporting fix — hundreds of metres of multipath from a
// poorly-sited antenna — is rejected outright no matter how far it jumps.
// Samples are taken every SAMPLE_MS; ones older than STALE_MS are dropped.
#ifndef CAR_NODE_FIX_SAMPLE_MS
  #define CAR_NODE_FIX_SAMPLE_MS 3000UL
#endif
#ifndef CAR_NODE_FIX_STALE_MS
  #define CAR_NODE_FIX_STALE_MS 60000UL
#endif

// The MeshCore park re-advert is handed to the repeater, which can fail to
// queue it (packet pool or send queue exhausted while forwarding a burst).
// That is silent and unrecoverable — the park is already marked reported, so
// nothing retries and the mesh never learns where we stopped. Re-arm the
// handoff instead, with a linear backoff, for this many attempts.
#ifndef CAR_NODE_READVERT_MAX_TRIES
  #define CAR_NODE_READVERT_MAX_TRIES 5
#endif
#ifndef CAR_NODE_READVERT_RETRY_MS
  #define CAR_NODE_READVERT_RETRY_MS 5000UL
#endif

// Consecutive transmits the chip reports as never-sent before we re-initialise
// the radio (see meshtastic::radioRecover). One dead transmit is bad luck; a run
// of them is a wedged modem that will otherwise stay wedged until a power cycle.
#ifndef MT_BEACON_FAIL_STREAK
  #define MT_BEACON_FAIL_STREAK 3
#endif

class CarNodeControl {
public:
  // A named QUIET ZONE: park inside one and the node goes silent — repeating
  // off, no Meshtastic burst, no MeshCore re-advert, no periodic presence — so
  // the place never goes on the air. This generalises the single "home" spot
  // that v8 and earlier had; `carnode home` is now just the zone named "home".
  struct Zone {
    double   lat, lon;
    uint16_t radius_m;
    uint8_t  used;
    char     name[9];      // 8 chars + NUL
  };
  static const uint8_t MAX_ZONES = 4;

  struct Config {
    uint32_t magic;
    uint8_t  enabled;
    uint8_t  region_idx;     // index into meshtastic::REGIONS
    uint8_t  preset_idx;     // index into meshtastic::PRESETS
    uint8_t  send_nodeinfo;  // emit NodeInfo (named node) on the Meshtastic burst
    uint8_t  send_position;  // emit Position (map pin) on the Meshtastic burst
    uint8_t  text_mult;      // chat text N times per flood-advert period (0=never)
    int8_t   tx_power;
    uint16_t park_secs;      // stationary time before a unified location update fires
    uint16_t stop_radius_m;  // movement within this radius counts as "stopped"
    uint16_t advert_delay_s; // gap between the Meshtastic burst and the MeshCore advert
    uint16_t sleep_hours;    // parked this long -> stop repeating until driving (0=never)
    uint16_t interval_mins;  // periodic presence cadence between parks (0=park-only)
    float    freq_override;  // 0 = auto (derived from region + preset)
    char     text[64];
    // derived from region/preset (recomputed on every change; persisted too)
    float    freq;
    float    bw;
    uint8_t  sf;
    uint8_t  cr;
    uint8_t  sync_word;
    uint16_t preamble;
    // v6: the original single "home" spot. RETIRED in v9 — load() moves it into
    // zones[0] (named "home") and clears have_home. These fields stay here, and
    // stay unused, only so that an older file's byte layout still lines up with
    // this struct's prefix; nothing reads them after the migration.
    uint8_t  have_home;
    uint16_t home_radius_m;
    double   home_lat, home_lon;
    // v7: Meshtastic hop limit (mtbeacon v0.2.0 parity). Appended, so the
    // v5/v6 -> v7 load migrations keep everything above.
    uint8_t  hop_limit;      // hop limit for presence + text (0-3, default 0)
    // v8: Meshtastic short_name / map marker label (mtbeacon v0.2.5 parity).
    // "" = auto ("MC" + 2 hex of the node id). Appended, as always.
    char     short_name[5];
    // v9: Telemetry/DeviceMetrics (battery + uptime), and the named quiet zones
    // that replace the single home spot. Appended, as always.
    uint8_t  send_telemetry;
    Zone     zones[MAX_ZONES];
  };

  // Live per-tick context the repeater supplies. lat/lon are the CURRENT GPS fix
  // and gps_valid says whether they're usable.
  struct Context {
    const char* node_name;
    double   lat, lon;
    bool     gps_valid;
    uint32_t epoch;             // 0 if unknown (Position time is then omitted)
    uint16_t flood_advert_hours; // repeater's flood-advert interval (0 = off)
    uint16_t batt_millivolts;   // 0 if the board has no battery sense
    uint32_t uptime_secs;       // 0 if unknown
    float    home_freq, home_bw;
    uint8_t  home_sf, home_cr, home_sync;
    int8_t   home_tx_power;
  };

private:
  static const uint32_t MAGIC    = 0x394E5241UL;  // 'ARN9' — v9 (+telemetry, +zones)
  static const uint32_t MAGIC_V8 = 0x384E5241UL;  // 'ARN8' — v8 (+short_name; migrated on load)
  static const uint32_t MAGIC_V7 = 0x374E5241UL;  // 'ARN7' — v7 (+hop_limit; migrated on load)
  static const uint32_t MAGIC_V6 = 0x364E5241UL;  // 'ARN6' — v6 (+home; migrated on load)
  static const uint32_t MAGIC_V5 = 0x354E5241UL;  // 'ARN5' — v5 (migrated on load)

  Config cfg;
  uint32_t node_num = 0;
  uint32_t packet_id = 1;
  uint8_t  chan_hash = 0x08;
  unsigned long hold_until = 0;        // earliest next TX (duty cycle / LBT backoff)
  unsigned long last_text_ms = 0;      // when the chat text last went out
  uint16_t flood_hours_seen = 0;       // last-known flood-advert interval (for status)
  bool     pending_text = false;       // include the chat text on the next burst
  bool     pending_send = false;       // a manual "carnode send" is queued
  bool     sleep_announced = false;    // last repeat-sleep state we logged to serial
  uint8_t  state_announced = 255;      // last drive_state we logged to serial
  unsigned long next_presence = 0;     // when the next periodic presence is due (0=unscheduled)
  uint8_t  presence_rot = 0;           // round-robins the presence kinds when rotating
  meshtastic::TxStats tx = {};         // transmit health ('mtbeacon stats')

  // --- median fix filter (defense against multipath outliers) ---
  static const uint8_t FIXWIN = 9;          // 9 samples @ 3 s = ~27 s span, robust to 4 outliers
  double   fw_lat[FIXWIN], fw_lon[FIXWIN];
  unsigned long fw_ms[FIXWIN];
  uint8_t  fw_head = 0, fw_count = 0;
  unsigned long fw_last_sample = 0;
  bool     fix_filtered = false;            // last tick's fix came from the median (status)

  static double medianOf(double* v, uint8_t n) {
    for (uint8_t i = 1; i < n; i++) {       // insertion sort; n <= FIXWIN
      double x = v[i]; int8_t j = i - 1;
      while (j >= 0 && v[j] > x) { v[j + 1] = v[j]; j--; }
      v[j + 1] = x;
    }
    return v[n / 2];
  }

  // Sample the raw fix into the ring buffer and return the component-wise
  // median of the recent samples. Until 3 fresh samples exist (first ~10 s
  // of fix, or after a long GPS outage) the raw fix passes through.
  bool filterFix(unsigned long now, double raw_lat, double raw_lon,
                 double& out_lat, double& out_lon) {
    if (fw_count == 0 || (unsigned long)(now - fw_last_sample) >= CAR_NODE_FIX_SAMPLE_MS) {
      fw_lat[fw_head] = raw_lat; fw_lon[fw_head] = raw_lon; fw_ms[fw_head] = now;
      fw_head = (fw_head + 1) % FIXWIN;
      if (fw_count < FIXWIN) fw_count++;
      fw_last_sample = now;
    }
    double la[FIXWIN], lo[FIXWIN]; uint8_t n = 0;
    for (uint8_t i = 0; i < fw_count; i++) {
      if ((unsigned long)(now - fw_ms[i]) <= CAR_NODE_FIX_STALE_MS) {
        la[n] = fw_lat[i]; lo[n] = fw_lon[i]; n++;
      }
    }
    if (n < 3) { out_lat = raw_lat; out_lon = raw_lon; return false; }
    out_lat = medianOf(la, n); out_lon = medianOf(lo, n);
    return true;
  }

  // --- park detection state ---
  double   anchor_lat = 0, anchor_lon = 0;  // reference point we measure movement from
  unsigned long stationary_since = 0;       // when we last started sitting near the anchor
  unsigned long outside_since = 0;          // fix beyond radius since (0 = currently inside)
  int      last_dist_m = -1;                // last fix's distance from the anchor (status; -1 = n/a)
  bool     have_anchor = false;
  bool     park_reported = false;           // already reported the current parked spot
  uint8_t  drive_state = 0;                 // 0=no fix, 1=driving, 2=parked
  int8_t   in_zone = -1;                    // parked inside this quiet zone (-1 = none)
  double   cur_lat = 0, cur_lon = 0;        // last filtered fix (what 'carnode zone add' captures)
  bool     cur_fix_valid = false;

  // --- MeshCore re-advert handoff + dedup ---
  double   advert_lat = 0, advert_lon = 0;  // last location we pushed to both networks
  bool     have_advert = false;
  bool     readvert_pending = false;        // a MeshCore re-advert is waiting for MyMesh
  double   readvert_lat = 0, readvert_lon = 0;
  unsigned long readvert_retry_at = 0;      // backoff before handing the re-advert over again
  uint8_t  readvert_tries = 0;              // failed attempts at queueing the re-advert
  uint8_t  readvert_drops = 0;              // re-adverts abandoned after MAX_TRIES (status)

  // Equirectangular small-distance approximation (metres). Good to <0.5% over
  // the tens-to-hundreds of metres that matter here.
  static double distMeters(double lat1, double lon1, double lat2, double lon2) {
    const double DEG2RAD = 0.017453292519943295;
    const double R = 6371000.0;
    double dlat = (lat2 - lat1) * DEG2RAD;
    double mlat = (lat1 + lat2) * 0.5 * DEG2RAD;
    double dlon = (lon2 - lon1) * DEG2RAD * cos(mlat);
    return sqrt(dlat * dlat + dlon * dlon) * R;
  }

  // --- quiet zones ---

  // Index of the zone containing (lat,lon), or -1 for none. First match wins,
  // so overlapping zones resolve to the lowest slot — deterministic, and the
  // only thing that differs between them is the name we print.
  int8_t zoneAt(double lat, double lon) const {
    for (uint8_t i = 0; i < MAX_ZONES; i++) {
      if (!cfg.zones[i].used) continue;
      if (distMeters(lat, lon, cfg.zones[i].lat, cfg.zones[i].lon)
            <= (double)cfg.zones[i].radius_m) return (int8_t)i;
    }
    return -1;
  }
  int8_t zoneFind(const char* name) const {
    for (uint8_t i = 0; i < MAX_ZONES; i++)
      if (cfg.zones[i].used && strcasecmp(cfg.zones[i].name, name) == 0) return (int8_t)i;
    return -1;
  }
  int8_t zoneFree() const {
    for (uint8_t i = 0; i < MAX_ZONES; i++) if (!cfg.zones[i].used) return (int8_t)i;
    return -1;
  }
  uint8_t zoneCount() const {
    uint8_t n = 0;
    for (uint8_t i = 0; i < MAX_ZONES; i++) if (cfg.zones[i].used) n++;
    return n;
  }

  // Text is due if its period has elapsed. Period = flood-advert interval / N,
  // so N times per advert period. No flood advert (0h) or N=0 -> text disabled.
  bool textDue(unsigned long now) const {
    if (cfg.text_mult == 0 || flood_hours_seen == 0) return false;
    unsigned long period = (unsigned long)flood_hours_seen * 3600000UL / cfg.text_mult;
    return (unsigned long)(now - last_text_ms) >= period;
  }

  void setDefaults() {
    memset(&cfg, 0, sizeof(cfg));
    cfg.magic = MAGIC;
    cfg.enabled = 0;
    cfg.region_idx = 0;        // US
    cfg.preset_idx = 0;        // LongFast
    cfg.send_nodeinfo = 1;
    cfg.send_position = 1;
    cfg.send_telemetry = 1;    // battery + uptime: small, and a parked car's battery matters
    cfg.text_mult = 1;         // text once per flood advert (rare, by design)
    cfg.hop_limit = 0;         // 0 hops: heard by direct neighbors, never rebroadcast
    cfg.freq_override = 0.0f;  // auto
    cfg.tx_power = 22;         // vehicle-powered: favour visibility (region-capped)
    cfg.park_secs = 300;       // stopped for 5 min -> push a location update
    cfg.stop_radius_m = 30;    // GPS jitter / small repositioning still counts as parked
    cfg.advert_delay_s = 10;   // MeshCore advert fires 10 s after the Meshtastic burst
    cfg.sleep_hours = 20;      // parked ~a day -> nobody's around; stop repeating
    cfg.interval_mins = 30;    // presence cadence between parks (like base mtbeacon)
    cfg.have_home = 0;         // retired in v9 — quiet zones live in cfg.zones now
    cfg.home_radius_m = 100;
    cfg.short_name[0] = 0;     // auto: "MC" + 2 hex of the node number
    strncpy(cfg.text, "MeshCore mobile node", sizeof(cfg.text) - 1);
    recompute();
  }

  // Fill the derived modem params + channel hash from region + preset (or the
  // manual frequency override). Call after any region/preset/freq change.
  void recompute() {
    if (cfg.region_idx >= meshtastic::NUM_REGIONS) cfg.region_idx = 0;
    if (cfg.preset_idx >= meshtastic::NUM_PRESETS) cfg.preset_idx = 0;
    const meshtastic::Preset& p = meshtastic::PRESETS[cfg.preset_idx];
    const meshtastic::Region& r = meshtastic::REGIONS[cfg.region_idx];
    cfg.bw = p.bw_khz; cfg.sf = p.sf; cfg.cr = p.cr;
    cfg.sync_word = 0x2B; cfg.preamble = 16;
    cfg.freq = (cfg.freq_override > 0.0f) ? cfg.freq_override
                                          : meshtastic::presetFreq(r, p);
    chan_hash = meshtastic::channelHash(p.name, meshtastic::DEFAULT_KEY,
                                        sizeof(meshtastic::DEFAULT_KEY));
  }

  void sanitize() {
    cfg.tx_power = constrain(cfg.tx_power, -9, 22);
    cfg.park_secs = constrain(cfg.park_secs, 30, 86400);
    cfg.stop_radius_m = constrain(cfg.stop_radius_m, 5, 2000);
    cfg.advert_delay_s = constrain(cfg.advert_delay_s, 0, 600);
    cfg.sleep_hours = constrain(cfg.sleep_hours, 0, 720);
    cfg.interval_mins = constrain(cfg.interval_mins, 0, 1440);
    cfg.home_radius_m = constrain(cfg.home_radius_m, 5, 2000);
    cfg.enabled = cfg.enabled ? 1 : 0;
    cfg.have_home = cfg.have_home ? 1 : 0;
    cfg.send_telemetry = cfg.send_telemetry ? 1 : 0;
    for (uint8_t i = 0; i < MAX_ZONES; i++) {
      cfg.zones[i].used = cfg.zones[i].used ? 1 : 0;
      cfg.zones[i].name[sizeof(cfg.zones[i].name) - 1] = 0;
      if (!cfg.zones[i].used || cfg.zones[i].name[0] == 0) { cfg.zones[i].used = 0; continue; }
      cfg.zones[i].radius_m = constrain(cfg.zones[i].radius_m, 5, 2000);
    }
    if (cfg.hop_limit > 3) cfg.hop_limit = 3;   // Meshtastic hop limit cap
    cfg.short_name[sizeof(cfg.short_name) - 1] = 0;
    cfg.text[sizeof(cfg.text) - 1] = 0;
    recompute();   // re-derive in case the preset/region tables changed
  }

  // Detailed help to the serial console (the reply buffer is too small for it).
  // The command surface is split: `mtbeacon` tunes the Meshtastic beacon itself,
  // `carnode` tunes the car-specific park behaviour.
  void printBeaconHelp() {
    Serial.println(F("mtbeacon commands (the Meshtastic beacon):"));
    Serial.println(F("  status             show beacon RF config"));
    Serial.println(F("  stats [clear]      transmit health since boot (or reset it)"));
    Serial.println(F("  on | off           enable / disable beaconing"));
    Serial.println(F("  send               push a location update now (both networks)"));
    Serial.println(F("  interval <min>     periodic presence between parks, 0-1440 (0=park-only)"));
    Serial.println(F("  preset <name>      modem preset (LongFast, MediumFast, ...)"));
    Serial.println(F("  region <name>      region/country band (US, EU_868, ...)"));
    Serial.println(F("  freq <MHz|auto>    manual frequency override; auto = region+preset"));
    Serial.println(F("  power <dBm>        TX power, -9..22 (capped to region limit)"));
    Serial.println(F("  hops <0-3>         Meshtastic hop limit for presence + text (default 0)"));
    Serial.println(F("  text <string>      the chat-message content (<=63 chars)"));
    Serial.println(F("  text.mult <N>      chat text N times per flood-advert period (0=never)"));
    Serial.println(F("  short <str|auto>   Meshtastic short name / map label, <=4 chars"));
    Serial.println(F("  nodeinfo on|off    include NodeInfo (named node 'MC <name>')"));
    Serial.println(F("  position on|off    include Position (map pin) from live GPS"));
    Serial.println(F("  telemetry on|off   include Telemetry (battery + uptime)"));
    Serial.println(F("  presets / regions  list available values"));
    Serial.println(F("Car-specific timing lives under 'carnode' (park / radius / zone)."));
  }

  void printCarHelp() {
    Serial.println(F("carnode commands (mobile/park behaviour):"));
    Serial.println(F("  status             show drive state + park config"));
    Serial.println(F("  on | off | send    enable/disable/push now (also under 'mtbeacon')"));
    Serial.println(F("  park <sec>         stopped time before an update fires, 30-86400"));
    Serial.println(F("  radius <m>         movement within this counts as stopped, 5-2000"));
    Serial.println(F("  advertdelay <sec>  gap: Meshtastic burst -> MeshCore advert, 0-600"));
    Serial.println(F("  sleep <hours>      parked this long -> stop repeating until driving, 0=never"));
    Serial.println(F("  zone               list the quiet zones (max 4)"));
    Serial.println(F("  zone add <name>    quiet zone = here; parking in it -> repeat off, radio quiet"));
    Serial.println(F("  zone del <name>    forget that zone"));
    Serial.println(F("  zone radius <name> <m>   how close counts, 5-2000 (default 100)"));
    Serial.println(F("  home [clear|radius <m>]  shorthand for the zone named 'home'"));
    Serial.println(F("A location update fires ONCE when the vehicle parks (stopped >park sec),"));
    Serial.println(F("pushing the Meshtastic beacon AND a MeshCore re-advert together. Nothing"));
    Serial.println(F("is sent while driving; re-parking the same spot won't re-broadcast."));
    Serial.println(F("Beacon RF/appearance lives under 'mtbeacon' (preset/region/text/...)."));
  }

  // append " <int>.<3frac>" style float for echoes / status
  static void appendFreq(char* dst, float f) {
    long i = (long)f;
    long frac = (long)((f - i) * 1000.0f + 0.5f);
    sprintf(dst + strlen(dst), "%ld.%03ld", i, frac);
  }

  uint32_t nextId() { uint32_t id = packet_id++; if (packet_id == 0) packet_id = 1; return id; }

  // TX power actually used: configured power clamped to the region's legal cap.
  int8_t effectivePower() const {
    int8_t cap = meshtastic::REGIONS[cfg.region_idx].max_power_dbm;
    return cfg.tx_power < cap ? cfg.tx_power : cap;
  }

  // Listen-before-talk: best-effort CAD on the (already-tuned) Meshtastic
  // channel. Returns true if it looks clear. Falls back to "proceed" if the
  // RadioLib CAD primitive isn't available.
  //
  // NOTE: we deliberately do NOT call RadioLib's blocking radio.scanChannel():
  // its internal wait `while(!digitalRead(irq)) yield();` has no timeout, so a
  // single missed CAD-done interrupt (e.g. after an aborted transmit leaves the
  // radio in an odd state) spins the CPU forever and hard-hangs the whole node
  // (the main loop never returns, serial goes dead — recoverable only by a power
  // cycle). Instead we start the scan and poll the result under a millis()
  // deadline, so LBT can never wedge the repeater. (Ported from mtbeacon v0.2.1.)
  template <class R>
  bool channelClear(R& radio) {
#ifdef RADIOLIB_CHANNEL_FREE
    for (int i = 0; i < 4; i++) {
      if (radio.startChannelScan() != RADIOLIB_ERR_NONE) return true;  // can't CAD -> proceed
      unsigned long t0 = millis();
      int16_t r = RADIOLIB_ERR_UNKNOWN;               // "still scanning" until CAD latches
      while ((unsigned long)(millis() - t0) < 50) {   // bounded wait for a CAD result
        r = radio.getChannelScanResult();
        if (r != RADIOLIB_ERR_UNKNOWN) break;         // CAD_DONE or CAD_DETECTED
        yield();
      }
      radio.standby();                                 // leave CAD mode deterministically
      if (r == RADIOLIB_CHANNEL_FREE) return true;     // clear -> transmit
      if (r != RADIOLIB_LORA_DETECTED) return true;    // timed out / error -> proceed, never hang
      delay(20 + (long)random(0, 80));                 // activity detected: back off, re-check
    }
    return false;                                      // busy on all attempts -> skip this burst
#else
    (void)radio; return true;
#endif
  }

  // Build packet kind k (0=NodeInfo, 1=Position, 2=Text, 3=Telemetry) into pkt[].
  // Returns wire length, or 0 if that kind is disabled / unavailable.
  int buildKind(uint8_t k, uint8_t* pkt, size_t cap, const Context& c) {
    uint8_t pl[240];
    const uint8_t* key = meshtastic::DEFAULT_KEY;
    const size_t klen = sizeof(meshtastic::DEFAULT_KEY);
    if (k == 3) {
      if (!cfg.send_telemetry) return 0;
      int n = meshtastic::buildTelemetryPayload(pl, c.batt_millivolts,
                                                c.uptime_secs, c.epoch);
      if (n <= 0) return 0;              // nothing measurable to report
      return meshtastic::buildDataPacket(pkt, cap, node_num, nextId(),
                meshtastic::PORT_TELEMETRY, pl, n, key, klen, chan_hash, cfg.hop_limit);
    }
    if (k == 0) {
      if (!cfg.send_nodeinfo) return 0;
      char ln[44], sn[5];
      snprintf(ln, sizeof(ln), "MC %s", (c.node_name && *c.node_name) ? c.node_name : "Mobile");
      meshtastic::resolveShortName(sn, cfg.short_name, node_num);
      int n = meshtastic::buildUserPayload(pl, node_num, ln, sn, MT_HW_MODEL);
      return meshtastic::buildDataPacket(pkt, cap, node_num, nextId(),
                meshtastic::PORT_NODEINFO, pl, n, key, klen, chan_hash, cfg.hop_limit);
    } else if (k == 1) {
      if (!cfg.send_position || !c.gps_valid) return 0;
      int n = meshtastic::buildPositionPayload(pl, c.lat, c.lon, c.epoch);
      return meshtastic::buildDataPacket(pkt, cap, node_num, nextId(),
                meshtastic::PORT_POSITION, pl, n, key, klen, chan_hash, cfg.hop_limit);
    }
    return meshtastic::buildTextPacket(pkt, cap, node_num, nextId(),
                cfg.text, key, klen, chan_hash, cfg.hop_limit);
  }

  // Schedule the next periodic presence, with up to 20 s of random jitter so we
  // don't lock-step onto the same airtime as other beacons. 0 = feature off.
  void scheduleNextPresence() {
    next_presence = (cfg.interval_mins == 0) ? 0
      : millis() + (unsigned long)cfg.interval_mins * 60000UL
                 + (unsigned long)random(0, 20001);
  }

  // One retune: transmit the given packet kinds back-to-back (LBT + duty-cycle
  // accounting), then restore the MeshCore PHY. Returns false if the channel was
  // busy or nothing left the antenna — the caller should retry.
  template <class D, class R>
  bool sendKinds(D& driver, R& radio, const Context& c, bool presence) {
    // Cheap pre-check before touching the radio: with everything switched off
    // there is nothing to say, so don't burn a retune to find that out.
    if (!cfg.send_nodeinfo && !cfg.send_position && !cfg.send_telemetry && !pending_text)
      return true;

    meshtastic::ModemPreset mt = { cfg.freq, cfg.bw, cfg.sf, cfg.cr, cfg.preamble, cfg.sync_word };
    meshtastic::radioEnterMeshtastic(driver, radio, mt, effectivePower());

    if (!channelClear(radio)) {                       // listen-before-talk
      tx.lbt_skips++;
      meshtastic::radioRestoreMeshCore(driver, radio, c.home_freq, c.home_bw,
                  c.home_sf, c.home_cr, c.home_sync, c.home_tx_power);
      return false;
    }

    // Choose what to send from INSIDE the tuned window. getEstAirtimeFor() asks
    // the radio for its current time-on-air, so the rotate decision is only
    // meaningful once we're on the Meshtastic PHY — asked a line earlier it
    // would answer for the (typically much faster) MeshCore PHY and never
    // rotate, however slow the Meshtastic preset is.
    uint8_t kinds[5]; int nk = 0;
    if (presence) {
      // Light periodic presence: NodeInfo(0) + Position(1) + Telemetry(3).
      // Position only while PARKED — it refreshes the pin at the parked spot,
      // whereas mid-drive it would strand the pin at a random point. At slow
      // presets send one kind per cycle (round-robin) to bound the window.
      bool pos_ok = cfg.send_position && c.gps_valid && drive_state == 2;
      uint8_t pres[3]; uint8_t np = 0;
      if (cfg.send_nodeinfo)  pres[np++] = 0;
      if (pos_ok)             pres[np++] = 1;
      if (cfg.send_telemetry) pres[np++] = 3;
      if (np > 0) {
        if ((driver.getEstAirtimeFor(60) * 2 + 120) > 2500) {
          if (presence_rot >= np) presence_rot = 0;
          kinds[nk++] = pres[presence_rot++];
        } else {
          for (uint8_t i = 0; i < np; i++) kinds[nk++] = pres[i];
        }
      }
    } else {
      // Park / manual burst — the full location update. A park is infrequent and
      // the whole point is the location, so ALWAYS send both Position and
      // NodeInfo. Position goes TWICE (first and last) for redundancy against a
      // missed broadcast on the busy public LongFast channel: the two copies are
      // separated by the NodeInfo so one collision is unlikely to take out both,
      // and each gets its own packet id (so they aren't deduped). The rotate
      // trick above would update the node but not its location on a park —
      // wrong for a car node. Telemetry rides along because a parked car is
      // exactly when its battery is worth knowing.
      kinds[nk++] = 1;
      kinds[nk++] = 0;
      kinds[nk++] = 1;
      if (cfg.send_telemetry) kinds[nk++] = 3;
    }
    if (pending_text) kinds[nk++] = 2;

    uint8_t pkt[256];
    uint32_t air = 0;
    bool first = true;
    bool wedged = false;
    int sent = 0, tried = 0, built = 0, irq_misses = 0, lost = 0;
    for (int i = 0; i < nk; i++) {
      int len = buildKind(kinds[i], pkt, sizeof(pkt), c);
      if (len <= 0) continue;                          // disabled / unavailable
      built++;
      if (!first) delay(120);                          // inter-packet gap
      meshtastic::TxOutcome r = meshtastic::radioSendChecked(driver, radio, pkt, len);
      tried++;
      if (meshtastic::txStatsRecord(tx, r, millis()) >= MT_BEACON_FAIL_STREAK)
        wedged = true;                                 // run of dead transmits: re-init below
      if (r == meshtastic::TX_NOT_STARTED) break;      // radio wouldn't start: abort burst
      if (meshtastic::txUsedFallback(r)) irq_misses++;
      air += driver.getEstAirtimeFor(len);
      first = false;
      if (!meshtastic::txDelivered(r)) { lost++; continue; }   // chip says it never went out
      sent++;
      if (kinds[i] == 2) { pending_text = false; last_text_ms = millis(); }  // text delivered
    }
    if (sent) tx.bursts++;
    // Diagnostics. A missed TxDone interrupt no longer means the packet is lost:
    // we ask the chip's own TxDone flag, so "IRQ missed" and "never transmitted"
    // are separate reports instead of one guess. (mtbeacon v0.2.5)
    if (irq_misses)
      Serial.printf("carnode: TxDone IRQ missed on %d/%d packet(s) - checked the chip instead\n",
                    irq_misses, tried);
    if (lost)
      Serial.printf("carnode: %d/%d packet(s) did NOT transmit (chip reports no TxDone)\n",
                    lost, tried);

    if (wedged) {
      // The chip has now reported several transmits in a row as never-sent.
      // Re-initialise instead of restoring, so the next park update isn't dead
      // too — a car node that stops transmitting is a car you can't find.
      tx.recoveries++;
      tx.fail_streak = 0;                              // give the re-init a clean run
      Serial.printf("carnode: %d dead transmits in a row - reinitialising the radio\n",
                    (int)MT_BEACON_FAIL_STREAK);
      meshtastic::radioRecover(driver, radio, c.home_freq, c.home_bw,
                  c.home_sf, c.home_cr, c.home_sync, c.home_tx_power);
    } else {
      meshtastic::radioRestoreMeshCore(driver, radio, c.home_freq, c.home_bw,
                  c.home_sf, c.home_cr, c.home_sync, c.home_tx_power);
    }

    // duty cycle: hold off next TX by on-time*(100-duty)/duty for limited regions
    uint8_t duty = meshtastic::REGIONS[cfg.region_idx].duty_pct;
    if (duty > 0 && duty < 100)
      hold_until = millis() + (unsigned long)air * (100 - duty) / duty;
    // Nothing BUILDABLE this cycle (e.g. position on but no fix yet) is a no-op,
    // not a failure — reporting failure there would put the caller in a 15-second
    // retry loop forever. Only a burst that tried and got nothing onto the air
    // asks to be retried.
    if (built == 0) return true;
    return sent > 0;
  }

  // Park/manual burst — the full location update. See sendKinds() for what goes
  // out and why; the packet selection lives there because it depends on the
  // Meshtastic modem params, which are only in effect after the retune.
  template <class D, class R>
  bool sendBurst(D& driver, R& radio, const Context& c) {
    return sendKinds(driver, radio, c, /*presence=*/false);
  }

  // Periodic light presence (base-mtbeacon style) — keeps the node from aging
  // out of Meshtastic node lists between park events. Position is included only
  // while PARKED (it refreshes the pin at the parked spot); while driving or
  // without a fix it's NodeInfo only, so the map pin never wanders off to some
  // random mid-drive point and sticks there. At slow presets one packet per
  // cycle (alternating) bounds the off-channel window, like the base beacon.
  template <class D, class R>
  bool sendPresence(D& driver, R& radio, const Context& c) {
    return sendKinds(driver, radio, c, /*presence=*/true);
  }

  // v8 and earlier kept a single "home" spot in its own fields. Move it into the
  // zone table under the name "home", so everything downstream has exactly one
  // code path, and retire the old flag. An operator who set a home before
  // upgrading keeps it, with its radius, and `carnode home` still finds it.
  void adoptLegacyHome() {
    if (!cfg.have_home) return;
    cfg.zones[0].used = 1;
    cfg.zones[0].lat = cfg.home_lat;
    cfg.zones[0].lon = cfg.home_lon;
    cfg.zones[0].radius_m = cfg.home_radius_m;
    strncpy(cfg.zones[0].name, "home", sizeof(cfg.zones[0].name) - 1);
    cfg.zones[0].name[sizeof(cfg.zones[0].name) - 1] = 0;
    cfg.have_home = 0;
  }

public:
  void load(FILESYSTEM* fs) {
#if defined(RP2040_PLATFORM)
    File f = fs->open(CAR_NODE_FILE, "r");
#else
    File f = fs->open(CAR_NODE_FILE);
#endif
    if (f) {
      uint8_t buf[sizeof(Config)];
      memset(buf, 0, sizeof(buf));
      int n = f.read(buf, sizeof(buf));
      f.close();
      uint32_t magic = 0;
      if (n >= 4) memcpy(&magic, buf, sizeof(magic));
      if (n == (int)sizeof(Config) && magic == MAGIC) {
        memcpy(&cfg, buf, sizeof(cfg));
        sanitize();
      } else if ((magic == MAGIC_V5 || magic == MAGIC_V6 || magic == MAGIC_V7 ||
                  magic == MAGIC_V8) && n > 4 && n <= (int)sizeof(Config)) {
        // Every version so far has only APPENDED fields, so the older prefix
        // layout is unchanged — copy what the file has (bytes past `n` keep the
        // setDefaults() values), then force defaults for the fields that version
        // didn't have, since the copy may have clobbered ones that landed in old
        // padding. Park/zone/sleep settings are preserved.
        memcpy(&cfg, buf, n);
        cfg.magic = MAGIC;
        if (magic == MAGIC_V5) {           // v5 had no home
          cfg.have_home = 0;
          cfg.home_radius_m = 100;
          cfg.home_lat = cfg.home_lon = 0;
        }
        if (magic == MAGIC_V5 || magic == MAGIC_V6) cfg.hop_limit = 0;  // no hop knob
        if (magic != MAGIC_V8) cfg.short_name[0] = 0;   // v7 and earlier -> auto
        cfg.send_telemetry = 1;                         // v8 and earlier had no telemetry knob
        memset(cfg.zones, 0, sizeof(cfg.zones));        // ... and no zone table
        adoptLegacyHome();
        sanitize();
      }
    }
  }

  void save(FILESYSTEM* fs) {
#if defined(NRF52_PLATFORM) || defined(STM32_PLATFORM)
    fs->remove(CAR_NODE_FILE);
    File f = fs->open(CAR_NODE_FILE, FILE_O_WRITE);
#elif defined(RP2040_PLATFORM)
    File f = fs->open(CAR_NODE_FILE, "w");
#else
    File f = fs->open(CAR_NODE_FILE, "w", true);
#endif
    if (f) { f.write((uint8_t*)&cfg, sizeof(cfg)); f.close(); }
  }

  // node: stable Meshtastic node number; seed: starting packet id.
  void begin(FILESYSTEM* fs, uint32_t node, uint32_t seed) {
    setDefaults();
    load(fs);   // load() -> sanitize() -> recompute() sets derived params + chan_hash
    node_num = (node == 0 || node == 0xFFFFFFFF) ? 0x0DECAFEDUL : node;
    packet_id = seed | 1u;
    last_text_ms = millis();   // wait one full text period before the first auto post
  }

  bool enabled() const { return cfg.enabled; }

  // The repeater just flooded a MeshCore advert (periodic timer, CLI 'advert',
  // or our own park re-advert): have the chat text ride a Meshtastic burst
  // shortly after, so the text follows the real advert cadence instead of a
  // boot-reset stopwatch (which, on a node that power-cycles with the ignition,
  // never reaches a full 47h period and so never fires). textDue() still paces
  // extra texts between adverts when text_mult > 1. In a quiet zone nothing
  // changes: presence stays suppressed, the text just stays pending until driving.
  //
  // NOT called for our own park re-advert: that burst has just gone out and
  // already carried the text (armed in tick()), and the advert is still sitting
  // in the send queue behind `advertdelay` -- scheduling another Meshtastic
  // burst 15 s out would drop a second off-channel retune right on top of it.
  void onFloodAdvert() {
    if (!cfg.enabled || cfg.text_mult == 0) return;
    pending_text = true;
    if (cfg.interval_mins > 0 && next_presence != 0) {   // pull the next presence close
      unsigned long soon = millis() + 15000;             // let the advert TX clear the air
      if ((int32_t)(next_presence - soon) > 0) next_presence = soon;
    }
  }

  // Hand a pending MeshCore re-advert to the repeater. Returns true once per
  // park event (clears the flag); the repeater then writes lat/lon into its
  // NodePrefs and floods a fresh advert. If that flood cannot be queued the
  // repeater calls readvertFailed() and we hand it back after a backoff --
  // without this the park is announced on Meshtastic and never on MeshCore.
  bool takeReadvert(double& lat, double& lon) {
    if (!readvert_pending) return false;
    if (readvert_retry_at != 0 && (int32_t)(millis() - readvert_retry_at) < 0) return false;
    readvert_pending = false;
    lat = readvert_lat;
    lon = readvert_lon;
    return true;
  }

  // The repeater could not queue the flood advert (no free packet, or the send
  // queue was full). Re-arm the handoff with a linear backoff so it rides out a
  // transient burst of forwarding. After MAX_TRIES we give up and let the next
  // periodic flood advert carry the location -- it is already in NodePrefs.
  void readvertFailed() {
    if (++readvert_tries >= CAR_NODE_READVERT_MAX_TRIES) {
      if (readvert_drops < 255) readvert_drops++;
      readvert_tries = 0;
      readvert_retry_at = 0;
      Serial.println(F("carnode: park re-advert dropped - mesh queue full, giving up"));
      return;
    }
    readvert_pending = true;
    readvert_retry_at = millis() + CAR_NODE_READVERT_RETRY_MS * readvert_tries;
  }

  // The flood advert made it onto the send queue.
  void readvertQueued() { readvert_tries = 0; readvert_retry_at = 0; }

  // How long the repeater should delay the MeshCore advert after the Meshtastic
  // burst (ms), so the two park transmissions don't land on top of each other.
  uint32_t advertDelayMs() const { return (uint32_t)cfg.advert_delay_s * 1000UL; }

  // True while the repeater should stop forwarding, for either reason:
  //   * parked inside a QUIET ZONE (`carnode zone add <name>`) — repeat goes off
  //     as soon as the park is detected, no waiting; a place you park often
  //     presumably has fixed coverage already; or
  //   * parked anywhere past `carnode sleep <hours>` — a car parked for a day
  //     is probably somewhere nobody needs a mobile repeater.
  // The repeater polls this each loop and toggles its actual 'repeat' pref to
  // match (the same switch as 'set repeat on|off'), so the state is visible
  // everywhere. Keyed off the park anchor (not drive_state), so losing the GPS
  // fix in a garage does NOT wake the repeater; only actually moving does.
  // Clears itself as soon as driving resumes (the anchor follows the vehicle,
  // the clock restarts, in_zone drops). Runtime-only: nothing is persisted
  // here, so a reboot starts awake.
  bool repeatSuppressed() const {
    if (!cfg.enabled || !have_anchor) return false;
    if (in_zone >= 0) return true;
    if (cfg.sleep_hours == 0) return false;
    return (unsigned long)(millis() - stationary_since) >=
           (unsigned long)cfg.sleep_hours * 3600000UL;
  }

  // Compact one-line status for the repeater's home screen. Inside a quiet zone
  // it shows the zone's name, so "which of my places is it at" is on the screen.
  void uiLine(char* out, size_t n) const {
    if (!cfg.enabled) { snprintf(out, n, "CarNode off"); return; }
    if (in_zone >= 0) { snprintf(out, n, "CarNode %s", cfg.zones[in_zone].name); return; }
    const char* s = repeatSuppressed() ? "sleeping"
                  : drive_state == 2 ? "parked" : drive_state == 1 ? "driving" : "no fix";
    snprintf(out, n, "CarNode %s", s);
  }

  // `mtbeacon status` — the Meshtastic beacon's RF config + appearance.
  void beaconStatus(char* reply) {
    const meshtastic::Preset& p = meshtastic::PRESETS[cfg.preset_idx];
    const meshtastic::Region& r = meshtastic::REGIONS[cfg.region_idx];
    int8_t ep = effectivePower();
    char fbuf[14] = {0};
    appendFreq(fbuf, cfg.freq);
    char txt[32];
    if (cfg.text_mult == 0) strcpy(txt, "txt:off");
    else if (flood_hours_seen == 0) snprintf(txt, sizeof(txt), "txt%dx(noadv)", (int)cfg.text_mult);
    else {
      // live countdown to the next timer-paced text ("due now" = armed, rides
      // the next burst). A flood-advert event can pull it in sooner.
      char due[10];
      unsigned long period = (unsigned long)flood_hours_seen * 3600000UL / cfg.text_mult;
      unsigned long since = (unsigned long)(millis() - last_text_ms);
      if (pending_text || since >= period) strcpy(due, "now");
      else {
        unsigned long left = (period - since) / 60000UL;   // minutes remaining
        if (left >= 60) snprintf(due, sizeof(due), "%luh", left / 60);
        else snprintf(due, sizeof(due), "%lum", left);
      }
      snprintf(txt, sizeof(txt), "txt%dx~%dh(due %s)", (int)cfg.text_mult,
               (int)(flood_hours_seen / cfg.text_mult), due);
    }
    char ivl[10];
    if (cfg.interval_mins == 0) strcpy(ivl, "i:park");
    else snprintf(ivl, sizeof(ivl), "i%dm", (int)cfg.interval_mins);
    char sn[5];
    meshtastic::resolveShortName(sn, cfg.short_name, node_num);
    // "!<id>/<short>" = Meshtastic node id and the short name / map label
    snprintf(reply, 160,
             "mtbeacon %s %sMHz%s %s %s(SF%d BW%d) %s %ddBm%s h%d %s%s%s %s !%08lx/%s",
             cfg.enabled ? "ON" : "off", fbuf, cfg.freq_override > 0.0f ? "*" : "",
             r.name, p.name, (int)cfg.sf, (int)cfg.bw, ivl,
             (int)ep, ep < cfg.tx_power ? "(cap)" : "", (int)cfg.hop_limit,
             cfg.send_nodeinfo ? "+info" : "", cfg.send_position ? "+pos" : "",
             cfg.send_telemetry ? "+tel" : "",
             txt, (unsigned long)node_num, sn);
  }

  // `mtbeacon stats` — transmit health since boot. RAM only; see TxStats.
  void statsReply(char* reply) {
    meshtastic::formatTxStats(reply, 160, tx, "carnode",
                              (uint32_t)(millis() - tx.last_fail_ms));
  }

  // `carnode zone` — the quiet-zone table, plus which one we're sitting in.
  void zoneReply(char* reply) {
    if (zoneCount() == 0) {
      strcpy(reply, "no quiet zones - 'carnode zone add <name>' captures this spot");
      return;
    }
    snprintf(reply, 160, "zones (%d/%d):", (int)zoneCount(), (int)MAX_ZONES);
    for (uint8_t i = 0; i < MAX_ZONES; i++) {
      if (!cfg.zones[i].used) continue;
      char one[40];
      snprintf(one, sizeof(one), " %s%s/%dm", cfg.zones[i].name,
               (in_zone >= 0 && (uint8_t)in_zone == i) ? "*" : "",
               (int)cfg.zones[i].radius_m);
      strncat(reply, one, 160 - strlen(reply) - 1);
    }
    if (in_zone >= 0) strncat(reply, " (* = here)", 160 - strlen(reply) - 1);
  }

  // `carnode status` — the car-specific park behaviour + drive state.
  // The [..] block includes live tracking: distance of the current fix from the
  // park anchor and how long the stationary clock has been running — so GPS
  // jitter (fix wandering past `radius`, which restarts the clock) is visible.
  void carStatus(char* reply) {
    const char* st = in_zone >= 0 ? cfg.zones[in_zone].name
                   : repeatSuppressed() ? "sleeping"
                   : drive_state == 2 ? "parked" : drive_state == 1 ? "driving" : "nofix";
    char trk[32] = {0};
    if (have_anchor && last_dist_m >= 0) {
      unsigned long mins = (unsigned long)(millis() - stationary_since) / 60000UL;
      snprintf(trk, sizeof(trk), " d%dm%s %lumin", last_dist_m,
               fix_filtered ? "" : "(raw)", mins);   // (raw) = median filter not warmed up yet
    }
    char slp[10];
    if (cfg.sleep_hours == 0) strcpy(slp, "slp:off");
    else snprintf(slp, sizeof(slp), "slp%dh", (int)cfg.sleep_hours);
    char hm[10];
    if (zoneCount() == 0) strcpy(hm, "z:off");
    else snprintf(hm, sizeof(hm), "z%d/%d", (int)zoneCount(), (int)MAX_ZONES);
    // "adv!N" = N park re-adverts the mesh could not queue (see readvertFailed);
    // omitted entirely when zero, which is the normal case.
    char drops[12] = {0};
    if (readvert_drops) snprintf(drops, sizeof(drops), " adv!%d", (int)readvert_drops);
    snprintf(reply, 160,
             "carnode %s [%s%s] park%ds r%dm adv%ds %s %s loc:%s%s !%08lx",
             cfg.enabled ? "ON" : "off", st, trk,
             (int)cfg.park_secs, (int)cfg.stop_radius_m, (int)cfg.advert_delay_s,
             slp, hm, have_advert ? "set" : "none", drops, (unsigned long)node_num);
  }

  // If `command` is `<verb>` (optionally followed by args), return a pointer to
  // the args with leading spaces skipped; else nullptr. Requires a word boundary
  // so e.g. "carnodex" does not match "carnode".
  static const char* matchVerb(const char* command, const char* verb) {
    size_t n = strlen(verb);
    if (strncmp(command, verb, n) != 0) return nullptr;
    if (command[n] != 0 && command[n] != ' ') return nullptr;
    const char* a = command + n;
    while (*a == ' ') a++;
    return a;
  }

  // Verb dispatch. This build routes BOTH `mtbeacon` (beacon RF/appearance) and
  // `carnode` (mobile/park behaviour) here; returns false for anything else so
  // the caller falls through to the common CLI.
  bool handleCommand(char* command, char* reply, FILESYSTEM* fs) {
    const char* a;
    if ((a = matchVerb(command, "mtbeacon")) != nullptr) return handleBeacon(a, reply, fs);
    if ((a = matchVerb(command, "carnode"))  != nullptr) return handleCar(a, reply, fs);
    return false;
  }

  // `mtbeacon ...` — configure the Meshtastic beacon itself.
  bool handleBeacon(const char* a, char* reply, FILESYSTEM* fs) {
    if (*a == 0 || strcmp(a, "status") == 0) {
      beaconStatus(reply);
    } else if (strcmp(a, "stats") == 0) {
      statsReply(reply);
    } else if (strcmp(a, "stats clear") == 0) {
      tx = meshtastic::TxStats();
      strcpy(reply, "OK - tx stats cleared");
    } else if (strcmp(a, "help") == 0 || strcmp(a, "?") == 0) {
      printBeaconHelp();
      strcpy(reply, "mtbeacon: status stats on off send | interval preset region freq power hops text text.mult short nodeinfo position telemetry | presets regions");
    } else if (memcmp(a, "nodeinfo ", 9) == 0) {
      cfg.send_nodeinfo = (strcasecmp(a + 9, "on") == 0) ? 1 : 0; save(fs);
      sprintf(reply, "OK - nodeinfo %s", cfg.send_nodeinfo ? "on" : "off");
    } else if (memcmp(a, "telemetry ", 10) == 0) {
      cfg.send_telemetry = (strcasecmp(a + 10, "on") == 0) ? 1 : 0; save(fs);
      sprintf(reply, "OK - telemetry %s", cfg.send_telemetry ? "on" : "off");
    } else if (memcmp(a, "position ", 9) == 0) {
      cfg.send_position = (strcasecmp(a + 9, "on") == 0) ? 1 : 0; save(fs);
      sprintf(reply, "OK - position %s", cfg.send_position ? "on" : "off");
    } else if (strcmp(a, "on") == 0) {
      cfg.enabled = 1; save(fs); strcpy(reply, "OK - beacon on");
    } else if (strcmp(a, "off") == 0) {
      cfg.enabled = 0; save(fs); strcpy(reply, "OK - beacon off");
    } else if (strcmp(a, "send") == 0) {
      pending_send = true; pending_text = true;
      strcpy(reply, "OK - pushing location update shortly");
    } else if (strcmp(a, "presets") == 0) {
      strcpy(reply, "presets:");
      for (uint8_t i = 0; i < meshtastic::NUM_PRESETS; i++) {
        strcat(reply, " "); strcat(reply, meshtastic::PRESETS[i].name);
      }
    } else if (strcmp(a, "regions") == 0) {
      strcpy(reply, "regions:");
      for (uint8_t i = 0; i < meshtastic::NUM_REGIONS; i++) {
        strcat(reply, " "); strcat(reply, meshtastic::REGIONS[i].name);
      }
    } else if (memcmp(a, "preset ", 7) == 0) {
      int idx = meshtastic::findPreset(a + 7);
      if (idx < 0) { strcpy(reply, "Error: unknown preset (try 'mtbeacon presets')"); }
      else {
        cfg.preset_idx = idx; cfg.freq_override = 0.0f; recompute(); save(fs);
        const meshtastic::Preset& p = meshtastic::PRESETS[idx];
        strcpy(reply, "OK - "); strcat(reply, p.name); strcat(reply, " @ ");
        appendFreq(reply, cfg.freq); strcat(reply, " MHz");
      }
    } else if (memcmp(a, "region ", 7) == 0 || memcmp(a, "country ", 8) == 0) {
      const char* arg = (a[0] == 'r') ? a + 7 : a + 8;
      int idx = meshtastic::findRegion(arg);
      if (idx < 0) { strcpy(reply, "Error: unknown region (try 'mtbeacon regions')"); }
      else {
        cfg.region_idx = idx; cfg.freq_override = 0.0f; recompute(); save(fs);
        strcpy(reply, "OK - "); strcat(reply, meshtastic::REGIONS[idx].name);
        strcat(reply, " @ "); appendFreq(reply, cfg.freq); strcat(reply, " MHz");
      }
    } else if (memcmp(a, "freq ", 5) == 0) {
      const char* arg = a + 5;
      if (strcasecmp(arg, "auto") == 0 || strtof(arg, nullptr) == 0.0f) {
        cfg.freq_override = 0.0f; recompute(); save(fs);
        strcpy(reply, "OK - freq auto: "); appendFreq(reply, cfg.freq); strcat(reply, " MHz");
      } else {
        float f = strtof(arg, nullptr);
        if (f < 150.0f || f > 960.0f) { strcpy(reply, "Error: freq 150-960 MHz (or 'auto')"); }
        else { cfg.freq_override = f; recompute(); save(fs);
               strcpy(reply, "OK - freq "); appendFreq(reply, f); strcat(reply, " MHz (override)"); }
      }
    } else if (memcmp(a, "interval ", 9) == 0) {
      int m = atoi(a + 9);
      if (m < 0 || m > 1440) { strcpy(reply, "Error: interval 0-1440 min (0 = park-only)"); }
      else { cfg.interval_mins = m; scheduleNextPresence(); save(fs);
             if (m == 0) strcpy(reply, "OK - periodic presence off (park-only)");
             else sprintf(reply, "OK - presence every %d min", m); }
    } else if (memcmp(a, "power ", 6) == 0) {
      int p = atoi(a + 6);
      if (p < -9 || p > 22) { strcpy(reply, "Error: power -9..22 dBm"); }
      else { cfg.tx_power = p; save(fs); sprintf(reply, "OK - %d dBm", p); }
    } else if (memcmp(a, "hops ", 5) == 0) {
      int h = atoi(a + 5);
      if (h < 0 || h > 3) { strcpy(reply, "Error: hops 0-3"); }
      else { cfg.hop_limit = (uint8_t)h; save(fs);
             sprintf(reply, "OK - hop limit %d%s", h, h == 0 ? " (neighbors only)" : ""); }
    } else if (memcmp(a, "text.mult ", 10) == 0) {
      int n = atoi(a + 10);
      if (n < 0 || n > 255) { strcpy(reply, "Error: 0-255 (0 = never post text)"); }
      else { cfg.text_mult = n; save(fs);
             if (n == 0) strcpy(reply, "OK - text off (silent presence only)");
             else sprintf(reply, "OK - text %dx per flood advert", n); }
    } else if (memcmp(a, "short ", 6) == 0) {
      const char* arg = a + 6;
      char sn[5];
      if (*arg == 0 || strcasecmp(arg, "auto") == 0) {
        cfg.short_name[0] = 0;                        // auto: "MC" + 2 hex
      } else {
        strncpy(cfg.short_name, arg, sizeof(cfg.short_name) - 1);
        cfg.short_name[sizeof(cfg.short_name) - 1] = 0;
      }
      save(fs);
      meshtastic::resolveShortName(sn, cfg.short_name, node_num);
      sprintf(reply, "OK - short name \"%s\"%s", sn, cfg.short_name[0] ? "" : " (auto)");
    } else if (memcmp(a, "text ", 5) == 0) {
      strncpy(cfg.text, a + 5, sizeof(cfg.text) - 1);
      cfg.text[sizeof(cfg.text) - 1] = 0;
      save(fs);
      sprintf(reply, "OK - \"%.40s\"", cfg.text);
    } else {
      strcpy(reply, "Unknown - try 'mtbeacon help'");
    }
    return true;
  }

  // `carnode ...` — configure the car-specific park behaviour.
  bool handleCar(const char* a, char* reply, FILESYSTEM* fs) {
    if (*a == 0 || strcmp(a, "status") == 0) {
      carStatus(reply);
    } else if (strcmp(a, "help") == 0 || strcmp(a, "?") == 0) {
      printCarHelp();
      strcpy(reply, "carnode: status on off send | park <sec> | radius <m> | advertdelay <sec> | sleep <hours> | zone [add|del|radius] | home");
    } else if (strcmp(a, "on") == 0) {          // alias of 'mtbeacon on'
      cfg.enabled = 1; save(fs); strcpy(reply, "OK - carnode on");
    } else if (strcmp(a, "off") == 0) {         // alias of 'mtbeacon off'
      cfg.enabled = 0; save(fs); strcpy(reply, "OK - carnode off");
    } else if (strcmp(a, "send") == 0) {        // alias of 'mtbeacon send'
      pending_send = true; pending_text = true; strcpy(reply, "OK - pushing location update shortly");
    } else if (memcmp(a, "park ", 5) == 0) {
      int s = atoi(a + 5);
      if (s < 30 || s > 86400) { strcpy(reply, "Error: park 30-86400 sec"); }
      else { cfg.park_secs = s; save(fs); sprintf(reply, "OK - update after %d sec stopped", s); }
    } else if (memcmp(a, "radius ", 7) == 0) {
      int m = atoi(a + 7);
      if (m < 5 || m > 2000) { strcpy(reply, "Error: radius 5-2000 m"); }
      else { cfg.stop_radius_m = m; save(fs); sprintf(reply, "OK - stopped = within %d m", m); }
    } else if (memcmp(a, "advertdelay ", 12) == 0) {
      int s = atoi(a + 12);
      if (s < 0 || s > 600) { strcpy(reply, "Error: advertdelay 0-600 sec"); }
      else { cfg.advert_delay_s = s; save(fs); sprintf(reply, "OK - MeshCore advert %d s after Meshtastic burst", s); }
    } else if (memcmp(a, "sleep ", 6) == 0) {
      int h = atoi(a + 6);
      if (h < 0 || h > 720) { strcpy(reply, "Error: sleep 0-720 hours (0 = never)"); }
      else { cfg.sleep_hours = h; save(fs);
             if (h == 0) strcpy(reply, "OK - repeat never sleeps");
             else sprintf(reply, "OK - repeat sleeps after %d h parked, wakes on driving", h); }
    } else if (memcmp(a, "zone", 4) == 0 && (a[4] == 0 || a[4] == ' ')) {
      const char* z = a + 4;
      while (*z == ' ') z++;
      handleZone(z, reply, fs);
    } else if (memcmp(a, "home", 4) == 0 && (a[4] == 0 || a[4] == ' ')) {
      // Back-compat: "home" is now just the zone named "home", so the commands
      // that existed before quiet zones keep working unchanged.
      const char* h = a + 4;
      while (*h == ' ') h++;
      if (*h == 0)                            handleZone("add home", reply, fs);
      else if (strcmp(h, "clear") == 0)       handleZone("del home", reply, fs);
      else if (memcmp(h, "radius ", 7) == 0) {
        char buf[32];
        snprintf(buf, sizeof(buf), "radius home %s", h + 7);
        handleZone(buf, reply, fs);
      } else {
        strcpy(reply, "Unknown - carnode home [clear | radius <m>]  (see 'carnode zone')");
      }
    } else {
      strcpy(reply, "Unknown - try 'carnode help'");
    }
    return true;
  }

  // Copy and validate a zone name out of `in`: 1-8 characters, no spaces.
  // Writes its own error into `reply` and returns false if it isn't usable.
  static bool zoneName(const char* in, char out[9], char* reply) {
    while (*in == ' ') in++;
    size_t n = 0;
    while (in[n] && in[n] != ' ' && n < 8) { out[n] = in[n]; n++; }
    out[n] = 0;
    if (n == 0) {
      strcpy(reply, "Error: zone needs a name, e.g. 'carnode zone add work'");
      return false;
    }
    if (in[n] && in[n] != ' ') {   // ran out of room before the name ended
      strcpy(reply, "Error: zone name max 8 chars");
      return false;
    }
    return true;
  }

  // `carnode zone ...` — the named quiet zones. Split out from handleCar because
  // `carnode home` routes here too: home is simply the zone named "home".
  void handleZone(const char* z, char* reply, FILESYSTEM* fs) {
    if (*z == 0 || strcmp(z, "list") == 0) {
      zoneReply(reply);
    } else if (memcmp(z, "add ", 4) == 0) {
      char name[9];
      if (!zoneName(z + 4, name, reply)) return;
      // Capture the FILTERED fix, same as the old 'carnode home': a zone pinned
      // to one multipath outlier would sit tens of metres off the real spot.
      if (!cur_fix_valid) { strcpy(reply, "Error: no GPS fix - can't set a zone"); return; }
      if (!fix_filtered)  { strcpy(reply, "Error: GPS filter warming up - retry in ~15 sec"); return; }
      int8_t i = zoneFind(name);                 // re-adding a name moves that zone here
      bool moved = (i >= 0);
      if (i < 0) i = zoneFree();
      if (i < 0) {
        snprintf(reply, 160, "Error: all %d zone slots in use - 'carnode zone del <name>' first",
                 (int)MAX_ZONES);
        return;
      }
      if (!moved) cfg.zones[i].radius_m = 100;   // generous: a driveway, not a parking bay
      cfg.zones[i].used = 1;
      cfg.zones[i].lat = cur_lat; cfg.zones[i].lon = cur_lon;
      strcpy(cfg.zones[i].name, name);
      save(fs);
      snprintf(reply, 160, "OK - zone \"%s\" %s here: parking within %d m turns repeat off",
               name, moved ? "moved to" : "set", (int)cfg.zones[i].radius_m);
    } else if (memcmp(z, "del ", 4) == 0) {
      char name[9];
      if (!zoneName(z + 4, name, reply)) return;
      int8_t i = zoneFind(name);
      if (i < 0) { snprintf(reply, 160, "Error: no zone named \"%s\"", name); return; }
      cfg.zones[i].used = 0;
      cfg.zones[i].name[0] = 0;
      if (in_zone == i) in_zone = -1;            // we may be sitting in the one just deleted
      save(fs);
      snprintf(reply, 160, "OK - zone \"%s\" removed", name);
    } else if (memcmp(z, "radius ", 7) == 0) {
      const char* arg = z + 7;
      while (*arg == ' ') arg++;
      const char* sp = strchr(arg, ' ');
      if (!sp) { strcpy(reply, "Error: carnode zone radius <name> <m>"); return; }
      char name[9];
      if (!zoneName(arg, name, reply)) return;
      int8_t i = zoneFind(name);
      if (i < 0) { snprintf(reply, 160, "Error: no zone named \"%s\"", name); return; }
      int m = atoi(sp + 1);
      if (m < 5 || m > 2000) { strcpy(reply, "Error: zone radius 5-2000 m"); return; }
      cfg.zones[i].radius_m = (uint16_t)m;
      save(fs);
      snprintf(reply, 160, "OK - zone \"%s\" = within %d m", name, m);
    } else {
      strcpy(reply, "Unknown - carnode zone [list | add <name> | del <name> | radius <name> <m>]");
    }
  }

  // Call every loop. `busy` should be true when the mesh has queued/in-flight
  // work, so the beacon never retunes mid-transaction. The Context carries the
  // current GPS fix. A unified update (Meshtastic burst + MeshCore re-advert
  // request) fires once when the vehicle parks, or immediately on "carnode send".
  template <class D, class R>
  void tick(D& driver, R& radio, bool busy, const Context& c) {
    flood_hours_seen = c.flood_advert_hours;                   // keep current for textDue/status
    unsigned long now = millis();

    // Median-filter the fix: everything below (park logic, dedup, and the
    // position that gets broadcast) sees the median of the recent samples,
    // so one teleporting fix can't fake movement or misplace the park pin.
    // Runs before the early returns so the filter stays warm through holds
    // and `carnode zone add` can capture the current spot even while disabled.
    Context fc = c;
    fix_filtered = c.gps_valid && filterFix(now, c.lat, c.lon, fc.lat, fc.lon);
    cur_fix_valid = fc.gps_valid;
    if (fc.gps_valid) { cur_lat = fc.lat; cur_lon = fc.lon; }

    // Log repeat-sleep transitions (the repeater polls repeatSuppressed() each
    // loop and toggles its repeat pref; this is just serial-console visibility).
    bool slp = repeatSuppressed();
    if (slp != sleep_announced) {
      sleep_announced = slp;
      if (!slp) {
        Serial.println(F("carnode: repeat back ON"));
      } else if (in_zone >= 0) {
        Serial.printf("carnode: parked in quiet zone \"%s\" - repeat OFF until driving\n",
                      cfg.zones[in_zone].name);
      } else {
        Serial.println(F("carnode: parked past sleep limit - repeat OFF until driving"));
      }
    }

    if (!cfg.enabled && !pending_send) return;
    if ((int32_t)(millis() - hold_until) < 0) return;          // duty-cycle / LBT hold

    // --- track movement -> detect the parked transition (needs a valid fix) ---
    // Movement must persist outside the radius for CAR_NODE_MOVE_CONFIRM_MS
    // before it counts as driving: a brief GPS excursion (multipath outlier)
    // neither re-anchors nor resets the parked/sleep clock.
    bool park_event = false;
    if (cfg.enabled && fc.gps_valid) {
      if (!have_anchor) {
        anchor_lat = fc.lat; anchor_lon = fc.lon; stationary_since = now;
        have_anchor = true; park_reported = false; drive_state = 1;
        outside_since = 0; last_dist_m = 0; in_zone = -1;
      } else {
        double d = distMeters(anchor_lat, anchor_lon, fc.lat, fc.lon);
        last_dist_m = (int)(d + 0.5);
        if (d > cfg.stop_radius_m) {
          if (outside_since == 0) outside_since = now;
          if ((unsigned long)(now - outside_since) >= CAR_NODE_MOVE_CONFIRM_MS) {
            anchor_lat = fc.lat; anchor_lon = fc.lon;          // moving: follow the vehicle
            stationary_since = now; park_reported = false; drive_state = 1;
            outside_since = 0; in_zone = -1;
          }
          // else: not confirmed yet — hold state, keep the stationary clock
        } else {
          outside_since = 0;                                   // sitting near the anchor
          if ((now - stationary_since) >= (unsigned long)cfg.park_secs * 1000UL) {
            drive_state = 2;                                   // parked
            if (!park_reported) park_event = true;
          }
          // Parked inside a quiet zone? Anchor vs stored zones — both stable, so
          // no flapping. Re-evaluated every tick so adding/removing a zone while
          // already parked takes effect immediately. NOT cleared on fix loss
          // (garage): like the sleep, only actually driving away wakes us.
          in_zone = (drive_state == 2) ? zoneAt(anchor_lat, anchor_lon) : -1;
        }
      }
    } else if (cfg.enabled) {
      drive_state = 0;   // no current GPS fix -> report "nofix" live (not latched)
    }

    // Log drive-state transitions to the serial console (diagnostics).
    if (cfg.enabled && drive_state != state_announced) {
      state_announced = drive_state;
      Serial.print(F("carnode: "));
      if (drive_state == 2 && in_zone >= 0) {
        Serial.print(F("parked in ")); Serial.print(cfg.zones[in_zone].name);
      } else {
        Serial.print(drive_state == 2 ? F("parked")
                     : drive_state == 1 ? F("driving") : F("no fix"));
      }
      if (have_anchor && last_dist_m >= 0) {
        Serial.print(F(" (")); Serial.print(last_dist_m); Serial.print(F(" m from anchor)"));
      }
      Serial.println();
    }

    // periodic presence timer (0 = park-only). First shot is one interval out.
    // Inside a quiet zone the presence is suppressed too — total radio silence,
    // nothing places the zone's location (or even the node) on the air. Driving
    // away clears in_zone and the presence cadence resumes.
    if (cfg.enabled && cfg.interval_mins > 0 && next_presence == 0) scheduleNextPresence();
    bool presence_due = cfg.enabled && cfg.interval_mins > 0 && next_presence != 0 &&
                        in_zone < 0 && (int32_t)(now - next_presence) >= 0;

    bool manual = pending_send;
    // Parked in a quiet zone: swallow the park event — no Meshtastic burst, no
    // MeshCore re-advert. An explicit 'carnode send' still transmits (operator
    // override); the repeat-off side is repeatSuppressed() above.
    if (park_event && in_zone >= 0 && !manual) { park_reported = true; park_event = false; }
    if (!manual && !park_event && !presence_due) return;
    if (busy) return;                                          // retune only when mesh idle

    // dedup: a park at (essentially) the same spot we last broadcast is skipped
    if (park_event && !manual) {
      bool new_spot = !have_advert || !fc.gps_valid ||
                      distMeters(advert_lat, advert_lon, fc.lat, fc.lon) > cfg.stop_radius_m;
      if (!new_spot) { park_reported = true; park_event = false; }
    }
    if (!manual && !park_event && !presence_due) return;

    if (cfg.enabled && textDue(now)) pending_text = true;      // arm the chat text if due

    // A new-spot park broadcast is always followed by a MeshCore flood
    // re-advert, so by the v0.2.3 rule (the text rides real flood adverts) the
    // text belongs in THIS burst. Arming it here costs no extra airtime -- the
    // retune is already happening -- and avoids scheduling a second Meshtastic
    // burst on top of the advert we are about to queue.
    if (park_event && cfg.enabled && cfg.text_mult > 0) pending_text = true;

    if (manual || park_event) {
      // Full location update:
      // 1) Meshtastic burst (median-filtered position from this fix)
      if (!sendBurst(driver, radio, fc)) {                     // channel busy (LBT)
        hold_until = millis() + 15000;                         // back off, stay pending
        return;
      }
      if (park_event) park_reported = true;
      pending_send = false;

      // 2) Request the MeshCore re-advert with the same fix -- both networks
      //    update together. (Skipped if we have no location to share.)
      if (fc.gps_valid) {
        readvert_lat = fc.lat; readvert_lon = fc.lon; readvert_pending = true;
        readvert_tries = 0; readvert_retry_at = 0;   // fresh fix supersedes any retry
        advert_lat = fc.lat; advert_lon = fc.lon; have_advert = true;
      }
    } else {
      // Light periodic presence between park events.
      if (!sendPresence(driver, radio, fc)) {                  // channel busy (LBT)
        hold_until = millis() + 15000;                         // back off, stay due
        return;
      }
    }
    scheduleNextPresence();   // any burst is fresh presence; restart the cadence
  }
};
