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
//   * Optional "home": `carnode home` stores the current spot. Parking within
//     `home radius` of it turns repeating off immediately (same switch as the
//     park sleep) and skips the park broadcast + periodic presence, so the
//     home location never goes on the air. Driving away restores everything.
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

class CarNodeControl {
public:
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
    // v6: "home" — parking within home_radius_m of this spot turns repeat off
    // immediately and suppresses all broadcasts (new fields appended only, so
    // the v5->v6 load migration can keep everything above).
    uint8_t  have_home;
    uint16_t home_radius_m;
    double   home_lat, home_lon;
  };

  // Live per-tick context the repeater supplies. lat/lon are the CURRENT GPS fix
  // and gps_valid says whether they're usable.
  struct Context {
    const char* node_name;
    double   lat, lon;
    bool     gps_valid;
    uint32_t epoch;             // 0 if unknown (Position time is then omitted)
    uint16_t flood_advert_hours; // repeater's flood-advert interval (0 = off)
    float    home_freq, home_bw;
    uint8_t  home_sf, home_cr, home_sync;
    int8_t   home_tx_power;
  };

private:
  static const uint32_t MAGIC    = 0x364E5241UL;  // 'ARN6' — car-node config v6 (+home)
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
  uint8_t  presence_rot = 0;           // alternates NodeInfo/Position when rotating

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
  bool     at_home = false;                 // parked within home radius (repeat forced off, radio quiet)
  double   cur_lat = 0, cur_lon = 0;        // last filtered fix (what 'carnode home' captures)
  bool     cur_fix_valid = false;

  // --- MeshCore re-advert handoff + dedup ---
  double   advert_lat = 0, advert_lon = 0;  // last location we pushed to both networks
  bool     have_advert = false;
  bool     readvert_pending = false;        // a MeshCore re-advert is waiting for MyMesh
  double   readvert_lat = 0, readvert_lon = 0;

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
    cfg.text_mult = 1;         // text once per flood advert (rare, by design)
    cfg.freq_override = 0.0f;  // auto
    cfg.tx_power = 22;         // vehicle-powered: favour visibility (region-capped)
    cfg.park_secs = 300;       // stopped for 5 min -> push a location update
    cfg.stop_radius_m = 30;    // GPS jitter / small repositioning still counts as parked
    cfg.advert_delay_s = 10;   // MeshCore advert fires 10 s after the Meshtastic burst
    cfg.sleep_hours = 20;      // parked ~a day -> nobody's around; stop repeating
    cfg.interval_mins = 30;    // presence cadence between parks (like base mtbeacon)
    cfg.have_home = 0;         // no home until 'carnode home' captures one
    cfg.home_radius_m = 100;   // generous: home is "the driveway", not a parking spot
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
    cfg.text[sizeof(cfg.text) - 1] = 0;
    recompute();   // re-derive in case the preset/region tables changed
  }

  // Detailed help to the serial console (the reply buffer is too small for it).
  // The command surface is split: `mtbeacon` tunes the Meshtastic beacon itself,
  // `carnode` tunes the car-specific park behaviour.
  void printBeaconHelp() {
    Serial.println(F("mtbeacon commands (the Meshtastic beacon):"));
    Serial.println(F("  status             show beacon RF config"));
    Serial.println(F("  on | off           enable / disable beaconing"));
    Serial.println(F("  send               push a location update now (both networks)"));
    Serial.println(F("  interval <min>     periodic presence between parks, 0-1440 (0=park-only)"));
    Serial.println(F("  preset <name>      modem preset (LongFast, MediumFast, ...)"));
    Serial.println(F("  region <name>      region/country band (US, EU_868, ...)"));
    Serial.println(F("  freq <MHz|auto>    manual frequency override; auto = region+preset"));
    Serial.println(F("  power <dBm>        TX power, -9..22 (capped to region limit)"));
    Serial.println(F("  text <string>      the chat-message content (<=63 chars)"));
    Serial.println(F("  text.mult <N>      chat text N times per flood-advert period (0=never)"));
    Serial.println(F("  nodeinfo on|off    include NodeInfo (named node 'MC <name>')"));
    Serial.println(F("  position on|off    include Position (map pin) from live GPS"));
    Serial.println(F("  presets / regions  list available values"));
    Serial.println(F("Car-specific timing lives under 'carnode' (park / radius)."));
  }

  void printCarHelp() {
    Serial.println(F("carnode commands (mobile/park behaviour):"));
    Serial.println(F("  status             show drive state + park config"));
    Serial.println(F("  on | off | send    enable/disable/push now (also under 'mtbeacon')"));
    Serial.println(F("  park <sec>         stopped time before an update fires, 30-86400"));
    Serial.println(F("  radius <m>         movement within this counts as stopped, 5-2000"));
    Serial.println(F("  advertdelay <sec>  gap: Meshtastic burst -> MeshCore advert, 0-600"));
    Serial.println(F("  sleep <hours>      parked this long -> stop repeating until driving, 0=never"));
    Serial.println(F("  home               set home = here; parking at home -> repeat off, radio quiet"));
    Serial.println(F("  home clear         forget the home location"));
    Serial.println(F("  home radius <m>    how close to home counts, 5-2000 (default 100)"));
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

  // Build packet kind k (0=NodeInfo, 1=Position, 2=Text) into pkt[].
  // Returns wire length, or 0 if that kind is disabled / unavailable.
  int buildKind(uint8_t k, uint8_t* pkt, size_t cap, const Context& c) {
    uint8_t pl[240];
    const uint8_t* key = meshtastic::DEFAULT_KEY;
    const size_t klen = sizeof(meshtastic::DEFAULT_KEY);
    if (k == 0) {
      if (!cfg.send_nodeinfo) return 0;
      char ln[44], sn[5];
      snprintf(ln, sizeof(ln), "MC %s", (c.node_name && *c.node_name) ? c.node_name : "Mobile");
      snprintf(sn, sizeof(sn), "%04lx", (unsigned long)(node_num & 0xFFFF));
      int n = meshtastic::buildUserPayload(pl, node_num, ln, sn, MT_HW_MODEL);
      return meshtastic::buildDataPacket(pkt, cap, node_num, nextId(),
                meshtastic::PORT_NODEINFO, pl, n, key, klen, chan_hash);
    } else if (k == 1) {
      if (!cfg.send_position || !c.gps_valid) return 0;
      int n = meshtastic::buildPositionPayload(pl, c.lat, c.lon, c.epoch);
      return meshtastic::buildDataPacket(pkt, cap, node_num, nextId(),
                meshtastic::PORT_POSITION, pl, n, key, klen, chan_hash);
    }
    return meshtastic::buildTextPacket(pkt, cap, node_num, nextId(),
                cfg.text, key, klen, chan_hash);
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
  bool sendKinds(D& driver, R& radio, const Context& c, const uint8_t* kinds, int nk) {
    meshtastic::ModemPreset mt = { cfg.freq, cfg.bw, cfg.sf, cfg.cr, cfg.preamble, cfg.sync_word };
    meshtastic::radioEnterMeshtastic(driver, radio, mt, effectivePower());

    if (!channelClear(radio)) {                       // listen-before-talk
      meshtastic::radioRestoreMeshCore(driver, radio, c.home_freq, c.home_bw,
                  c.home_sf, c.home_cr, c.home_sync, c.home_tx_power);
      return false;
    }

    uint8_t pkt[256];
    uint32_t air = 0;
    bool first = true;
    int sent = 0, irq_misses = 0;
    for (int i = 0; i < nk; i++) {
      int len = buildKind(kinds[i], pkt, sizeof(pkt), c);
      if (len <= 0) continue;                          // disabled / unavailable
      if (!first) delay(120);                          // inter-packet gap
      bool miss = false;
      if (!meshtastic::radioSendBlocking(driver, pkt, len, &miss)) break;  // radio wouldn't start: abort burst
      sent++; if (miss) irq_misses++;
      air += driver.getEstAirtimeFor(len);
      first = false;
      if (kinds[i] == 2) { pending_text = false; last_text_ms = millis(); }  // text delivered
    }
    // Diagnostic: a missed TxDone interrupt means the packet still went out (via
    // the airtime fallback) but the radio ISR isn't firing after the retune.
    if (irq_misses)
      Serial.printf("carnode: TxDone IRQ missed on %d/%d packet(s) - used airtime fallback\n",
                    irq_misses, sent);

    meshtastic::radioRestoreMeshCore(driver, radio, c.home_freq, c.home_bw,
                c.home_sf, c.home_cr, c.home_sync, c.home_tx_power);

    // duty cycle: hold off next TX by on-time*(100-duty)/duty for limited regions
    uint8_t duty = meshtastic::REGIONS[cfg.region_idx].duty_pct;
    if (duty > 0 && duty < 100)
      hold_until = millis() + (unsigned long)air * (100 - duty) / duty;
    return sent > 0;   // nothing left the antenna -> let the caller retry
  }

  // Park/manual burst — the full location update.
  // A park is infrequent and the whole point is the location, so ALWAYS send
  // both Position(1) and NodeInfo(0). Position is sent TWICE (once first, once
  // last) for redundancy against a missed broadcast on the busy public LongFast
  // channel — the two copies are separated by the NodeInfo so a single collision
  // is unlikely to take out both, and each gets its own packet id (not deduped).
  // (The mtbeacon rotate/alternate trick, which sent only one presence packet
  // per burst to save airtime, could update the node but not its location on a
  // park — wrong for a car node.) Text(2) is appended only when due.
  template <class D, class R>
  bool sendBurst(D& driver, R& radio, const Context& c) {
    uint8_t kinds[4]; int nk = 0;
    kinds[nk++] = 1;   // Position — the important one, sent first
    kinds[nk++] = 0;   // NodeInfo — names the node
    kinds[nk++] = 1;   // Position again — redundancy
    if (pending_text) kinds[nk++] = 2;
    return sendKinds(driver, radio, c, kinds, nk);
  }

  // Periodic light presence (base-mtbeacon style) — keeps the node from aging
  // out of Meshtastic node lists between park events. Position is included only
  // while PARKED (it refreshes the pin at the parked spot); while driving or
  // without a fix it's NodeInfo only, so the map pin never wanders off to some
  // random mid-drive point and sticks there. At slow presets one packet per
  // cycle (alternating) bounds the off-channel window, like the base beacon.
  template <class D, class R>
  bool sendPresence(D& driver, R& radio, const Context& c) {
    bool pos_ok = cfg.send_position && c.gps_valid && drive_state == 2;
    if (!cfg.send_nodeinfo && !pos_ok && !pending_text)
      return true;   // nothing sendable this cycle: no-op, keep the cadence
    uint8_t kinds[3]; int nk = 0;
    bool rotate = (driver.getEstAirtimeFor(60) * 2 + 120) > 2500;
    if (!pos_ok)     kinds[nk++] = 0;                                    // NodeInfo only
    else if (rotate) { kinds[nk++] = presence_rot; presence_rot ^= 1; }  // alternate
    else             { kinds[nk++] = 0; kinds[nk++] = 1; }               // both
    if (pending_text) kinds[nk++] = 2;
    return sendKinds(driver, radio, c, kinds, nk);
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
      } else if (magic == MAGIC_V5 && n > 4 && n <= (int)sizeof(Config)) {
        // v5 -> v6: the home fields were appended, so the v5 prefix layout is
        // unchanged — copy what the file has, then force defaults for the new
        // fields (the copy may have clobbered ones that landed in old padding).
        memcpy(&cfg, buf, n);
        cfg.magic = MAGIC;
        cfg.have_home = 0;
        cfg.home_radius_m = 100;
        cfg.home_lat = cfg.home_lon = 0;
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
  // extra texts between adverts when text_mult > 1. At home nothing changes:
  // presence stays suppressed, the text just stays pending until driving.
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
  // NodePrefs and floods a fresh advert.
  bool takeReadvert(double& lat, double& lon) {
    if (!readvert_pending) return false;
    readvert_pending = false;
    lat = readvert_lat;
    lon = readvert_lon;
    return true;
  }

  // How long the repeater should delay the MeshCore advert after the Meshtastic
  // burst (ms), so the two park transmissions don't land on top of each other.
  uint32_t advertDelayMs() const { return (uint32_t)cfg.advert_delay_s * 1000UL; }

  // True while the repeater should stop forwarding, for either reason:
  //   * parked at HOME (`carnode home`) — repeat goes off as soon as the park
  //     is detected, no waiting; home presumably has fixed coverage already; or
  //   * parked anywhere past `carnode sleep <hours>` — a car parked for a day
  //     is probably somewhere nobody needs a mobile repeater.
  // The repeater polls this each loop and toggles its actual 'repeat' pref to
  // match (the same switch as 'set repeat on|off'), so the state is visible
  // everywhere. Keyed off the park anchor (not drive_state), so losing the GPS
  // fix in a garage does NOT wake the repeater; only actually moving does.
  // Clears itself as soon as driving resumes (the anchor follows the vehicle,
  // the clock restarts, at_home drops). Runtime-only: nothing is persisted
  // here, so a reboot starts awake.
  bool repeatSuppressed() const {
    if (!cfg.enabled || !have_anchor) return false;
    if (at_home) return true;
    if (cfg.sleep_hours == 0) return false;
    return (unsigned long)(millis() - stationary_since) >=
           (unsigned long)cfg.sleep_hours * 3600000UL;
  }

  // Compact one-line status for the repeater's home screen.
  void uiLine(char* out, size_t n) const {
    if (!cfg.enabled) { snprintf(out, n, "CarNode off"); return; }
    const char* s = at_home ? "home" : repeatSuppressed() ? "sleeping"
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
    snprintf(reply, 160,
             "mtbeacon %s %sMHz%s %s %s(SF%d BW%d) %s %ddBm%s %s%s %s !%08lx",
             cfg.enabled ? "ON" : "off", fbuf, cfg.freq_override > 0.0f ? "*" : "",
             r.name, p.name, (int)cfg.sf, (int)cfg.bw, ivl,
             (int)ep, ep < cfg.tx_power ? "(cap)" : "",
             cfg.send_nodeinfo ? "+info" : "", cfg.send_position ? "+pos" : "",
             txt, (unsigned long)node_num);
  }

  // `carnode status` — the car-specific park behaviour + drive state.
  // The [..] block includes live tracking: distance of the current fix from the
  // park anchor and how long the stationary clock has been running — so GPS
  // jitter (fix wandering past `radius`, which restarts the clock) is visible.
  void carStatus(char* reply) {
    const char* st = at_home ? "home" : repeatSuppressed() ? "sleeping"
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
    if (!cfg.have_home) strcpy(hm, "hm:off");
    else snprintf(hm, sizeof(hm), "hm%dm", (int)cfg.home_radius_m);
    snprintf(reply, 160,
             "carnode %s [%s%s] park%ds r%dm adv%ds %s %s loc:%s !%08lx",
             cfg.enabled ? "ON" : "off", st, trk,
             (int)cfg.park_secs, (int)cfg.stop_radius_m, (int)cfg.advert_delay_s,
             slp, hm, have_advert ? "set" : "none", (unsigned long)node_num);
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
    } else if (strcmp(a, "help") == 0 || strcmp(a, "?") == 0) {
      printBeaconHelp();
      strcpy(reply, "mtbeacon: status on off send | interval preset region freq power text text.mult nodeinfo position | presets regions");
    } else if (memcmp(a, "nodeinfo ", 9) == 0) {
      cfg.send_nodeinfo = (strcasecmp(a + 9, "on") == 0) ? 1 : 0; save(fs);
      sprintf(reply, "OK - nodeinfo %s", cfg.send_nodeinfo ? "on" : "off");
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
    } else if (memcmp(a, "text.mult ", 10) == 0) {
      int n = atoi(a + 10);
      if (n < 0 || n > 255) { strcpy(reply, "Error: 0-255 (0 = never post text)"); }
      else { cfg.text_mult = n; save(fs);
             if (n == 0) strcpy(reply, "OK - text off (silent presence only)");
             else sprintf(reply, "OK - text %dx per flood advert", n); }
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
      strcpy(reply, "carnode: status on off send | park <sec> | radius <m> | advertdelay <sec> | sleep <hours> | home [clear|radius <m>]");
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
    } else if (memcmp(a, "home", 4) == 0 && (a[4] == 0 || a[4] == ' ')) {
      const char* h = a + 4;
      while (*h == ' ') h++;
      if (*h == 0) {   // "carnode home" — capture the current (filtered) fix as home
        if (!cur_fix_valid) { strcpy(reply, "Error: no GPS fix - can't set home"); }
        else if (!fix_filtered) { strcpy(reply, "Error: GPS filter warming up - retry in ~15 sec"); }
        else {
          cfg.have_home = 1; cfg.home_lat = cur_lat; cfg.home_lon = cur_lon; save(fs);
          sprintf(reply, "OK - home set: parking within %d m turns repeat off",
                  (int)cfg.home_radius_m);
        }
      } else if (strcmp(h, "clear") == 0) {
        cfg.have_home = 0; save(fs); strcpy(reply, "OK - home cleared");
      } else if (memcmp(h, "radius ", 7) == 0) {
        int m = atoi(h + 7);
        if (m < 5 || m > 2000) { strcpy(reply, "Error: home radius 5-2000 m"); }
        else { cfg.home_radius_m = m; save(fs); sprintf(reply, "OK - home = within %d m", m); }
      } else {
        strcpy(reply, "Unknown - carnode home [clear | radius <m>]");
      }
    } else {
      strcpy(reply, "Unknown - try 'carnode help'");
    }
    return true;
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
    // and `carnode home` can capture the current spot even while disabled.
    Context fc = c;
    fix_filtered = c.gps_valid && filterFix(now, c.lat, c.lon, fc.lat, fc.lon);
    cur_fix_valid = fc.gps_valid;
    if (fc.gps_valid) { cur_lat = fc.lat; cur_lon = fc.lon; }

    // Log repeat-sleep transitions (the repeater polls repeatSuppressed() each
    // loop and toggles its repeat pref; this is just serial-console visibility).
    bool slp = repeatSuppressed();
    if (slp != sleep_announced) {
      sleep_announced = slp;
      Serial.println(slp ? (at_home ? F("carnode: parked at home - repeat OFF until driving")
                                    : F("carnode: parked past sleep limit - repeat OFF until driving"))
                         : F("carnode: repeat back ON"));
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
        outside_since = 0; last_dist_m = 0; at_home = false;
      } else {
        double d = distMeters(anchor_lat, anchor_lon, fc.lat, fc.lon);
        last_dist_m = (int)(d + 0.5);
        if (d > cfg.stop_radius_m) {
          if (outside_since == 0) outside_since = now;
          if ((unsigned long)(now - outside_since) >= CAR_NODE_MOVE_CONFIRM_MS) {
            anchor_lat = fc.lat; anchor_lon = fc.lon;          // moving: follow the vehicle
            stationary_since = now; park_reported = false; drive_state = 1;
            outside_since = 0; at_home = false;
          }
          // else: not confirmed yet — hold state, keep the stationary clock
        } else {
          outside_since = 0;                                   // sitting near the anchor
          if ((now - stationary_since) >= (unsigned long)cfg.park_secs * 1000UL) {
            drive_state = 2;                                   // parked
            if (!park_reported) park_event = true;
          }
          // Parked at home? Anchor vs stored home — both stable, so no flapping.
          // Re-evaluated every tick so setting/clearing home while already
          // parked takes effect immediately. NOT cleared on fix loss (garage):
          // like the sleep, only actually driving away wakes the repeater.
          at_home = cfg.have_home && drive_state == 2 &&
                    distMeters(anchor_lat, anchor_lon, cfg.home_lat, cfg.home_lon)
                      <= (double)cfg.home_radius_m;
        }
      }
    } else if (cfg.enabled) {
      drive_state = 0;   // no current GPS fix -> report "nofix" live (not latched)
    }

    // Log drive-state transitions to the serial console (diagnostics).
    if (cfg.enabled && drive_state != state_announced) {
      state_announced = drive_state;
      Serial.print(F("carnode: "));
      Serial.print(drive_state == 2 ? (at_home ? F("parked at home") : F("parked"))
                   : drive_state == 1 ? F("driving") : F("no fix"));
      if (have_anchor && last_dist_m >= 0) {
        Serial.print(F(" (")); Serial.print(last_dist_m); Serial.print(F(" m from anchor)"));
      }
      Serial.println();
    }

    // periodic presence timer (0 = park-only). First shot is one interval out.
    // At home the presence is suppressed too — total radio silence, nothing
    // places the home location (or even the node) on the air. Driving away
    // clears at_home and the presence cadence resumes.
    if (cfg.enabled && cfg.interval_mins > 0 && next_presence == 0) scheduleNextPresence();
    bool presence_due = cfg.enabled && cfg.interval_mins > 0 && next_presence != 0 &&
                        !at_home && (int32_t)(now - next_presence) >= 0;

    bool manual = pending_send;
    // Parked at home: swallow the park event — no Meshtastic burst, no MeshCore
    // re-advert. An explicit 'carnode send' still transmits (operator override);
    // the repeat-off side is repeatSuppressed() above.
    if (park_event && at_home && !manual) { park_reported = true; park_event = false; }
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
