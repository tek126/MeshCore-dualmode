#pragma once

// MtBeaconControl — runtime-configurable Meshtastic beacon for a repeater.
//
// Wraps the stateless helpers in MeshtasticBeacon.h with persisted config and a
// MeshCore CLI verb ("mtbeacon ...") so the beacon can be turned on/off and
// tuned over the air or serial without recompiling. Header-only; designed to be
// dropped into simple_repeater behind -D WITH_MT_BEACON.
//
// Must be included AFTER the platform filesystem header (it uses FILESYSTEM /
// File), which MyMesh.h already pulls in.

#include <Arduino.h>
#include <stdlib.h>
#include <string.h>
#include <stddef.h>   // size_t
#include "MeshtasticBeacon.h"

// RadioLib's 1-byte private LoRa sync word (0x12) — what MeshCore uses on its
// LoRa chips; restored after each beacon. Override per board if needed.
#ifndef MESHCORE_SYNC_WORD
  #define MESHCORE_SYNC_WORD 0x12
#endif

#ifndef MT_BEACON_FILE
  #define MT_BEACON_FILE "/mtbeacon"
#endif

// Meshtastic HardwareModel reported in NodeInfo. Set per board via -D MT_HW_MODEL
// (e.g. 9=RAK4631, 43=Heltec V3, 69=T114, 71=T1000-E, 95=Seeed Solar). Defaults
// to 255 (PRIVATE_HW) for boards with no Meshtastic equivalent.
#ifndef MT_HW_MODEL
  #define MT_HW_MODEL 255
#endif

// Consecutive transmits the chip reports as never-sent before we re-initialise
// the radio (see meshtastic::radioRecover). One dead transmit is bad luck; a run
// of them is a wedged modem that will otherwise stay wedged until a power cycle.
#ifndef MT_BEACON_FAIL_STREAK
  #define MT_BEACON_FAIL_STREAK 3
#endif

class MtBeaconControl {
public:
  struct Config {
    uint32_t magic;
    uint8_t  enabled;
    uint8_t  region_idx;     // index into meshtastic::REGIONS
    uint8_t  preset_idx;     // index into meshtastic::PRESETS
    uint8_t  send_nodeinfo;  // emit NodeInfo (named node) as part of presence
    uint8_t  send_position;  // emit Position (map pin) as part of presence
    uint8_t  hop_limit;      // Meshtastic hop limit for presence + text (0-3, default 0)
    uint8_t  text_mult;      // chat text N times per flood-advert period (0=never; higher=more often)
    int8_t   tx_power;
    uint16_t interval_mins;  // PRESENCE cadence (silent NodeInfo+Position)
    float    freq_override;  // 0 = auto (derived from region + preset)
    char     text[64];
    // derived from region/preset (recomputed on every change; persisted too)
    float    freq;
    float    bw;
    uint8_t  sf;
    uint8_t  cr;
    uint8_t  sync_word;
    uint16_t preamble;
    // v6: Meshtastic short_name (map marker label), "" = auto ("MC" + 2 hex).
    // Fields are only ever APPENDED here, so the older prefix layout stays
    // byte-identical and load() can migrate an older file in place.
    char     short_name[5];
    // v7: emit Telemetry/DeviceMetrics (battery + uptime) with the presence.
    uint8_t  send_telemetry;
  };

  // Live per-send context the repeater supplies each tick (name/location/clock,
  // device health and the MeshCore PHY to restore). Keeps tick() tidy.
  struct Context {
    const char* node_name;
    double   lat, lon;
    uint32_t epoch;             // 0 if unknown (Position time is then omitted)
    uint16_t flood_advert_hours; // repeater's flood-advert interval (0 = off)
    uint16_t batt_millivolts;   // 0 if the board has no battery sense
    uint32_t uptime_secs;       // 0 if unknown
    float    home_freq, home_bw;
    uint8_t  home_sf, home_cr, home_sync;
    int8_t   home_tx_power;
  };

private:
  static const uint32_t MAGIC    = 0x3742544DUL;  // 'MTB7' (bumped: added send_telemetry)
  static const uint32_t MAGIC_V6 = 0x3642544DUL;  // 'MTB6' (+short_name; migrated on load)
  static const uint32_t MAGIC_V5 = 0x3542544DUL;  // 'MTB5' (+hop_limit; migrated on load)

  Config cfg;
  uint32_t node_num = 0;
  uint32_t packet_id = 1;
  uint8_t  chan_hash = 0x08;
  unsigned long next_due = 0;
  unsigned long duty_block_until = 0;   // earliest next TX allowed (duty cycle)
  uint8_t  presence_rot = 0;            // alternates NodeInfo/Position when rotating
  unsigned long last_text_ms = 0;      // when the chat text last went out
  uint16_t flood_hours_seen = 0;       // last-known flood-advert interval (for status)
  bool pending_text = false;            // include the chat text on the next burst
  bool pending_send = false;
  meshtastic::TxStats tx = {};          // transmit health ('mtbeacon stats')

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
    cfg.send_telemetry = 1;    // battery + uptime: small, and it fills the node's battery ring
    cfg.hop_limit = 0;         // 0 hops: heard by direct neighbors, never rebroadcast
    cfg.text_mult = 1;         // text once per flood advert (~rare, by design)
    cfg.freq_override = 0.0f;  // auto
    cfg.tx_power = 22;
    cfg.interval_mins = 30;    // presence cadence: frequent enough to stay live
    cfg.short_name[0] = 0;     // auto: "MC" + 2 hex of the node number
    strncpy(cfg.text, "MeshCore repeater in range", sizeof(cfg.text) - 1);
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
    cfg.interval_mins = constrain(cfg.interval_mins, 1, 1440);
    cfg.enabled = cfg.enabled ? 1 : 0;
    cfg.send_telemetry = cfg.send_telemetry ? 1 : 0;
    if (cfg.hop_limit > 3) cfg.hop_limit = 3;   // Meshtastic hop limit cap
    cfg.short_name[sizeof(cfg.short_name) - 1] = 0;
    cfg.text[sizeof(cfg.text) - 1] = 0;
    recompute();   // re-derive in case the preset/region tables changed
  }

  // Schedule the next beacon, with up to 20 s of random jitter so multiple
  // beacons (or beacon + a real node) don't lock-step onto the same airtime.
  void scheduleNext() {
    next_due = millis() + (unsigned long)cfg.interval_mins * 60000UL
             + (unsigned long)random(0, 20001);
  }

  // Detailed help to the serial console (the reply buffer is too small for it).
  void printHelp() {
    Serial.println(F("mtbeacon commands:"));
    Serial.println(F("  status             show current config"));
    Serial.println(F("  stats [clear]      transmit health since boot (or reset it)"));
    Serial.println(F("  on | off           enable / disable beaconing"));
    Serial.println(F("  send               transmit presence + text now"));
    Serial.println(F("  interval <min>     PRESENCE cadence (NodeInfo+Position), 1-1440"));
    Serial.println(F("  text.mult <N>      chat text N times per flood-advert period (0=never)"));
    Serial.println(F("  preset <name>      modem preset (LongFast, MediumFast, ...)"));
    Serial.println(F("  region <name>      region/country band (US, EU_868, ...)"));
    Serial.println(F("  freq <MHz|auto>    manual frequency override; auto = region+preset"));
    Serial.println(F("  power <dBm>        TX power, -9..22 (capped to region limit)"));
    Serial.println(F("  hops <0-3>         Meshtastic hop limit for presence + text (default 0)"));
    Serial.println(F("  text <string>      the chat-message content (<=63 chars)"));
    Serial.println(F("  short <str|auto>   Meshtastic short name / map label, <=4 chars"));
    Serial.println(F("  nodeinfo on|off    include NodeInfo (named node 'MC <name>')"));
    Serial.println(F("  position on|off    include Position (map pin) if location set"));
    Serial.println(F("  telemetry on|off   include Telemetry (battery + uptime)"));
    Serial.println(F("  presets / regions  list available values"));
    Serial.println(F("  help               this list"));
    Serial.println(F("PRESENCE (NodeInfo+Position) is silent and frequent -> keeps the map"));
    Serial.println(F("pin live. The TEXT is the only thing that shows in Meshtastic chat, so"));
    Serial.println(F("it's posted rarely, paced off the MeshCore flood advert: text.mult N"));
    Serial.println(F("= N posts per advert period (flood/N), e.g. 12h advert + 2x -> every 6h."));
    Serial.println(F("LBT + jitter + EU duty-cycle enforced; slow presets rotate to bound airtime."));
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
  // deadline, so LBT can never wedge the repeater.
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
      snprintf(ln, sizeof(ln), "MC %s", (c.node_name && *c.node_name) ? c.node_name : "Repeater");
      meshtastic::resolveShortName(sn, cfg.short_name, node_num);
      int n = meshtastic::buildUserPayload(pl, node_num, ln, sn, MT_HW_MODEL);
      return meshtastic::buildDataPacket(pkt, cap, node_num, nextId(),
                meshtastic::PORT_NODEINFO, pl, n, key, klen, chan_hash, cfg.hop_limit);
    } else if (k == 1) {
      if (!cfg.send_position || (c.lat == 0.0 && c.lon == 0.0)) return 0;
      int n = meshtastic::buildPositionPayload(pl, c.lat, c.lon, c.epoch);
      return meshtastic::buildDataPacket(pkt, cap, node_num, nextId(),
                meshtastic::PORT_POSITION, pl, n, key, klen, chan_hash, cfg.hop_limit);
    }
    return meshtastic::buildTextPacket(pkt, cap, node_num, nextId(),
                cfg.text, key, klen, chan_hash, cfg.hop_limit);
  }

  // One retune: emit the silent presence (NodeInfo + Position) plus the chat
  // text if it's due (pending_text), then restore the MeshCore PHY. Returns
  // false if the channel was busy (LBT) or nothing left the antenna -- either
  // way the caller retries.
  template <class D, class R>
  bool sendBurst(D& driver, R& radio, const Context& c) {
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

    // Presence = NodeInfo(0) + Position(1) + Telemetry(3), all silent. At slow
    // presets, send ONE of them per cycle (round-robin) to bound the off-channel
    // window. Only enabled kinds enter the rotation, so switching one off buys
    // a faster cycle through the rest instead of a wasted slot.
    // The chat text(2) is appended only when it's due (every Nth flood advert).
    //
    // The rotate decision is made HERE, after the retune, and not a line
    // earlier: getEstAirtimeFor() asks the radio for its current time-on-air,
    // so before the retune it would answer for the MeshCore PHY (typically much
    // faster) and never rotate, however slow the Meshtastic preset is.
    uint8_t pres[3]; uint8_t np = 0;
    if (cfg.send_nodeinfo)  pres[np++] = 0;
    if (cfg.send_position)  pres[np++] = 1;
    if (cfg.send_telemetry) pres[np++] = 3;

    uint8_t kinds[4]; int nk = 0;
    bool rotate = (driver.getEstAirtimeFor(60) * 2 + 120) > 2500;
    if (np > 0) {
      if (rotate) {
        if (presence_rot >= np) presence_rot = 0;
        kinds[nk++] = pres[presence_rot++];
      } else {
        for (uint8_t i = 0; i < np; i++) kinds[nk++] = pres[i];
      }
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
    // are now separate reports instead of one guess.
    if (irq_misses)
      Serial.printf("beacon: TxDone IRQ missed on %d/%d packet(s) - checked the chip instead\n",
                    irq_misses, tried);
    if (lost)
      Serial.printf("beacon: %d/%d packet(s) did NOT transmit (chip reports no TxDone)\n",
                    lost, tried);

    if (wedged) {
      // The chip has now reported several transmits in a row as never-sent.
      // Re-initialise instead of restoring, so the next beacon isn't dead too.
      tx.recoveries++;
      tx.fail_streak = 0;                              // give the re-init a clean run
      Serial.printf("beacon: %d dead transmits in a row - reinitialising the radio\n",
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
      duty_block_until = millis() + (unsigned long)air * (100 - duty) / duty;
    // Nothing BUILDABLE this cycle (e.g. position on but no location yet) is a
    // no-op, not a failure — reporting failure there would put the caller in a
    // 30-second retry loop forever. Only a burst that tried and got nothing onto
    // the air asks to be retried.
    if (built == 0) return true;
    return sent > 0;
  }

public:
  void load(FILESYSTEM* fs) {
#if defined(RP2040_PLATFORM)
    File f = fs->open(MT_BEACON_FILE, "r");
#else
    File f = fs->open(MT_BEACON_FILE);
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
      } else if ((magic == MAGIC_V5 || magic == MAGIC_V6) &&
                 n > 4 && n <= (int)sizeof(Config)) {
        // Fields are only ever appended, so an older file's prefix is exactly
        // this struct's prefix — copy what it has (bytes past `n` keep the
        // setDefaults() values), then force defaults for the fields that
        // version didn't have, since the copy can land in its trailing padding.
        // Every existing setting (preset, region, power, text, hops, ...) is kept.
        memcpy(&cfg, buf, n);
        cfg.magic = MAGIC;
        if (magic == MAGIC_V5) cfg.short_name[0] = 0;  // MTB5 had no short name -> auto
        cfg.send_telemetry = 1;                        // MTB6 and earlier had no telemetry knob
        sanitize();
      }
    }
  }

  void save(FILESYSTEM* fs) {
#if defined(NRF52_PLATFORM) || defined(STM32_PLATFORM)
    fs->remove(MT_BEACON_FILE);
    File f = fs->open(MT_BEACON_FILE, FILE_O_WRITE);
#elif defined(RP2040_PLATFORM)
    File f = fs->open(MT_BEACON_FILE, "w");
#else
    File f = fs->open(MT_BEACON_FILE, "w", true);
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
    scheduleNext();
  }

  bool enabled() const { return cfg.enabled; }

  // The repeater just flooded a MeshCore advert: have the chat text ride a
  // beacon burst shortly after, so the Meshtastic text follows the repeater's
  // actual advert cadence instead of a boot-reset stopwatch (which, on nodes
  // that reboot or hang before a full 47h period elapses, never fires at all).
  // textDue() still paces extra texts between adverts when text_mult > 1.
  void onFloodAdvert() {
    if (!cfg.enabled || cfg.text_mult == 0) return;
    pending_text = true;
    unsigned long soon = millis() + 15000;      // let the advert TX clear the air first
    if ((int32_t)(next_due - soon) > 0) next_due = soon;
  }

  // Compact one-line status for the repeater's home screen, e.g.
  // "Beacon ON LongFast".
  void uiLine(char* out, size_t n) const {
    if (cfg.enabled) snprintf(out, n, "Beacon ON %s", meshtastic::PRESETS[cfg.preset_idx].name);
    else             snprintf(out, n, "Beacon off");
  }

  void status(char* reply) {
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
    char sn[5];
    meshtastic::resolveShortName(sn, cfg.short_name, node_num);
    // single bounded write -> shows the full text, never overflows reply[160]
    // "p<N>m" = presence cadence; "txt<N>x" = text N times per flood-advert period
    // "!<id>/<short>" = Meshtastic node id and the short name / map label
    snprintf(reply, 160,
             "beacon %s %sMHz%s %s %s(SF%d BW%d) p%dm %ddBm%s h%d %s%s%s %s !%08lx/%s \"%s\"",
             cfg.enabled ? "ON" : "off", fbuf, cfg.freq_override > 0.0f ? "*" : "",
             r.name, p.name, (int)cfg.sf, (int)cfg.bw, (int)cfg.interval_mins,
             (int)ep, ep < cfg.tx_power ? "(cap)" : "", (int)cfg.hop_limit,
             cfg.send_nodeinfo ? "+info" : "", cfg.send_position ? "+pos" : "",
             cfg.send_telemetry ? "+tel" : "",
             txt, (unsigned long)node_num, sn, cfg.text);
  }

  // `mtbeacon stats` — transmit health since boot. RAM only; see TxStats.
  void statsReply(char* reply) {
    meshtastic::formatTxStats(reply, 160, tx, "beacon",
                              (uint32_t)(millis() - tx.last_fail_ms));
  }

  // Returns false if `command` is not an "mtbeacon" verb (caller falls through).
  bool handleCommand(char* command, char* reply, FILESYSTEM* fs) {
    if (memcmp(command, "mtbeacon", 8) != 0) return false;
    const char* a = command + 8;
    while (*a == ' ') a++;

    if (*a == 0 || strcmp(a, "status") == 0) {
      status(reply);
    } else if (strcmp(a, "stats") == 0) {
      statsReply(reply);
    } else if (strcmp(a, "stats clear") == 0) {
      tx = meshtastic::TxStats();
      strcpy(reply, "OK - tx stats cleared");
    } else if (strcmp(a, "help") == 0 || strcmp(a, "?") == 0) {
      printHelp();
      strcpy(reply, "cmds: status stats on off send | interval text.mult text short freq power hops preset region nodeinfo position telemetry | presets regions help");
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
      cfg.enabled = 1; scheduleNext(); save(fs); strcpy(reply, "OK - beacon on");
    } else if (strcmp(a, "off") == 0) {
      cfg.enabled = 0; save(fs); strcpy(reply, "OK - beacon off");
    } else if (strcmp(a, "send") == 0) {
      pending_send = true; pending_text = true; next_due = millis();
      strcpy(reply, "OK - sending shortly");
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
    } else if (memcmp(a, "interval ", 9) == 0) {
      int m = atoi(a + 9);
      if (m < 1 || m > 1440) { strcpy(reply, "Error: interval 1-1440 min"); }
      else { cfg.interval_mins = m; scheduleNext(); save(fs); sprintf(reply, "OK - presence every %d min", m); }
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

  // Call every loop. `busy` should be true when the mesh has queued/in-flight
  // work, so the beacon never retunes mid-transaction (it defers a few seconds).
  template <class D, class R>
  void tick(D& driver, R& radio, bool busy, const Context& c) {
    flood_hours_seen = c.flood_advert_hours;                   // keep current for textDue/status
    if (!cfg.enabled && !pending_send) return;
    if ((int32_t)(millis() - next_due) < 0) return;            // not due yet
    if ((int32_t)(millis() - duty_block_until) < 0) {          // duty-cycle hold
      next_due = duty_block_until; return;
    }
    if (busy) { next_due = millis() + 3000; return; }          // defer past mesh activity
    if (cfg.enabled && textDue(millis())) pending_text = true; // arm the chat text if due
    if (!sendBurst(driver, radio, c)) {                        // channel busy (LBT)
      next_due = millis() + 30000;                             // retry soon, stay pending
      return;
    }
    pending_send = false;
    scheduleNext();
  }
};
