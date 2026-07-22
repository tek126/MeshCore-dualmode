#pragma once

// MeshtasticProto.h — the pure, dependency-free half of the beacon:
// channel hashing, the modem-preset / region tables, the Meshtastic
// default-channel frequency calculation, and the protobuf payload encoders.
//
// Deliberately depends only on the C standard library (no Arduino, no Crypto),
// so it can be unit-tested on a host (see test/). MeshtasticBeacon.h includes
// this and adds the AES-CTR packet assembly + radio control.

#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <strings.h>  // strcasecmp
#include <stdio.h>    // snprintf

namespace meshtastic {

// Well-known PUBLIC channel ("LongFast") AES key: the 16-byte expansion of the
// default key index 1 ("AQ=="). 16 bytes -> AES-128.
static const uint8_t DEFAULT_KEY[16] = {
  0xd4, 0xf1, 0xbb, 0x3a, 0x20, 0x29, 0x07, 0x59,
  0xf0, 0xbc, 0xff, 0xab, 0xcf, 0x4e, 0x69, 0x01
};
static const char DEFAULT_CHANNEL_NAME[] = "LongFast";

// Meshtastic channel-hash byte = XOR of name bytes ^ XOR of key bytes.
// channelHash(DEFAULT_CHANNEL_NAME, DEFAULT_KEY, 16) == 0x08.
inline uint8_t xorHash(const uint8_t* p, size_t n) {
  uint8_t h = 0;
  for (size_t i = 0; i < n; i++) h ^= p[i];
  return h;
}
inline uint8_t channelHash(const char* name, const uint8_t* key, size_t key_len) {
  return xorHash((const uint8_t*)name, strlen(name)) ^ xorHash(key, key_len);
}

// ---------------------------------------------------------------------------
// Modem presets and region bands. TX frequency for the default public channel
// is computed exactly the way Meshtastic does it:
//   numChannels = floor((freqEnd - freqStart) / bw)   (>= 1)
//   slot        = djb2(presetName) % numChannels
//   freq        = freqStart + bw/2 + slot * bw
// Verified: US LongFast = 906.875 MHz, EU_868 LongFast = 869.525 MHz.
// ---------------------------------------------------------------------------
struct Preset { const char* name; float bw_khz; uint8_t sf; uint8_t cr; };
static const Preset PRESETS[] = {
  {"LongFast",   250.0f, 11, 5},
  {"LongMod",    125.0f, 11, 8},
  {"LongSlow",   125.0f, 12, 8},
  {"MediumFast", 250.0f,  9, 5},
  {"MediumSlow", 250.0f, 10, 5},
  {"ShortFast",  250.0f,  7, 5},
  {"ShortSlow",  250.0f,  8, 5},
  {"ShortTurbo", 500.0f,  7, 5},
};
static const uint8_t NUM_PRESETS = sizeof(PRESETS) / sizeof(PRESETS[0]);

// max_power_dbm = region radio-output power cap (the SX1262/LR1110 also caps 22).
// duty_pct      = legal transmit duty-cycle ceiling (100 = unenforced here).
struct Region { const char* name; float freq_start; float freq_end; int8_t max_power_dbm; uint8_t duty_pct; };
static const Region REGIONS[] = {
  {"US",     902.0f, 928.0f,  30, 100},
  {"EU_868", 869.4f, 869.65f, 27,  10},
  {"EU_433", 433.0f, 434.0f,  12,  10},
  {"ANZ",    915.0f, 928.0f,  30, 100},
  {"CN",     470.0f, 510.0f,  19, 100},
  {"JP",     920.8f, 927.8f,  16, 100},
  {"KR",     920.0f, 923.0f,  22, 100},
  {"TW",     920.0f, 925.0f,  27, 100},
  {"RU",     868.7f, 869.2f,  20, 100},
  {"IN",     865.0f, 867.0f,  30, 100},
  {"NZ_865", 864.0f, 868.0f,  30, 100},
  {"TH",     920.0f, 925.0f,  16, 100},
  {"UA_433", 433.0f, 434.7f,  10,  10},
  {"UA_868", 868.0f, 868.6f,  27,  10},
};
static const uint8_t NUM_REGIONS = sizeof(REGIONS) / sizeof(REGIONS[0]);

inline uint32_t djb2(const char* s) {
  uint32_t h = 5381;
  while (*s) h = (h * 33u) + (uint8_t)*s++;
  return h;
}

inline float presetFreq(const Region& r, const Preset& p) {
  float bw = p.bw_khz / 1000.0f;                     // MHz
  int numch = (int)((r.freq_end - r.freq_start) / bw);
  if (numch < 1) numch = 1;                          // guard narrow bands (no %0)
  uint32_t slot = djb2(p.name) % (uint32_t)numch;
  return r.freq_start + bw / 2.0f + (float)slot * bw;
}

inline int findPreset(const char* name) {            // -1 if not found
  for (uint8_t i = 0; i < NUM_PRESETS; i++)
    if (strcasecmp(name, PRESETS[i].name) == 0) return i;
  return -1;
}
inline int findRegion(const char* name) {
  for (uint8_t i = 0; i < NUM_REGIONS; i++)
    if (strcasecmp(name, REGIONS[i].name) == 0) return i;
  return -1;
}

// --- minimal protobuf writers ---
enum PortNum { PORT_TEXT = 1, PORT_POSITION = 3, PORT_NODEINFO = 4,
               PORT_TELEMETRY = 67 };

inline int pbVarint(uint8_t* o, uint64_t v) {
  int n = 0;
  while (v > 0x7F) { o[n++] = (uint8_t)(v & 0x7F) | 0x80; v >>= 7; }
  o[n++] = (uint8_t)v;
  return n;
}
inline int pbKey(uint8_t* o, uint32_t field, uint32_t wiretype) {
  return pbVarint(o, ((uint64_t)field << 3) | wiretype);
}
inline int pbString(uint8_t* o, uint32_t field, const char* s) {
  size_t len = strlen(s);
  int n = pbKey(o, field, 2);
  n += pbVarint(o + n, len);
  memcpy(o + n, s, len); n += len;
  return n;
}
inline int pbSfixed32(uint8_t* o, uint32_t field, int32_t v) {
  int n = pbKey(o, field, 5);          // wire type 5 = 32-bit
  memcpy(o + n, &v, 4); n += 4;        // little-endian (ARM/x86 native)
  return n;
}
inline int pbFixed32(uint8_t* o, uint32_t field, uint32_t v) {
  int n = pbKey(o, field, 5);          // wire type 5 = 32-bit
  memcpy(o + n, &v, 4); n += 4;
  return n;
}
inline int pbUint32(uint8_t* o, uint32_t field, uint32_t v) {
  int n = pbKey(o, field, 0);          // wire type 0 = varint
  n += pbVarint(o + n, v);
  return n;
}
inline int pbFloat(uint8_t* o, uint32_t field, float v) {
  int n = pbKey(o, field, 5);          // wire type 5 = 32-bit
  memcpy(o + n, &v, 4); n += 4;        // little-endian IEEE-754 (ARM/x86 native)
  return n;
}

// Meshtastic HardwareModel enum: 255 = PRIVATE_HW (honest for a non-Meshtastic
// device — fills the "hardware" field instead of leaving it blank).
static const uint8_t HW_PRIVATE = 255;

// Meshtastic User message (NodeInfo payload): id, long_name, short_name, hw_model.
inline int buildUserPayload(uint8_t* out, uint32_t node_num,
                            const char* long_name, const char* short_name,
                            uint16_t hw_model = HW_PRIVATE) {
  char id[12];
  snprintf(id, sizeof(id), "!%08lx", (unsigned long)node_num);
  int n = 0;
  n += pbString(out + n, 1, id);            // id
  n += pbString(out + n, 2, long_name);     // long_name
  n += pbString(out + n, 3, short_name);    // short_name
  n += pbUint32(out + n, 5, hw_model);      // hw_model (enum, varint)
  return n;
}

// Default Meshtastic short_name for a beacon: "MC" + the low byte of the node
// number in hex, e.g. "MC7a". short_name is capped at 4 characters and is what
// Meshtastic renders as the MAP MARKER LABEL, so a constant "MC" would make
// every beacon an indistinguishable pin; the two hex digits keep 256-way local
// uniqueness while still reading as MeshCore at a glance. The long name
// ("MC <node name>") disambiguates on tap.
inline void defaultShortName(char out[5], uint32_t node_num) {
  snprintf(out, 5, "MC%02x", (unsigned)(node_num & 0xFFu));
}

// Copy an operator-set short name into out[5], truncated to Meshtastic's 4-char
// limit; falls back to defaultShortName() when unset (empty string = "auto").
inline void resolveShortName(char out[5], const char* configured, uint32_t node_num) {
  if (configured && configured[0]) {
    strncpy(out, configured, 4);
    out[4] = 0;
  } else {
    defaultShortName(out, node_num);
  }
}

// Meshtastic Position message: latitude_i, longitude_i (1e-7 deg), time, precision.
// NOTE: Position.time is fixed32 (wire type 5) in mesh.proto, NOT a varint — a
// varint here makes nanopb reject the whole message, so the pin never shows.
inline int buildPositionPayload(uint8_t* out, double lat, double lon, uint32_t time_s) {
  int32_t lat_i = (int32_t)(lat * 1e7 + (lat < 0 ? -0.5 : 0.5));
  int32_t lon_i = (int32_t)(lon * 1e7 + (lon < 0 ? -0.5 : 0.5));
  int n = 0;
  n += pbSfixed32(out + n, 1, lat_i);     // latitude_i  (sfixed32)
  n += pbSfixed32(out + n, 2, lon_i);     // longitude_i (sfixed32)
  // Only stamp time if the clock is plausibly real (> 2020-09); otherwise omit
  // it and let the receiver use its RX time, rather than claim a 1970 fix.
  if (time_s > 1600000000u) n += pbFixed32(out + n, 4, time_s);  // time (fixed32!)
  n += pbUint32(out + n, 23, 32);         // precision_bits (field 23!) = full -> map pin
  return n;
}

// Meshtastic DeviceMetrics.battery_level is a PERCENTAGE, but a MeshCore board
// only exposes a raw millivolt reading (mesh::MainBoard::getBattMilliVolts).
// Map it across the usual 1S Li-ion working range, 3.3 V empty -> 4.2 V full.
// That is a coarse straight line through a curved discharge, but it is the same
// approximation used everywhere there's no fuel-gauge IC, and it is honest at
// the two ends that matter (nearly flat / nearly full).
// Returns -1 when the board reports nothing at all (no battery sense wired up),
// so the caller can omit the field rather than publish a confident 0%.
inline int batteryPercent(uint16_t milli_volts) {
  if (milli_volts == 0) return -1;
  const long lo = 3300, hi = 4200;
  long pct = ((long)milli_volts - lo) * 100 / (hi - lo);
  if (pct < 0) pct = 0;
  if (pct > 100) pct = 100;
  return (int)pct;
}

// Meshtastic Telemetry message carrying DeviceMetrics — what phone clients
// render as the node's battery ring and uptime:
//   Telemetry     { fixed32 time = 1; DeviceMetrics device_metrics = 2; }
//   DeviceMetrics { uint32 battery_level = 1;       float voltage = 2;
//                   float channel_utilization = 3;  float air_util_tx = 4;
//                   uint32 uptime_seconds = 5; }
//
// Telemetry.time is fixed32 (wire type 5), the same trap as Position.time — a
// varint there makes nanopb reject the whole message. Only stamped when the
// clock is plausibly real; otherwise the receiver uses its RX time.
//
// channel_utilization and air_util_tx are deliberately OMITTED. We do have
// airtime numbers, but they are for the MeshCore channel this node actually
// repeats on, not the Meshtastic one — publishing them would put a confidently
// wrong load figure on someone else's network map.
//
// Returns the payload length, or 0 if there is nothing worth reporting (no
// battery sense and no uptime), so the caller can skip the packet entirely.
inline int buildTelemetryPayload(uint8_t* out, uint16_t batt_mv,
                                 uint32_t uptime_s, uint32_t time_s) {
  uint8_t dm[32];
  int d = 0;
  int pct = batteryPercent(batt_mv);
  if (pct >= 0) {
    d += pbUint32(dm + d, 1, (uint32_t)pct);              // battery_level (%)
    d += pbFloat(dm + d, 2, (float)batt_mv / 1000.0f);    // voltage
  }
  if (uptime_s) d += pbUint32(dm + d, 5, uptime_s);       // uptime_seconds
  if (d == 0) return 0;

  int n = 0;
  if (time_s > 1600000000u) n += pbFixed32(out + n, 1, time_s);  // time (fixed32!)
  n += pbKey(out + n, 2, 2);                              // device_metrics, len-delimited
  n += pbVarint(out + n, (uint64_t)d);
  memcpy(out + n, dm, d); n += d;
  return n;
}

// ---------------------------------------------------------------------------
// Transmit outcome + health accounting.
//
// These live here, on the dependency-free side, so the bookkeeping and its
// formatting are host-testable; the code that actually talks to the radio and
// produces a TxOutcome is radioSendChecked() in MeshtasticBeacon.h.
// ---------------------------------------------------------------------------

// What actually happened to one transmit. Callers mostly care about the
// did-it-go-out question (txDelivered); the finer grades exist so a node can
// say WHY on the console instead of guessing.
enum TxOutcome : uint8_t {
  TX_NOT_STARTED,   // the radio refused to begin the transmit
  TX_IRQ,           // normal path: the TxDone interrupt arrived
  TX_HW_CONFIRMED,  // interrupt missed, but the chip's TxDone flag is set
  TX_HW_FAILED,     // interrupt missed AND the chip's TxDone flag is clear
  TX_PRESUMED,      // interrupt missed, chip can't report -> airtime bound only
};

inline bool txDelivered(TxOutcome o) {
  return o == TX_IRQ || o == TX_HW_CONFIRMED || o == TX_PRESUMED;
}
inline bool txUsedFallback(TxOutcome o) {   // the TxDone interrupt didn't fire
  return o == TX_HW_CONFIRMED || o == TX_HW_FAILED || o == TX_PRESUMED;
}

// Rolling transmit health. RAM only, by design: these are diagnostics for a
// drive test or a site visit, not something worth a flash write per packet —
// and a reboot is exactly the moment you want the counters to start over.
struct TxStats {
  uint32_t bursts;        // retunes to the Meshtastic channel that transmitted
  uint32_t lbt_skips;     // bursts abandoned because the channel was busy
  uint32_t attempts;      // packets handed to the radio
  uint32_t delivered;     // ... that went out (IRQ, chip-confirmed or presumed)
  uint32_t irq_missed;    // ... where the TxDone interrupt never arrived
  uint32_t hw_failed;     // ... and the chip then said it never transmitted
  uint32_t not_started;   // the radio refused to begin the transmit
  uint32_t recoveries;    // radio re-inits triggered by a run of dead transmits
  uint32_t last_fail_ms;  // millis() of the most recent failure (0 = none yet)
  uint8_t  fail_streak;   // consecutive dead transmits right now
};

// Fold one transmit outcome into the counters. `now_ms` is millis() (passed in
// so this stays free of Arduino). Returns the current failure streak, which the
// caller compares against its recovery threshold.
inline uint8_t txStatsRecord(TxStats& s, TxOutcome o, uint32_t now_ms) {
  s.attempts++;
  if (o == TX_NOT_STARTED) s.not_started++;
  if (o == TX_HW_FAILED)   s.hw_failed++;
  if (txUsedFallback(o))   s.irq_missed++;
  if (txDelivered(o)) {
    s.delivered++;
    s.fail_streak = 0;              // proof of life: the run of failures is over
  } else {
    if (s.fail_streak < 255) s.fail_streak++;
    s.last_fail_ms = now_ms ? now_ms : 1;   // never 0 — that means "no failure yet"
  }
  return s.fail_streak;
}

// One-line health summary for the CLI, e.g.
//   "beacon tx 61/64 ok | dead 3 irq-miss 7 nostart 0 | burst 22 lbt 1 | recov 1 (last fail 12m ago)"
//
// Read it as nested, not disjoint: "irq-miss" counts every transmit where the
// TxDone interrupt never arrived, and "dead" is the subset of those the chip
// then confirmed had not gone out. irq-miss high with dead 0 is the healthy
// case — the interrupt path is flaky but the packets are flying.
//
// `cap` is the caller's reply buffer size (160 on the MeshCore CLI).
// `age_ms` is millis() - last_fail_ms; ignored when no failure has happened.
inline void formatTxStats(char* out, size_t cap, const TxStats& s,
                          const char* tag, uint32_t age_ms) {
  char ago[40] = {0};
  if (s.last_fail_ms) {
    unsigned long mins = age_ms / 60000UL;
    if (mins >= 60) snprintf(ago, sizeof(ago), " (last fail %luh ago)", mins / 60);
    else            snprintf(ago, sizeof(ago), " (last fail %lum ago)", mins);
  }
  snprintf(out, cap,
           "%s tx %lu/%lu ok | dead %lu irq-miss %lu nostart %lu | burst %lu lbt %lu | recov %lu%s",
           tag,
           (unsigned long)s.delivered, (unsigned long)s.attempts,
           (unsigned long)s.hw_failed, (unsigned long)s.irq_missed,
           (unsigned long)s.not_started,
           (unsigned long)s.bursts, (unsigned long)s.lbt_skips,
           (unsigned long)s.recoveries, ago);
}

} // namespace meshtastic
