// Host unit tests for MeshtasticProto.h (the dependency-free half of the beacon).
// Build & run with test/run.sh  (or: c++ -std=c++17 test/test_proto.cpp && ./a.out)
//
// These pin the interop-critical math: the Meshtastic channel-frequency
// calculation, the channel-hash byte, and the NodeInfo/Position protobuf
// encoders — the things that, if wrong, make the beacon invisible or mislocated.

#include "../MeshtasticProto.h"
#include <cstdio>
#include <cmath>
#include <cstring>

using namespace meshtastic;

static int failures = 0;
#define CHECK(cond) do { \
  if (cond) { printf("  ok   %s\n", #cond); } \
  else { printf("  FAIL %s   (line %d)\n", #cond, __LINE__); failures++; } } while (0)

static bool near(float a, float b) { return std::fabs(a - b) < 1e-3f; }

static float freqOf(const char* region, const char* preset) {
  return presetFreq(REGIONS[findRegion(region)], PRESETS[findPreset(preset)]);
}

int main() {
  printf("channel hash:\n");
  CHECK(channelHash(DEFAULT_CHANNEL_NAME, DEFAULT_KEY, 16) == 0x08);
  CHECK(channelHash("MediumFast", DEFAULT_KEY, 16) == 0x1f);

  printf("frequency calc (matches Meshtastic):\n");
  CHECK(near(freqOf("US", "LongFast"),    906.875f));
  CHECK(near(freqOf("EU_868", "LongFast"), 869.525f));
  CHECK(near(freqOf("US", "MediumFast"),  913.125f));
  CHECK(near(freqOf("US", "ShortTurbo"),  926.750f));
  // narrow-band guard: 500 kHz preset in a 0.25 MHz band must not crash / divide by 0
  float f = freqOf("EU_868", "ShortTurbo");
  CHECK(f >= 869.4f && f <= 869.9f);

  printf("explicit frequency slots (1-based, matches Meshtastic):\n");
  {
    const Region& us = REGIONS[findRegion("US")];
    const Preset& lf = PRESETS[findPreset("LongFast")];
    CHECK(numChannels(us, lf) == 104);                  // 902-928 MHz / 250 kHz
    CHECK(near(slotFreq(us, lf, 20), 906.875f));        // US LongFast default = slot 20
    CHECK(near(slotFreq(us, lf, 1), 902.125f));         // first slot
    CHECK(near(slotFreq(us, lf, 104), 927.875f));       // last slot
    CHECK(near(presetFreq(us, lf), slotFreq(us, lf, 20)));  // auto == default slot
  }

  printf("table lookups (case-insensitive):\n");
  CHECK(findPreset("longfast") == 0);
  CHECK(findPreset("ShortTurbo") == 7);
  CHECK(findPreset("bogus") == -1);
  CHECK(findRegion("us") == 0);
  CHECK(findRegion("EU_868") == 1);
  CHECK(findRegion("zz") == -1);

  printf("region caps:\n");
  CHECK(REGIONS[findRegion("EU_433")].max_power_dbm == 12);
  CHECK(REGIONS[findRegion("US")].max_power_dbm == 30);
  CHECK(REGIONS[findRegion("EU_868")].duty_pct == 10);
  CHECK(REGIONS[findRegion("US")].duty_pct == 100);

  printf("NodeInfo (User) protobuf:\n");
  uint8_t u[128];
  int un = buildUserPayload(u, 0x12abcdef, "MC My Repeater", "cdef");
  CHECK(un == 36);                                      // +3 for hw_model
  CHECK(u[0] == 0x0a && u[1] == 0x09);                 // field1 id, len 9
  CHECK(memcmp(u + 2, "!12abcdef", 9) == 0);
  CHECK(u[11] == 0x12 && u[12] == 14);                 // field2 long_name, len 14
  CHECK(memcmp(u + 13, "MC My Repeater", 14) == 0);
  CHECK(u[27] == 0x1a && u[28] == 4);                  // field3 short_name, len 4
  CHECK(u[33] == 0x28 && u[34] == 0xff && u[35] == 0x01); // field5 hw_model=255 (varint)

  printf("Position protobuf:\n");
  uint8_t pos[64];
  int pn = buildPositionPayload(pos, 30.2672, -97.7431, 1751299200u);
  CHECK(pn == 18);                                      // time is fixed32, not varint
  CHECK(pos[0] == 0x0d);                                // field1 latitude_i, sfixed32 (wt5)
  int32_t lat_i; memcpy(&lat_i, pos + 1, 4);
  CHECK(lat_i == 302672000);
  CHECK(pos[5] == 0x15);                                // field2 longitude_i (wt5)
  int32_t lon_i; memcpy(&lon_i, pos + 6, 4);
  CHECK(lon_i == -977431000);
  CHECK(pos[10] == 0x25);                               // field4 time, FIXED32 (wt5) <- the fix
  uint32_t t; memcpy(&t, pos + 11, 4);
  CHECK(t == 1751299200u);
  CHECK(pos[15] == 0xb8 && pos[16] == 0x01 && pos[17] == 0x20); // field23 precision_bits=32

  // hw_model is configurable (default PRIVATE_HW; per-board override)
  uint8_t u2[128];
  int un2 = buildUserPayload(u2, 0x12abcdef, "MC R", "abcd", 9 /*RAK4631*/);
  CHECK(u2[un2 - 2] == 0x28 && u2[un2 - 1] == 0x09);   // field5 hw_model=9

  printf("short name (map marker label, 4 chars max):\n");
  char sn[5];
  defaultShortName(sn, 0x12abcdef);
  CHECK(strcmp(sn, "MCef") == 0);                       // "MC" + low byte, hex
  defaultShortName(sn, 0x00000007);
  CHECK(strcmp(sn, "MC07") == 0);                       // zero-padded
  resolveShortName(sn, "", 0x12abcdef);
  CHECK(strcmp(sn, "MCef") == 0);                       // empty config = auto
  resolveShortName(sn, nullptr, 0x12abcdef);
  CHECK(strcmp(sn, "MCef") == 0);
  resolveShortName(sn, "KC2K", 0x12abcdef);
  CHECK(strcmp(sn, "KC2K") == 0);                       // operator override wins
  resolveShortName(sn, "TOOLONG", 0x12abcdef);
  CHECK(strcmp(sn, "TOOL") == 0);                       // truncated to 4, terminated

  printf("battery percent (1S Li-ion, 3.3-4.2 V):\n");
  CHECK(batteryPercent(0)    == -1);                    // no battery sense -> omit the field
  CHECK(batteryPercent(3300) == 0);
  CHECK(batteryPercent(4200) == 100);
  CHECK(batteryPercent(3750) == 50);
  CHECK(batteryPercent(2500) == 0);                     // clamped, never negative
  CHECK(batteryPercent(5000) == 100);                   // clamped, never over 100

  printf("telemetry payload (DeviceMetrics):\n");
  uint8_t tp[64];
  int tn = buildTelemetryPayload(tp, 3750, 86400, 1751299200u);
  // Telemetry.time is field 1 FIXED32 (wt5) — a varint here voids the message,
  // exactly like Position.time.
  CHECK(tp[0] == 0x0d);
  uint32_t tt; memcpy(&tt, tp + 1, 4);
  CHECK(tt == 1751299200u);
  CHECK(tp[5] == 0x12);                                 // field 2 device_metrics, len-delimited
  int dlen = tp[6];
  CHECK(tn == 7 + dlen);                                // the submessage length is honest
  const uint8_t* dm = tp + 7;
  CHECK(dm[0] == 0x08 && dm[1] == 50);                  // field1 battery_level = 50%
  CHECK(dm[2] == 0x15);                                 // field2 voltage, FLOAT (wt5)
  float volts; memcpy(&volts, dm + 3, 4);
  CHECK(near(volts, 3.75f));
  CHECK(dm[7] == 0x28);                                 // field5 uptime_seconds, varint
  // no clock yet -> time omitted, but the metrics still go out
  int tn2 = buildTelemetryPayload(tp, 3750, 100, 0);
  CHECK(tp[0] == 0x12 && tn2 < tn);
  // nothing measurable at all -> caller should skip the packet entirely
  CHECK(buildTelemetryPayload(tp, 0, 0, 1751299200u) == 0);
  // no battery sense but a known uptime -> uptime only
  int tn3 = buildTelemetryPayload(tp, 0, 3600, 0);
  CHECK(tn3 > 0 && tp[2] == 0x28);

  printf("tx health accounting:\n");
  TxStats s = {};
  CHECK(txStatsRecord(s, TX_IRQ, 1000) == 0);           // clean send: no streak
  CHECK(s.delivered == 1 && s.attempts == 1 && s.irq_missed == 0);
  txStatsRecord(s, TX_HW_CONFIRMED, 2000);              // IRQ lost, chip says it went
  CHECK(s.delivered == 2 && s.irq_missed == 1 && s.hw_failed == 0 && s.fail_streak == 0);
  txStatsRecord(s, TX_PRESUMED, 3000);                  // chip can't tell -> still counts as sent
  CHECK(s.delivered == 3 && s.irq_missed == 2 && s.fail_streak == 0);
  CHECK(txStatsRecord(s, TX_HW_FAILED, 4000) == 1);     // chip says it never transmitted
  CHECK(txStatsRecord(s, TX_HW_FAILED, 5000) == 2);
  CHECK(s.hw_failed == 2 && s.delivered == 3 && s.last_fail_ms == 5000);
  // "dead" nests inside "irq-miss": a HW_FAILED transmit is one where the
  // interrupt never arrived AND the chip then said nothing went out.
  CHECK(s.irq_missed == 4);
  CHECK(txStatsRecord(s, TX_NOT_STARTED, 6000) == 3);   // a refused start extends the streak
  CHECK(s.not_started == 1 && s.irq_missed == 4);       // ... but never began, so not an IRQ miss
  CHECK(txStatsRecord(s, TX_IRQ, 7000) == 0);           // one good send clears the streak
  CHECK(s.last_fail_ms == 6000);                        // ... but the last failure is remembered
  // a failure at millis()==0 must not read back as "never failed"
  TxStats z = {};
  txStatsRecord(z, TX_HW_FAILED, 0);
  CHECK(z.last_fail_ms != 0);

  printf("tx health formatting:\n");
  char line[160];
  formatTxStats(line, sizeof(line), s, "beacon", 90u * 60000u);
  CHECK(strstr(line, "beacon tx 4/7 ok") != nullptr);
  CHECK(strstr(line, "dead 2 irq-miss 4 nostart 1") != nullptr);
  CHECK(strstr(line, "(last fail 1h ago)") != nullptr);
  TxStats fresh = {};
  formatTxStats(line, sizeof(line), fresh, "carnode", 0);
  CHECK(strstr(line, "carnode tx 0/0 ok") != nullptr);
  CHECK(strstr(line, "last fail") == nullptr);          // no failure yet -> no age clause
  CHECK(strlen(line) < 160);

  printf(failures ? "\nFAILED: %d check(s)\n" : "\nAll checks passed.\n", failures);
  return failures ? 1 : 0;
}
