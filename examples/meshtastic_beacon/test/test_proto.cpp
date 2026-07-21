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

  printf(failures ? "\nFAILED: %d check(s)\n" : "\nAll checks passed.\n", failures);
  return failures ? 1 : 0;
}
