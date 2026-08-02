#pragma once

#include <Arduino.h>
#include <Mesh.h>
#include <Utils.h>

// Channel repeat blocklist.
//
// Lets a repeater operator stop forwarding specific #hashtag channels by NAME
// alone -- e.g. `block #dispatches`. MeshCore hashtag channels derive their key
// deterministically from the name (channel key = SHA256("#name")[:16]), so the
// on-wire 1-byte channel hash -- SHA256(key)[0], carried at payload[0] of
// PAYLOAD_TYPE_GRP_TXT / GRP_DATA packets -- is computable here WITHOUT ever
// holding the channel's key. We persist the human-readable names and derive the
// match bytes at load time, so the list reads back in the operator's language.
//
// Two caveats, both inherent to the wire format (not choices we can undo):
//  - Only #hashtag (name-keyed) channels can be blocked this way. A private
//    channel with a random shared key is not derivable from its name; blocking
//    it would need the key, which this feature deliberately does not take.
//  - The channel id on the wire is a single byte, so a block can occasionally
//    also catch an unrelated channel that happens to share the hash byte
//    (~1 in 256). `block list` prints the byte so a collision is diagnosable.
//
// This is a generic repeater feature: it lives in simple_repeater and is not
// gated behind WITH_CAR_NODE / WITH_MT_BEACON, so every build inherits it.

#define CHAN_BLOCK_FILE   "/ch_block"
#define CHAN_BLOCK_MAGIC  0x43424B31ul   // 'CBK1'

class ChannelBlocker {
public:
  static const uint8_t MAX_BLOCKED = 8;
  static const uint8_t NAME_LEN = 24;    // stored name incl. leading '#' and null

private:
  struct Config {
    uint32_t magic;
    char names[MAX_BLOCKED][NAME_LEN];   // "" == free slot; always '#'-prefixed
  } cfg;
  uint8_t hashes[MAX_BLOCKED];           // derived from names at load / on edit
  bool    used[MAX_BLOCKED];

  // Derive the 1-byte on-wire channel hash for a #hashtag channel name.
  // `name` includes the leading '#'. Mirrors BaseChatMesh::addChannel(): the
  // channel key is SHA256(name)[:16], and the channel hash is SHA256(key)[0].
  static uint8_t deriveHash(const char* name) {
    uint8_t key[16];
    mesh::Utils::sha256(key, sizeof(key), (const uint8_t*)name, strlen(name));
    uint8_t hb = 0;
    mesh::Utils::sha256(&hb, 1, key, sizeof(key));
    return hb;
  }

  void recompute() {
    for (int i = 0; i < MAX_BLOCKED; i++) {
      used[i]   = cfg.names[i][0] != 0;
      hashes[i] = used[i] ? deriveHash(cfg.names[i]) : 0;
    }
  }

  // Copy a raw token into a normalized '#'-prefixed, length-checked name.
  // Accepts `#dispatches` or bare `dispatches` (we add the '#', since the '#'
  // is part of the hashed string). Returns false with an error in `reply` if the
  // token is empty or too long.
  static bool normalize(const char* in, char* out, char* reply) {
    while (*in == ' ') in++;
    char tmp[NAME_LEN];
    int j = 0;
    if (*in != '#') tmp[j++] = '#';
    while (*in && *in != ' ' && j < NAME_LEN - 1) tmp[j++] = *in++;
    tmp[j] = 0;
    if (*in && *in != ' ') {
      snprintf(reply, 158, "Error: channel name too long (max %d chars)", NAME_LEN - 2);
      return false;
    }
    if (j <= 1) {   // nothing but the '#'
      strcpy(reply, "Error: expected a channel, e.g. block #dispatches");
      return false;
    }
    strcpy(out, tmp);
    return true;
  }

  int8_t find(const char* name) const {
    for (int i = 0; i < MAX_BLOCKED; i++)
      if (used[i] && strcmp(cfg.names[i], name) == 0) return i;
    return -1;
  }
  int8_t freeSlot() const {
    for (int i = 0; i < MAX_BLOCKED; i++) if (!used[i]) return i;
    return -1;
  }

  void listReply(char* reply) const {
    const int cap = 158;
    int n = snprintf(reply, cap, "Blocked:");
    bool any = false;
    for (int i = 0; i < MAX_BLOCKED && n < cap - 2; i++) {
      if (!used[i]) continue;
      any = true;
      n += snprintf(reply + n, cap - n, " %s(%02x)", cfg.names[i], hashes[i]);
    }
    if (!any) snprintf(reply, cap, "Blocked: (none)");
  }

public:
  void begin(FILESYSTEM* fs) {
    memset(&cfg, 0, sizeof(cfg));
    cfg.magic = CHAN_BLOCK_MAGIC;
    load(fs);
    recompute();
  }

  // True if a group packet on this 1-byte channel hash should NOT be forwarded.
  bool isBlocked(uint8_t channel_hash) const {
    for (int i = 0; i < MAX_BLOCKED; i++)
      if (used[i] && hashes[i] == channel_hash) return true;
    return false;
  }

  void load(FILESYSTEM* fs) {
#if defined(RP2040_PLATFORM)
    File f = fs->open(CHAN_BLOCK_FILE, "r");
#else
    File f = fs->open(CHAN_BLOCK_FILE);
#endif
    if (f) {
      uint8_t buf[sizeof(Config)];
      memset(buf, 0, sizeof(buf));
      int n = f.read(buf, sizeof(buf));
      f.close();
      uint32_t magic = 0;
      if (n >= 4) memcpy(&magic, buf, sizeof(magic));
      if (n == (int)sizeof(Config) && magic == CHAN_BLOCK_MAGIC) {
        memcpy(&cfg, buf, sizeof(cfg));
      }
    }
    for (int i = 0; i < MAX_BLOCKED; i++) cfg.names[i][NAME_LEN - 1] = 0;  // ensure terminated
  }

  void save(FILESYSTEM* fs) {
#if defined(NRF52_PLATFORM) || defined(STM32_PLATFORM)
    fs->remove(CHAN_BLOCK_FILE);
    File f = fs->open(CHAN_BLOCK_FILE, FILE_O_WRITE);
#elif defined(RP2040_PLATFORM)
    File f = fs->open(CHAN_BLOCK_FILE, "w");
#else
    File f = fs->open(CHAN_BLOCK_FILE, "w", true);
#endif
    if (f) { f.write((uint8_t*)&cfg, sizeof(cfg)); f.close(); }
  }

  // CLI: `block` / `block list` (list), `block <#name>` (add), `unblock <#name>`
  // (remove). Returns true if the command was one of ours.
  bool handleCommand(const char* command, char* reply, FILESYSTEM* fs) {
    if (strcmp(command, "block") == 0 || memcmp(command, "block ", 6) == 0) {
      const char* arg = command + 5;
      while (*arg == ' ') arg++;
      if (*arg == 0 || strcmp(arg, "list") == 0) { listReply(reply); return true; }
      char name[NAME_LEN];
      if (!normalize(arg, name, reply)) return true;
      if (find(name) >= 0) { snprintf(reply, 158, "Already blocking %s", name); return true; }
      int8_t i = freeSlot();
      if (i < 0) { snprintf(reply, 158, "Error: all %d block slots in use", (int)MAX_BLOCKED); return true; }
      strcpy(cfg.names[i], name);
      used[i]   = true;
      hashes[i] = deriveHash(name);
      save(fs);
      snprintf(reply, 158, "Blocking %s (hash %02x)", name, hashes[i]);
      return true;
    }
    if (memcmp(command, "unblock ", 8) == 0) {
      char name[NAME_LEN];
      if (!normalize(command + 8, name, reply)) return true;
      int8_t i = find(name);
      if (i < 0) { snprintf(reply, 158, "Not blocking %s", name); return true; }
      cfg.names[i][0] = 0;
      used[i]   = false;
      hashes[i] = 0;
      save(fs);
      snprintf(reply, 158, "Unblocked %s", name);
      return true;
    }
    return false;
  }
};
