#pragma once

#include <Arduino.h>
#include <Mesh.h>
#include <Utils.h>

// Bitchat bridge channel registry.
//
// Which #hashtag channels this repeater bridges between MeshCore and BitChat
// (BLE), configured by NAME alone — `bitchat add #offroad`. MeshCore hashtag
// channels derive their key deterministically from the name (channel key =
// SHA256("#name")[:16], channel hash = SHA256(key)[0]), and the bridge uses
// the same name on the BitChat side, so a name is all the configuration a
// channel needs. Private channels with random keys are deliberately out of
// scope, same as the `block` feature.
//
// Slot 0 is the DEFAULT channel: plain-text BitChat messages (which carry no
// channel field on the wire) are treated as belonging to it. On first boot the
// registry seeds itself with "#mesh" — the channel the upstream bridge
// hardcoded — so an unconfigured node behaves like the original.
//
// Only compiled into ENABLE_BITCHAT builds (see the *_carnode_bitchat env).

#define BITCHAT_CHANS_FILE   "/bc_chans"
#define BITCHAT_CHANS_MAGIC  0x42434331ul   // 'BCC1'

class BitchatChannels {
public:
  static const uint8_t MAX_CHANNELS = 4;   // matches BitchatBridge::MAX_CHANNEL_MAPPINGS
  static const uint8_t NAME_LEN = 24;      // stored name incl. leading '#' and null

private:
  struct Config {
    uint32_t magic;
    char names[MAX_CHANNELS][NAME_LEN];    // "" == free slot; always '#'-prefixed
  } cfg;
  mesh::GroupChannel channels[MAX_CHANNELS];  // derived from names at load / on edit
  bool used[MAX_CHANNELS];
  bool dirty_mappings;                     // set on edit; host re-syncs bridge mappings

  // Derive the GroupChannel for a #hashtag channel name. `name` includes the
  // leading '#'. Mirrors BaseChatMesh::addChannel(): the channel key is
  // SHA256(name)[:16] (rest of the 32-byte secret zeroed), and the on-wire
  // 1-byte channel hash is SHA256(key)[0].
  static void deriveChannel(const char* name, mesh::GroupChannel& out) {
    memset(&out, 0, sizeof(out));
    mesh::Utils::sha256(out.secret, 16, (const uint8_t*)name, strlen(name));
    mesh::Utils::sha256(out.hash, 1, out.secret, 16);
  }

  void recompute() {
    for (int i = 0; i < MAX_CHANNELS; i++) {
      used[i] = cfg.names[i][0] != 0;
      if (used[i]) deriveChannel(cfg.names[i], channels[i]);
      else memset(&channels[i], 0, sizeof(channels[i]));
    }
  }

  // Copy a raw token into a normalized '#'-prefixed, length-checked name.
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
      strcpy(reply, "Error: expected a channel, e.g. bitchat add #offroad");
      return false;
    }
    strcpy(out, tmp);
    return true;
  }

  int8_t find(const char* name) const {
    for (int i = 0; i < MAX_CHANNELS; i++)
      if (used[i] && strcmp(cfg.names[i], name) == 0) return i;
    return -1;
  }
  int8_t freeSlot() const {
    for (int i = 0; i < MAX_CHANNELS; i++) if (!used[i]) return i;
    return -1;
  }

  void listReply(char* reply) const {
    const int cap = 158;
    int n = snprintf(reply, cap, "Bridged:");
    bool any = false;
    for (int i = 0; i < MAX_CHANNELS && n < cap - 2; i++) {
      if (!used[i]) continue;
      // the first used slot is the default channel (marked *): plain-text
      // BitChat messages with no channel field are filed under it
      n += snprintf(reply + n, cap - n, " %s(%02x)%s", cfg.names[i], channels[i].hash[0],
                    any ? "" : "*");
      any = true;
    }
    if (!any) snprintf(reply, cap, "Bridged: (none)");
  }

public:
  void begin(FILESYSTEM* fs) {
    memset(&cfg, 0, sizeof(cfg));
    cfg.magic = BITCHAT_CHANS_MAGIC;
    if (!load(fs)) {
      // First boot: seed with the channel the upstream bridge hardcoded
      strcpy(cfg.names[0], "#mesh");
      save(fs);
    }
    recompute();
    dirty_mappings = true;   // force initial bridge sync
  }

  uint8_t count() const {
    uint8_t n = 0;
    for (int i = 0; i < MAX_CHANNELS; i++) if (used[i]) n++;
    return n;
  }
  bool slotUsed(int i) const { return i >= 0 && i < MAX_CHANNELS && used[i]; }
  const char* name(int i) const { return cfg.names[i]; }         // incl. '#'
  const mesh::GroupChannel& channel(int i) const { return channels[i]; }

  // True when mappings changed since the last takeDirty() — host re-registers
  // them with the bridge.
  bool takeDirty() { bool d = dirty_mappings; dirty_mappings = false; return d; }

  // For Mesh::searchChannelsByHash(): fill matches for a 1-byte channel hash.
  int findByHash(const uint8_t* hash, mesh::GroupChannel dest[], int max_matches) const {
    int n = 0;
    for (int i = 0; i < MAX_CHANNELS && n < max_matches; i++) {
      if (used[i] && channels[i].hash[0] == hash[0]) dest[n++] = channels[i];
    }
    return n;
  }

  bool load(FILESYSTEM* fs) {
#if defined(RP2040_PLATFORM)
    File f = fs->open(BITCHAT_CHANS_FILE, "r");
#else
    File f = fs->open(BITCHAT_CHANS_FILE);
#endif
    bool ok = false;
    if (f) {
      uint8_t buf[sizeof(Config)];
      memset(buf, 0, sizeof(buf));
      int n = f.read(buf, sizeof(buf));
      f.close();
      uint32_t magic = 0;
      if (n >= 4) memcpy(&magic, buf, sizeof(magic));
      if (n == (int)sizeof(Config) && magic == BITCHAT_CHANS_MAGIC) {
        memcpy(&cfg, buf, sizeof(cfg));
        ok = true;
      }
    }
    for (int i = 0; i < MAX_CHANNELS; i++) cfg.names[i][NAME_LEN - 1] = 0;  // ensure terminated
    return ok;
  }

  void save(FILESYSTEM* fs) {
#if defined(NRF52_PLATFORM) || defined(STM32_PLATFORM)
    fs->remove(BITCHAT_CHANS_FILE);
    File f = fs->open(BITCHAT_CHANS_FILE, FILE_O_WRITE);
#elif defined(RP2040_PLATFORM)
    File f = fs->open(BITCHAT_CHANS_FILE, "w");
#else
    File f = fs->open(BITCHAT_CHANS_FILE, "w", true);
#endif
    if (f) { f.write((uint8_t*)&cfg, sizeof(cfg)); f.close(); }
  }

  // CLI: `bitchat` / `bitchat list` (list; slot 0 marked * = default),
  // `bitchat add <#name>`, `bitchat del <#name>`. Returns true if the command
  // was one of ours. (`bitchat status` is handled by the caller, which has
  // access to the bridge's counters.)
  bool handleCommand(const char* command, char* reply, FILESYSTEM* fs) {
    if (strcmp(command, "bitchat") == 0 || strcmp(command, "bitchat list") == 0) {
      listReply(reply);
      return true;
    }
    if (memcmp(command, "bitchat add ", 12) == 0) {
      char name[NAME_LEN];
      if (!normalize(command + 12, name, reply)) return true;
      if (find(name) >= 0) { snprintf(reply, 158, "Already bridging %s", name); return true; }
      int8_t i = freeSlot();
      if (i < 0) { snprintf(reply, 158, "Error: all %d bridge slots in use", (int)MAX_CHANNELS); return true; }
      strcpy(cfg.names[i], name);
      used[i] = true;
      deriveChannel(name, channels[i]);
      save(fs);
      dirty_mappings = true;
      bool isDefault = true;   // default = first used slot
      for (int j = 0; j < i; j++) if (used[j]) { isDefault = false; break; }
      snprintf(reply, 158, "Bridging %s (hash %02x)%s", name, channels[i].hash[0],
               isDefault ? " as default" : "");
      return true;
    }
    if (memcmp(command, "bitchat del ", 12) == 0) {
      char name[NAME_LEN];
      if (!normalize(command + 12, name, reply)) return true;
      int8_t i = find(name);
      if (i < 0) { snprintf(reply, 158, "Not bridging %s", name); return true; }
      cfg.names[i][0] = 0;
      used[i] = false;
      memset(&channels[i], 0, sizeof(channels[i]));
      save(fs);
      dirty_mappings = true;
      snprintf(reply, 158, "Stopped bridging %s", name);
      return true;
    }
    return false;
  }
};
