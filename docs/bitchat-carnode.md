# BitChat bridge on the car node (EXPERIMENTAL)

The `heltec_v4_carnode_bitchat` build adds a BitChat BLE bridge to the car
node repeater: phones running [BitChat](https://bitchat.app) near the node can
exchange messages with MeshCore hashtag channels, while the node keeps doing
its normal repeater + Meshtastic-beacon jobs.

Based on jooray's MeshCore-BitChat bridge (`feature/bitchat-bridge`), with
three substantial changes:

1. **Runs on a repeater, not a companion.** The upstream bridge required a
   companion node. Here the repeater's `Mesh` layer registers the bridged
   channels (`searchChannelsByHash` / `onGroupDataRecv`), decrypts just those,
   and still repeats every packet unchanged. Since the repeater has no other
   BLE user, the bridge owns the BLE stack in standalone mode — the
   configuration the upstream author documented as the reliable one.

2. **Bridged channels are configurable, and plural.** Upstream hardcoded
   `#mesh`. Here up to 4 hashtag channels are configured by name and persisted
   (`/bc_chans`); keys derive from the name (`SHA256("#name")[:16]`), same rule
   as MeshCore hashtag rooms, so a name is all the configuration needed.
   Private random-key channels are deliberately out of scope.

3. **Stability rework** for the freeze the upstream build shows (whole node
   dead until power cycle):
   - all unconditional `Serial` debug prints and `Serial.flush()` calls in the
     per-message parse and announce paths are compiled out unless
     `BITCHAT_DEBUG` — on a headless USB-CDC node a blocked flush can stall
     the loop task indefinitely;
   - the BLE write-reassembly buffer shared between the Bluedroid task and the
     main loop is now guarded by a spinlock, with snapshot-based parsing;
   - outgoing notifies go through a paced queue (one per 40 ms) instead of
     back-to-back bursts, which are a known way to wedge Bluedroid;
   - a 60 s ESP32 task watchdog reboots the node if the loop ever wedges
     anyway;
   - the bridge's RTC sync from phone packet timestamps is bounded by the
     TimeSanity plausibility window (see `src/helpers/TimeSanity.h`), so a
     phone with a broken clock cannot poison the repeater's time.

## CLI

```
bitchat              list bridged channels ('*' marks the default)
bitchat add #name    bridge a channel (max 4)
bitchat del #name    stop bridging a channel
bitchat status       ble state, client presence, relay counters
```

First boot seeds the registry with `#mesh` (what upstream hardcoded). The
default channel — the first used slot — is where plain-text BitChat messages
(which carry no channel field on the wire) are filed, and its mesh→BitChat
messages use the proven plain-text broadcast form. Other channels use the
canonical TLV message form with the channel field set.

## Behavior notes

- Messages from BitChat appear on MeshCore prefixed with 📱; messages from
  MeshCore appear in BitChat as `<sender> text`. Both markers are
  loop-prevention: the bridge won't re-relay its own output.
- Power-saving sleep is suppressed while the bridge runs (ESP32 light sleep
  kills BLE).
- Long messages split into `[1/N]` parts, 15 s apart, ≤8 parts.
- The `ver` command reports `v1.17.1+carnode-bitchat-<git sha>` so an
  experimental node is unmistakable.

## Build

```
pio run -e heltec_v4_carnode_bitchat
```

V4 (ESP32-S3) only for now. T1000-E later — nRF52 brings the SoftDevice heap
limits (no long-message decompression) and a different BLE service backend.
