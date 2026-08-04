# Meshtastic presence (BLE companion)

Make a MeshCore **companion** node also show up on the **Meshtastic** side — in
everyone's nodelist, and (optionally) on the map — without it ever sending a
Meshtastic chat message. It reuses the battle-tested beacon engine from the
repeater (`examples/meshtastic_beacon`) in a *presence-only* profile.

Built behind `-D WITH_MT_PRESENCE`. See the reference env
`Heltec_t114_companion_radio_ble_mtpresence` in `variants/heltec_t114/platformio.ini`.

## What it does (and doesn't)

- **Presence, not chat.** It periodically retunes to the Meshtastic channel and
  emits **NodeInfo** (so the node appears named in the nodelist), optionally a
  **Position** (a map pin), and **Telemetry** (battery + uptime), then retunes
  back. It never posts a Meshtastic text message — `text_mult` defaults to `0`.
- **The radio is the phone's link.** The retune only happens between packets
  (the engine defers while the mesh has queued/in-flight work), so the phone's
  BLE session is not disrupted. The presence cadence defaults to 30 min.
- **Off by default.** Presence starts disabled; enable it with `mt.presence:1`.

## Location is gated twice — it can't over-share

A companion is personal and mobile, so Position is handled conservatively:

1. **Only if MeshCore already shares location.** The Meshtastic Position is sent
   *only* when this node's own advert shares its location
   (`advert_loc_policy == ADVERT_LOC_SHARE`). If MeshCore isn't publishing a
   location, the beacon emits **NodeInfo only** — no pin, nothing leaked. This
   mirrors `MyMesh::advert()` exactly, so the two can never disagree.
2. **Fuzzed, not exact.** When location *is* shared, it's obfuscated using
   Meshtastic's own **position precision** (`precision_bits`): the low lat/lon
   bits are masked off and receivers draw an uncertainty *circle* instead of a
   point. Default is **13 bits ≈ a 2.9 km circle** ("this node lives in this
   town"), not a street address. Dial it with `mt.precision`.

| precision | radius | | precision | radius |
|---|---|---|---|---|
| 10 | ~23 km | | 16 | ~360 m |
| 12 | ~5.8 km | | 18 | ~90 m |
| 13 | ~2.9 km | | 19 | ~45 m |
| 14 | ~1.5 km | | 32 | exact pin |

`mt.precision:0` suppresses Position entirely even when location is shared.

## Configuration — from the phone app, no app changes

The BLE companion has no serial CLI, so presence is configured through the phone
app's generic **custom variables** editor (`CMD_GET/SET_CUSTOM_VAR`). These keys
appear automatically alongside the usual node settings:

| var | values | meaning |
|---|---|---|
| `mt.presence` | `0` / `1` | enable presence beaconing |
| `mt.interval` | `1`–`1440` | presence cadence, minutes |
| `mt.position` | `0` / `1` | include a Position pin at all |
| `mt.precision` | `0`, `10`–`32` | position fuzz (32 = exact, 0 = off) |
| `mt.region` | `US`, `EU_868`, … | Meshtastic region/band |
| `mt.preset` | `LongFast`, `MediumFast`, … | Meshtastic modem preset |

Each set is routed to the same validated `mtbeacon` config path the repeater's
serial CLI uses, so out-of-range values are rejected (the app sees an error) and
accepted values are persisted to `/mtbeacon`.

## Limits

- Only the default Meshtastic channel's presence is emitted (region + preset
  pick the frequency); it is not a Meshtastic router and never rebroadcasts.
- Meshtastic and MeshCore are incompatible on-air, so presence necessarily
  borrows the radio for a brief retune — infrequent, but validate on-air that
  your phone link tolerates it before relying on it.
