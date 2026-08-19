# MeshCore Dual-Node

A fork of [MeshCore](https://github.com/meshcore-dev/MeshCore) that builds
**two apps into one firmware**: a phone-paired **BLE companion** that also
shows up on **Meshtastic** as a silent presence, and a
[**car-node repeater**](https://github.com/tek126/MeshCore-Carpeater) that
beacons its parked location onto both networks. **Press the user button five
times** to reboot into the other mode — one device, one flash, two jobs.

> Prebuilt firmware + install instructions:
> [**kc2kvy.com/dualnode**](https://kc2kvy.com/dualnode/) (T1000-E, T114) and
> [**kc2kvy.com/v4presence**](https://kc2kvy.com/v4presence/) (Heltec V4,
> including a companion-only build).

## The two modes

**Companion mode** (the default) is a stock MeshCore BLE companion — contacts,
channels, messages, map — plus a **Meshtastic presence**: the node appears in
Meshtastic node lists with a name, battery, and (optionally) a position that
is deliberately **fuzzed into an uncertainty circle** using Meshtastic's own
`precision_bits` (default ≈ 2.9 km). It never sends Meshtastic chat. Because a
BLE companion has no serial console, everything is set from the phone app's
**custom-variables editor** with no app changes: `mt.presence`, `mt.interval`,
`mt.position`, `mt.precision`, `mt.region`, `mt.preset`, `mt.slot` (explicit
Meshtastic frequency slot), and — on boards with a switchable LoRa front-end
like the Heltec V4 — `radio.fem.rxgain` / `radio.fem.txgain` for the external
LNA/PA. See [`examples/companion_radio/MTPRESENCE.md`](examples/companion_radio/MTPRESENCE.md).

**Repeater mode** is the [Carpeater](https://github.com/tek126/MeshCore-Carpeater)
car node: a MeshCore repeater that pushes its live GPS location to both
networks when the vehicle parks, with quiet zones, repeat sleep, a channel
block list, and the `carnode` / `mtbeacon` CLI verbs over serial or an admin
remote session. It runs headless — the screen belongs to companion mode.

## How the dual-boot works

The launcher ([`examples/dualmode/`](examples/dualmode/)) owns the real
Arduino `setup()`/`loop()`; both apps' entry points are renamed under
`-D DUALMODE` and only one app runs per boot, chosen by a flag file in the
internal filesystem. Five button presses (a count neither app uses) flip the
flag and reboot. The two apps' same-named classes are renamed apart so each
half keeps its own vtables, and each half persists its settings to its own
file — switching modes never clobbers the other side's config.

## Supported boards

| Board | Radio | Env | Notes |
| --- | --- | --- | --- |
| Seeed T1000-E | nRF52840 · LR1110 | `t1000e_dualmode_carnode_mtpresence` | onboard GPS; beeps confirm a mode switch |
| Heltec V4 | ESP32-S3 · SX1262 | `heltec_v4_dualmode_carnode_mtpresence` | switchable LNA (`radio.fem.rxgain`); screen on = companion mode |
| Heltec T114 | nRF52840 · SX1262 | `Heltec_t114_dualmode_carnode_mtpresence` | screen on = companion mode |

```
pio run -e t1000e_dualmode_carnode_mtpresence
pio run -e heltec_v4_dualmode_carnode_mtpresence
pio run -e Heltec_t114_dualmode_carnode_mtpresence
```

Plainer variants exist alongside (`t1000e_dualmode` = stock companion + stock
repeater; `t1000e_dualmode_carnode` = car node without the Meshtastic
presence).

## Related repos

Dual-Node is the **composition** of two sibling projects — their code is
canonical there and synced into this repo:

- [**MeshCore-mtbeacon**](https://github.com/tek126/MeshCore-mtbeacon) — the
  Meshtastic beacon engine: the fixed-repeater beacon fleet and the
  companion-side presence.
- [**MeshCore-Carpeater**](https://github.com/tek126/MeshCore-Carpeater) — the
  car node (the repeater half here).
- [**MeshCore**](https://github.com/meshcore-dev/MeshCore) — the upstream
  project; everything not listed above is unmodified base firmware.

## Scope / etiquette

One-way presence only, in both modes: the node appears on Meshtastic as a
named node + map pin but does **not** route or rebroadcast Meshtastic
traffic. The presence adds airtime to a shared public Meshtastic channel —
keep intervals reasonable, respect your region's limits, and don't beacon
onto channels you don't operate.

## License

MIT, same as upstream MeshCore (see [`license.txt`](license.txt)). MeshCore
Dual-Node is an independent project and is not affiliated with or endorsed by
the MeshCore or Meshtastic projects. Meshtastic® is a registered trademark of
Meshtastic LLC.

## Built with generative AI

The dual-mode launcher, the presence companion, the car node, and this fork's
docs were developed with the assistance of generative AI (Anthropic's Claude),
then reviewed and tested by a human. Read the source, verify behavior, and
test before relying on it.
