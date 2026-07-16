# MeshCore Car Node

A fork of [MeshCore](https://github.com/meshcore-dev/MeshCore) that turns a
repeater into a **mobile, in-vehicle node that reports where it parks**. When
the vehicle stops moving for a few minutes, it pushes its live GPS location to
*both* networks at once — a **Meshtastic beacon** and a fresh **MeshCore
advert** — so the node shows up where you actually left it, on both maps.

> **This is the `carnode` branch.** The car node lives in
> [`examples/car_node/`](examples/car_node/) — the
> [**full documentation is there**](examples/car_node/README.md).
> Prebuilt firmware + flashing instructions:
> [**alexdowney.net/carnode**](https://alexdowney.net/carnode/).

## What it does

- **Park-triggered** — while driving it sends no location, only a light
  presence. Once the GPS fix sits still past `park` seconds (default 5 min),
  one unified update fires on both networks with the same fix.
- **Jitter-proof** — the park logic uses the median of the last ~9 fixes and
  a movement-confirm window, so trunk-mounted GPS multipath can't fake a drive
  or misplace the pin.
- **Repeat sleep** — parked past `sleep` hours (default 20), the node stops
  repeating until it drives again; a car parked overnight isn't a useful
  mobile repeater.
- **Home** — `carnode home` stores where the vehicle lives. Parking there
  turns repeat off immediately and suppresses every broadcast, so the home
  location never goes on the air.
- **Self-recovering** — nRF52 boards arm a hardware watchdog (a hung node
  reboots itself) and answer `get pwrmgt.bootreason` with why they last reset.

Everything is runtime-configurable over serial or an admin remote-CLI session
via two verbs: `mtbeacon` (the Meshtastic beacon's RF + appearance) and
`carnode` (park behaviour). See the
[command reference](examples/car_node/README.md#runtime-control).

## Supported boards

| Board | Radio | GPS | Env |
| --- | --- | --- | --- |
| Heltec V4 | ESP32-S3 · SX1262 | external NMEA module | `heltec_v4_carnode` |
| Heltec T114 | nRF52840 · SX1262 | external NMEA module | `Heltec_t114_carnode` |
| Seeed T1000-E | nRF52840 · LR1110 | onboard | `t1000e_dualmode_carnode` |

The T1000-E build is [dual-mode](examples/dualmode/): a 5× button press
switches between the car node (repeater mode) and a normal phone-paired
companion node.

```
pio run -e heltec_v4_carnode
pio run -e Heltec_t114_carnode
pio run -e t1000e_dualmode_carnode
```

Adding a board is a small platformio.ini env on top of its `*_repeater` env —
see [the build docs](examples/car_node/README.md#building).

## Relationship to the other branches

The car node is built on the
[`meshtastic_beacon`](examples/meshtastic_beacon/) add-on (branch
[`meshtastic-beacon`](../../tree/meshtastic-beacon)) and reuses its
on-air-verified Meshtastic interop math unchanged; the static fixed-interval
beacon is replaced with the park-triggered, GPS-driven one. Everything else is
unmodified MeshCore — see the
[upstream project](https://github.com/meshcore-dev/MeshCore) for the base
firmware.

## Scope / etiquette

One-way presence only: the node appears on Meshtastic as a named node + map
pin (+ optional chat text) but does **not** route or rebroadcast Meshtastic
traffic. A park floods a MeshCore advert mesh-wide and adds airtime to a
shared public Meshtastic channel — keep `park` reasonable, respect your
region's limits, and don't beacon onto channels you don't operate.

## License

MIT, same as upstream MeshCore (see [`license.txt`](license.txt)). MeshCore
Car Node is an independent project and is not affiliated with or endorsed by
the MeshCore or Meshtastic projects. Meshtastic® is a registered trademark of
Meshtastic LLC.

## Built with generative AI

The car node, the underlying beacon add-on, and this fork's docs were
developed with the assistance of generative AI (Anthropic's Claude), then
reviewed and tested by a human. Read the source, verify behavior, and test
before relying on it.
