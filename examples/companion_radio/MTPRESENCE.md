# Meshtastic presence (BLE companion)

A MeshCore **companion** node can also show on **Meshtastic**. It shows with a name
in the node list, and on the map if you permit it. It does not send a Meshtastic
chat message. It uses the beacon engine from the repeater
(`examples/meshtastic_beacon`) in a presence-only mode.

Build it with `-D WITH_MT_PRESENCE`. For example envs, see
`Heltec_t114_companion_radio_ble_mtpresence` in `variants/heltec_t114/platformio.ini`
and `heltec_v4_companion_radio_ble_mtpresence` in `variants/heltec_v4/platformio.ini`.

## What it does

- **It sends a presence, not a chat message.** The node changes the radio to the
  Meshtastic channel at intervals. It sends **NodeInfo** (so the node shows with a
  name in the node list), an optional **Position** (a map point), and **Telemetry**
  (the battery and the run time). Then it changes the radio back. It does not send a
  Meshtastic text message. `text_mult` is `0` by default.
- **The radio is the phone link.** The node changes the radio only between packets.
  The engine waits while the mesh has queued or in-progress work. Thus the phone BLE
  session continues. The default interval is 30 minutes.
- **It is off by default.** The presence starts off. To start it, set `mt.presence`
  to `1`.

## The location has two controls

A companion is personal and mobile. Thus the node controls the location carefully:

1. **It sends a location only if MeshCore shares a location.** The node sends a
   Meshtastic Position only when it shares its location in its MeshCore advert
   (`advert_loc_policy == ADVERT_LOC_SHARE`). If MeshCore does not share a location,
   the node sends **NodeInfo only** — no point, no location data. This is the same
   as `MyMesh::advert()`, so the two always agree.
2. **It makes the location less accurate.** When the node shares a location, it makes
   the location less accurate. It uses the Meshtastic **position precision**
   (`precision_bits`): it removes the low bits of the latitude and the longitude, and
   the Meshtastic client shows an *uncertainty circle*, not a point. The default is
   **13 bits (a circle near 2.9 km)** — "this node is in this town", not a street
   address. Set it with `mt.precision`.

| precision | circle | | precision | circle |
|---|---|---|---|---|
| 10 | ~23 km | | 16 | ~360 m |
| 12 | ~5.8 km | | 18 | ~90 m |
| 13 | ~2.9 km | | 19 | ~45 m |
| 14 | ~1.5 km | | 32 | exact point |

`mt.precision:0` stops the Position fully, even when the node shares a location.

## How to configure it from the phone app

The BLE companion does not have a serial CLI. Thus you set the presence from the
**custom-variables** list in the phone app (`CMD_GET/SET_CUSTOM_VAR`). These
variables show automatically with the other node settings. You do not need to change
the app.

| Variable | Values | Function |
|---|---|---|
| `mt.presence` | `0` / `1` | Set the presence to on. The default is off. |
| `mt.interval` | `1`–`1440` | The interval between presence messages, in minutes. |
| `mt.position` | `0` / `1` | Set the map point to on or off. |
| `mt.precision` | `0`, `10`–`32` | The location accuracy (32 = exact, 0 = off). |
| `mt.region` | `US`, `EU_868`, and others | The Meshtastic region and band. |
| `mt.preset` | `LongFast`, and others | The Meshtastic modem preset. |
| `mt.slot` | `0`–num. channels | The Meshtastic frequency slot (1-based); `0` = the default slot for the channel. |

Each value goes to the same `mtbeacon` configuration path that the repeater serial
CLI uses. Thus the firmware refuses a value that is out of range (the app shows an
error), and it keeps a good value in the `/mtbeacon` file.

### External LoRa FEM gain (LNA / PA)

Boards with a switchable LoRa front-end module (for example, the Heltec V4 LNA)
show two more variables in the same list. These are not presence settings — they
show in every companion build on such a board:

| Variable | Values | Function |
|---|---|---|
| `radio.fem.rxgain` | `0` / `1` | The external RX LNA. The default is on. |
| `radio.fem.txgain` | `0` / `1` | The external TX PA gain, on boards that have it. |

These are the same settings the repeater CLI names `radio.fem.rxgain` and
`radio.fem.txgain`. The value applies to the hardware immediately, persists, and
applies again at boot. A board with no switchable FEM does not show the variables.

## Limits

- The node sends the presence for the default Meshtastic channel only (the region and
  the preset select the frequency). The node is not a Meshtastic router; it does not
  repeat traffic.
- Meshtastic and MeshCore are not compatible on the air. Thus a presence needs a short
  radio change. This is not frequent. Before you depend on it, make a test on the air
  to make sure that your phone link is stable.
