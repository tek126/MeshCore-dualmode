# How the beacon hooks into `simple_repeater`

Everything is behind `#ifdef WITH_MT_BEACON`, so a stock repeater build
(without the flag) compiles the guards to nothing and is byte-identical to
upstream. The add-on itself is header-only (`MtBeaconControl.h` +
`MeshtasticBeacon.h` / `MeshtasticProto.h`); the repeater owns one
`MtBeaconControl` instance and calls into it at six points.

## Hook points

### 1. Member + accessor — `MyMesh.h`

```cpp
#ifdef WITH_MT_BEACON
#include "MtBeaconControl.h"   // via -I examples/meshtastic_beacon
...
  MtBeaconControl _beacon;     // private member
...
  MtBeaconControl* getBeacon() { return &_beacon; }   // for the UI task
#endif
```

`FIRMWARE_VERSION` also gains a `+mtbeacon` suffix so `ver` shows which build
is flashed.

### 2. Identity + persisted config — `MyMesh::begin()`

The Meshtastic node number and starting packet id are derived from the
repeater's public key, so they're stable across reboots without any extra
state:

```cpp
const uint8_t* pk = self_id.pub_key;
uint32_t node = pk[0..3];   // big-endian
uint32_t seed = pk[4..7];
_beacon.begin(_fs, node, seed);   // loads /mtbeacon from the filesystem
```

### 3. CLI verbs — `MyMesh::handleCommand()`

`_beacon.handleCommand(command, reply, _fs)` is tried before the common CLI;
it claims only `mtbeacon ...` and returns false for everything else, so the
existing command surface is untouched. Works over serial and admin remote-CLI
sessions alike.

### 4. The beacon tick — tail of `MyMesh::loop()`

The repeater builds a `Context` (node name, configured lat/lon, RTC epoch,
flood-advert interval, and the CURRENT MeshCore radio params to restore —
including any active `tempradio` override) and calls:

```cpp
bool busy = hasPendingWork() || radio_driver.isReceiving()
         || !radio_driver.isInRecvMode();   // in-flight TX guard
_beacon.tick(radio_driver, radio, busy, ctx);
```

The `busy` guard is load-bearing: `hasPendingWork()` misses a packet that has
already been dequeued into an in-flight transmit, and `isReceiving()` is false
during TX — `!isInRecvMode()` covers that window so the beacon can never
retune the radio mid-transmit. When busy, the beacon defers a few seconds and
retries.

`tick()` does everything else internally: interval scheduling with jitter,
listen-before-talk (bounded CAD — it never calls RadioLib's blocking
`scanChannel()`, whose IRQ wait has no timeout), EU duty-cycle hold, the
retune/transmit/restore cycle, and an unconditional `startRecv()` after
restore so a power-saving node can't sleep with the radio in standby.

The transmit wait is bounded the same way (v0.2.4): `radioSendChecked()`
normally ends on the TxDone interrupt, but caps the wait at the packet's
estimated airtime x2 + 500 ms, since past that a transmit that started has
physically finished — so a lost interrupt no longer stalls the loop for seconds.

When the interrupt is missed it then reads the chip's own TxDone flag via
RadioLib's family-agnostic `checkIrq(RADIOLIB_IRQ_TX_DONE)` (v0.2.5) and
returns a `TxOutcome` saying which happened:

| outcome | meaning |
|---|---|
| `TX_IRQ` | normal — the interrupt arrived |
| `TX_HW_CONFIRMED` | interrupt missed, chip confirms it transmitted |
| `TX_HW_FAILED` | interrupt missed, chip says it never transmitted |
| `TX_PRESUMED` | interrupt missed, chip can't report — airtime bound only |
| `TX_NOT_STARTED` | `startSendRaw()` refused |

Use `txDelivered(r)` for the did-it-go-out question and `txUsedFallback(r)` for
the ISR-health one. The register read must precede `onSendFinished()`, whose
`finishTransmit()` clears the IRQ flags.

### 5. Flood-advert events — the chat text pacing (v0.2.3)

Both places the repeater floods a MeshCore advert notify the beacon, so the
chat text rides a beacon burst ~15 s later:

```cpp
// periodic timer block in MyMesh::loop()
updateFloodAdvertTimer();
_beacon.onFloodAdvert();

// and in sendSelfAdvertisement(delay, flood) when flood == true (CLI 'advert')
_beacon.onFloodAdvert();
```

This is what paces the text to the repeater's real advert cadence instead of a
boot-reset uptime timer. `onFloodAdvert()` arms `pending_text` and pulls the
next burst close; delivery still dedups through the single pending flag.

### 6. UI line — `UITask`

`ui_task.setBeacon(the_mesh.getBeacon())` (in `main.cpp` setup); the task
calls `_beacon->uiLine()` to show `Beacon ON LongFast` / `Beacon off` on the
OLED home screen.

## Shared-radio requirements

The beacon drives the raw RadioLib object directly for the off-band burst, so
`RadioLibWrapper::startRecv()` is public (it re-arms hardware RX and re-syncs
the wrapper's cached state unconditionally — `recvRaw()` is not a substitute,
since the wrapper's state can be stale after the raw retune).

## Adding a board

Copy the `*_repeater_mtbeacon` env shape onto the board's repeater env:

```ini
[env:<board>_repeater_mtbeacon]
extends = env:<board>_repeater
build_flags = ${env:<board>_repeater.build_flags}
  -D WITH_MT_BEACON
  -D MT_HW_MODEL=<n>        ; Meshtastic HardwareModel (255 = PRIVATE_HW)
  -I examples/meshtastic_beacon
lib_deps = ${env:<board>_repeater.lib_deps}
```

Only `MESHCORE_SYNC_WORD` needs overriding if a board's MeshCore sync word
differs from the default `0x12`. The radio helpers are templated on the
concrete radio class, so SX1262 / SX1276 / LR1110 all bind unchanged.
