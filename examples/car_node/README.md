# car_node

A **mobile, in-vehicle** MeshCore repeater that reports **where it parks**. When
the vehicle stops moving for a few minutes, it pushes its GPS location to *both*
networks at once — a Meshtastic beacon **and** a fresh MeshCore advert — so the
node shows up where you actually left it, on both maps.

This is a fork of the [`meshtastic_beacon`](../meshtastic_beacon/)
project. It reuses that project's
on-air-verified interop math (`MeshtasticProto.h` / `MeshtasticBeacon.h`)
unchanged, and replaces the static, fixed-interval beacon with a park-triggered,
GPS-driven one.

## What's different from mtbeacon

| | mtbeacon | car_node |
| --- | --- | --- |
| Position source | repeater's configured lat/lon (static) | **live GPS fix** at park time |
| Trigger | fixed interval (minutes) | **vehicle parks** (stopped > `park` sec) + light presence every `interval` min |
| Networks updated | Meshtastic only | **Meshtastic + MeshCore together** (one fix, one event) |
| While driving | beacons periodically | presence only (NodeInfo, no position) at `interval` |
| Power posture | frugal (shared spectrum) | favours visibility (still LBT + region cap + EU duty) |
| CLI verbs | `mtbeacon ...` | `mtbeacon ...` (beacon RF) + `carnode ...` (park) |
| Config file | `/mtbeacon` | `/carnode` |

`CarNodeControl.h` is header-only and drops into `simple_repeater` behind
`-D WITH_CAR_NODE`, exactly like the beacon's `-D WITH_MT_BEACON` (the two can
even coexist — separate config, separate verbs). The interop headers stay
single-sourced in `meshtastic_beacon` via `-I examples/meshtastic_beacon`.

## Building

The Heltec boards expose GPS UART pins in their base variant, so an external
NMEA GPS module is all the extra hardware you need; the T1000-E has GPS onboard.
GPS defaults **on** for these builds.

```
pio run -e heltec_v4_carnode          # Heltec V4     (ESP32-S3, SX1262)
pio run -e Heltec_t114_carnode        # Heltec T114   (nRF52840, SX1262)
pio run -e t1000e_dualmode_carnode    # Seeed T1000-E (nRF52840, LR1110)
```

The T1000-E build is the [dual-mode](../dualmode/) firmware: `companion_radio`
is co-compiled alongside the repeater and a 5x button press reboots into the
other mode. The car node runs in **repeater mode**; switch to companion mode and
it's a normal phone-paired MeshCore node. Onboard GPS, button and buzzer — no
external hardware at all.

The V4 env (the T114 one is the same shape, plus `-D ENV_INCLUDE_GPS=1` since the
T114 repeater base doesn't enable GPS by default):

```ini
[env:heltec_v4_carnode]
extends = env:heltec_v4_repeater
build_flags = ${env:heltec_v4_repeater.build_flags}
  -D WITH_CAR_NODE
  -D MT_HW_MODEL=110
  -I examples/meshtastic_beacon
  -I examples/car_node
lib_deps = ${env:heltec_v4_repeater.lib_deps}
```

To add another board, copy that shape onto its `*_repeater` env: set
`MT_HW_MODEL` to the board's Meshtastic HardwareModel, add `-D ENV_INCLUDE_GPS=1`
if its base doesn't already, and make sure the variant constructs a GPS
`LocationProvider` (and declares `extern MomentaryButton user_btn;` if you want
hold-to-hibernate).

## Hibernate: hold the user button (~3 s)

Hold the board's user button for `CAR_NODE_HOLD_OFF_MILLIS` (default **3000 ms**,
overridable via `-D`) and the node hibernates through `board.powerOff()`:

- **T114** — true system-off (`sd_power_system_off`); GPS is powered down. Wakes
  on the user button / reset.
- **T1000-E** — same system-off via `T1000eBoard::powerOff()` (GPS and sensor
  rails powered down, button SENSE wake). Repeater mode only — a 5x press still
  switches modes, a ~3 s hold hibernates. In companion mode the usual companion
  button behaviour applies instead.
- **V4** — button-only deep sleep (`HeltecV4Board::hibernateButtonWake`): it
  stays asleep through incoming LoRa traffic and revives **only** on the user
  button. Because that button is GPIO0 (the BOOT strapping pin), the firmware
  waits for you to release it before sleeping, and on wake a brief **tap** is
  safest — holding GPIO0 down through the reset can drop the chip into
  serial-download mode instead of booting.

Detection uses the shared `MomentaryButton user_btn` (handles each board's
button polarity), so it works on any board that declares one. The OLED briefly
shows "Hibernating..." before the board powers down.

## Runtime control

Over serial or an admin remote-CLI session. The command surface is split into
two verbs — one underlying beacon engine, no duplicate transmitters:

**`mtbeacon`** — the Meshtastic beacon itself (RF + appearance):

| Command | Effect |
| --- | --- |
| `mtbeacon` / `mtbeacon status` | show beacon RF config + node id |
| `mtbeacon on` / `mtbeacon off` | enable / disable beaconing |
| `mtbeacon send` | push a location update now (both networks) |
| `mtbeacon interval <min>` | periodic presence between parks (0–1440, 0 = park-only, default 30) |
| `mtbeacon preset <name>` | modem preset (LongFast, MediumFast, …) |
| `mtbeacon region <name>` | region/country band (US, EU_868, …) |
| `mtbeacon freq <MHz\|auto>` | manual frequency override; `auto` re-derives |
| `mtbeacon power <dBm>` | TX power (−9…22), capped to region limit |
| `mtbeacon text <string>` | announced chat text (≤63 chars) |
| `mtbeacon text.mult <N>` | post text N times per flood-advert period (0 = never) |
| `mtbeacon nodeinfo on/off` | include NodeInfo (named node) — default on |
| `mtbeacon position on/off` | include Position (live map pin) — default on |
| `mtbeacon presets` / `mtbeacon regions` | list available values |

**`carnode`** — the car-specific park behaviour:

| Command | Effect |
| --- | --- |
| `carnode` / `carnode status` | show drive state (nofix/driving/parked/sleeping/home) + park config |
| `carnode park <sec>` | stopped time before an update fires (30–86400, default 300) |
| `carnode radius <m>` | movement within this counts as "stopped" (5–2000, default 30) |
| `carnode advertdelay <sec>` | gap from the Meshtastic burst to the MeshCore advert (0–600, default 10) |
| `carnode sleep <hours>` | parked this long → stop repeating until driving again (0–720, 0 = never, default 20) |
| `carnode home` | set home = the current spot; parking at home → repeat off immediately, radio quiet |
| `carnode home clear` | forget the home location |
| `carnode home radius <m>` | how close to home counts as home (5–2000, default 100) |

**Park model.** The node watches its GPS fix. While the position keeps moving
outside `radius` metres, it's *driving* and sends no location. Once the fix sits
within `radius` for `park` seconds (default 5 min), the vehicle is *parked* and a
single **unified update** fires. Two defenses make this reliable on a
poorly-sited GPS (trunk mount, metallized glass): the park logic — and the
position that gets broadcast — uses the **median of the last ~9 fixes**
(~27 s window), so a single fix teleporting hundreds of metres is rejected
outright; and movement must then persist for ~15 s outside the radius before it
counts as driving. `carnode status` shows the live tracking (`d<n>m` = filtered
fix's distance from the park anchor — `(raw)` while the filter warms up,
`<n>min` = how long the stationary clock has run); if it still flips to
*driving* while the car sits still, raise `carnode radius`. The update:

1. a **Meshtastic** beacon burst (NodeInfo + Position [+ text]), and
2. a **MeshCore** update — the repeater writes the parked fix into its
   `NodePrefs` location and **floods a fresh advert**, so the whole MeshCore mesh
   learns where you stopped.

Both use the *same* fix at the *same* moment. Re-parking in the same spot won't
re-broadcast (dedup by `radius`); `mtbeacon send` forces an update immediately.
The MeshCore location is persisted, so it survives a reboot while parked.

**Periodic presence.** Between park events, a light presence beacon (base-mtbeacon
style) fires every `interval` minutes (default 30, `0` = park-only) so the node
doesn't age out of Meshtastic node lists. While **parked** it includes Position —
refreshing the pin at the parked spot; while **driving** (or without a fix) it
sends NodeInfo only, so the map pin never wanders off to a random mid-drive point.
Location still updates *only* at park time. The chat text also rides these
presence cycles when due.

> The MeshCore side relies on `advert_loc_policy = prefs` (the repeater default),
> which this build keeps. The car node writes the fix into prefs itself rather
> than using the `gps advert share` policy, so MeshCore updates *only* at park
> time — not on every periodic advert.

**Repeat sleep.** A car parked for a day is probably somewhere nobody needs a
mobile repeater. After the vehicle has sat still for `sleep` hours (default 20)
the node turns **repeat off** — the same switch as `set repeat off`, so
`get repeat`, remote status and the actual forwarding all agree — while it still
receives, answers its own commands, adverts, and beacons. Repeat switches back
on the moment driving is detected (the parked fix moves outside `radius`);
if *you* had set repeat off before the sleep tripped, waking leaves it off.
Losing the GPS fix while parked (underground garage) does *not* wake it; only
movement does. The sleep toggle is not persisted: a reboot starts awake, and
`carnode sleep 0` disables the feature.

**Home.** Park where the vehicle usually lives and say `carnode home` — the node
stores the current (median-filtered) spot as *home*, persisted in `/carnode`.
From then on, parking within `home radius` metres of it (default 100) turns
**repeat off** immediately — no waiting for the `sleep` timer — on the logic that
home already has fixed coverage and doesn't need a mobile repeater idling in the
driveway. While at home the node also goes **radio-quiet**: the park burst, the
MeshCore re-advert, *and* the periodic presence are all suppressed, so the home
location is never put on the air (an explicit `carnode send` still transmits if
you ask for it). Driving away restores repeat and the normal beacon behaviour,
exactly like waking from sleep. Losing the fix at home (garage) keeps it home;
only driving away clears it. `carnode home clear` forgets the location.

Defaults: US LongFast, 5-min park / 30 m radius, 30-min presence, 22 dBm
(region-capped), 20 h repeat sleep, no home until set, disabled until
`mtbeacon on`. The OLED home screen shows `CarNode driving` / `parked` /
`sleeping` / `home`.

## Scope / etiquette

Same as the beacon: one-way presence only (it does not route or rebroadcast
Meshtastic traffic), shared public spectrum — respect your region's limits and
don't beacon onto channels you don't operate. Because a park floods a MeshCore
advert mesh-wide, keep `park` reasonable so frequent short stops don't spam the
mesh.
