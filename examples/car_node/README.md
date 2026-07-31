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

**Get it:** prebuilt firmware + flashing instructions at
[alexdowney.net/carnode](https://alexdowney.net/carnode/) · source on the
[`carnode` branch](https://github.com/tek126/MeshCore-mtbeacon/tree/carnode).

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
`LocationProvider` **passing `&rtc_clock`** — e.g.
`MicroNMEALocationProvider(Serial1, &rtc_clock)`. The clock pointer is what lets
GPS set the time: the map-pin and telemetry timestamps come from the RTC, and
on a board with no hardware RTC that clock is wrong until the first fix syncs it.
(Also declare `extern MomentaryButton user_btn;` if you want hold-to-hibernate.)

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
| `mtbeacon hops <0-3>` | Meshtastic hop limit for presence + text (default 0 = direct neighbors only) |
| `mtbeacon text <string>` | announced chat text (≤63 chars) |
| `mtbeacon text.mult <N>` | text pacing: rides each flood advert, plus N−1 timer-paced extras per period (0 = never) |
| `mtbeacon short <str\|auto>` | Meshtastic short name / map label (≤4 chars); `auto` = `MC` + 2 hex |
| `mtbeacon nodeinfo on/off` | include NodeInfo (named node) — default on |
| `mtbeacon position on/off` | include Position (live map pin) — default on |
| `mtbeacon telemetry on/off` | include Telemetry (battery + uptime) — default on |
| `mtbeacon stats` / `mtbeacon stats clear` | transmit health since boot / reset the counters |
| `mtbeacon presets` / `mtbeacon regions` | list available values |

**`carnode`** — the car-specific park behaviour:

| Command | Effect |
| --- | --- |
| `carnode` / `carnode status` | show drive state (nofix/driving/parked/sleeping/*zone*) + park config |
| `carnode park <sec>` | stopped time before an update fires (30–86400, default 300) |
| `carnode radius <m>` | movement within this counts as "stopped" (5–2000, default 30) |
| `carnode advertdelay <sec>` | gap from the Meshtastic burst to the MeshCore advert (0–600, default 10) |
| `carnode sleep <hours>` | parked this long → stop repeating until driving again (0–720, 0 = never, default 20) |
| `carnode zone` | list the quiet zones (up to 4) |
| `carnode zone add <name>` | quiet zone = the current spot; parking in it → repeat off immediately, radio quiet |
| `carnode zone del <name>` | forget that zone |
| `carnode zone radius <name> <m>` | how close to it counts (5–2000, default 100) |
| `carnode home [clear\|radius <m>]` | shorthand for the zone named `home` |

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

The two halves are independent once armed: if the mesh cannot queue the flood
advert (packet pool or send queue full while the repeater is forwarding a
burst), the car node retries the handoff a few times with a backoff rather than
losing the park silently, and leaves the periodic flood-advert timer alone so
the next scheduled advert still carries the location. `carnode status` shows
`adv!N` if any park re-advert was abandoned outright — normally absent.

**Periodic presence.** Between park events, a light presence beacon (base-mtbeacon
style) fires every `interval` minutes (default 30, `0` = park-only) so the node
doesn't age out of Meshtastic node lists. While **parked** it includes Position —
refreshing the pin at the parked spot; while **driving** (or without a fix) it
sends NodeInfo only, so the map pin never wanders off to a random mid-drive point.
Location still updates *only* at park time.

**Chat text.** The `mtbeacon text` message is **event-driven**: whenever the
repeater floods a MeshCore advert — the park re-advert, the periodic flood
advert, or an explicit CLI `advert` — the text rides a Meshtastic burst a few
seconds later. So each *new-spot* park announces itself in chat, and running
`advert` doubles as a live test. (`text.mult > 1` adds timer-paced extras
between adverts; `text.mult 0` disables the text entirely.) `mtbeacon status`
shows the pacing live, e.g. `txt1x~47h(due 13h)` — or `(due now)` when a text
is armed and waiting for the next burst. In a quiet zone nothing transmits; an
armed text just stays pending until you drive away.

**Hop limit.** Meshtastic packets go out at `hops 0` by default — heard by
direct neighbors, never rebroadcast. A car node is mobile and transmits from
wherever it parks, so rebroadcasting its position across someone else's mesh is
poor manners; `mtbeacon hops <0-3>` raises it if you specifically need the
reach. (Matches the base beacon's default since mtbeacon v0.2.0.)

**Map label.** Meshtastic draws a node's **short name** on the map marker. The
car node's defaults to `MC` + 2 hex of its node id (e.g. `MC7a`) — recognisable
as MeshCore at a glance, while staying distinct from other nearby beacons. Set
your own with `mtbeacon short <str>` (≤4 chars, e.g. a callsign) or go back with
`mtbeacon short auto`. The long name stays `MC <node name>`.
(mtbeacon v0.2.5 parity.)

**Transmit confirmation.** Every burst retunes the radio to the Meshtastic PHY
and back, and the TxDone interrupt that signals "transmit finished" can be lost
across that retune. The wait is bounded by the packet's estimated airtime so a
lost interrupt can never freeze the loop; if the interrupt doesn't arrive, the
node then reads the **radio chip's own TxDone flag** to determine whether the
packet actually went out, rather than assuming it did. A park burst that truly
didn't transmit is reported as a failure and retried, and an undelivered chat
text stays pending instead of being marked sent:

```
carnode: TxDone IRQ missed on 1/3 packet(s) - checked the chip instead
carnode: 1/3 packet(s) did NOT transmit (chip reports no TxDone)
```

After **3 dead transmits in a row** the node stops merely reporting it and
**re-initialises the radio** — standby, clear latched IRQ flags, re-arm the
driver's TxDone interrupt, reprogram the MeshCore PHY, re-enter receive. A car
node that has silently stopped transmitting is a car you can't find, and before
this a wedged modem stayed wedged until the ignition cycled.

```
carnode: 3 dead transmits in a row - reinitialising the radio
```

`mtbeacon stats` reports the running totals, which is what makes a drive test
readable afterwards instead of a scroll through serial output:

```
mtbeacon stats
> carnode tx 61/64 ok | dead 3 irq-miss 7 nostart 0 | burst 22 lbt 1 | recov 1 (last fail 12m ago)
```

`dead` nests inside `irq-miss`: `irq-miss` is every transmit whose interrupt
never arrived, `dead` the subset the chip confirmed never went out. High
`irq-miss` with `dead 0` is healthy — flaky interrupts, packets still flying.
The counters are RAM-only (a reboot resets them); `mtbeacon stats clear` zeroes
them on demand.

(mtbeacon v0.2.4 bounded the wait; v0.2.5 added the hardware check; v0.2.6
added the recovery and the counters.)

**Battery reporting.** Each park burst — and the periodic presence — carries a
Meshtastic **Telemetry** packet with the board's battery percentage, voltage and
uptime, so a car parked for three days shows its charge draining in any
Meshtastic client. Turn it off with `mtbeacon telemetry off`. Boards with no
battery sense report nothing rather than a misleading 0%.

> The MeshCore side relies on `advert_loc_policy = prefs` (the repeater default),
> which this build keeps. The car node writes the fix into prefs itself rather
> than using the `gps advert share` policy, so MeshCore updates *only* at park
> time — not on every periodic advert.

**Repeat sleep.** A car parked for a day is probably somewhere nobody needs a
mobile repeater. After the vehicle has sat still for `sleep` hours (default 20)
the node stops **forwarding** — remote status reports it disabled, and it drops
out of node-discover — while it still receives, answers its own commands,
adverts, and beacons. Forwarding resumes the moment driving is detected (the
parked fix moves outside `radius`). Losing the GPS fix while parked (underground
garage) does *not* wake it; only movement does. `carnode sleep 0` disables the
feature.

Suppression is a **runtime condition layered over** the `set repeat on|off`
pref, not a write to it. Your own setting is never touched, `get repeat` always
reports what *you* chose, and nothing can persist a sleeping state — a reboot
always starts awake. `carnode status` shows `rpt:off` while suppression is
active, which is the one place the pref and the effective state can disagree.

> Before v0.2.7 the sleep wrote the pref directly and restored it on wake. Any
> unrelated `set …` command persists the whole prefs struct, so changing a
> setting while parked in a quiet zone saved the suppressed value to flash —
> after which the wake path read it back as "the operator wanted this off" and
> never restored forwarding again. If a node has been quiet since, `set repeat
> on` once (parked outside any zone) clears it for good.

**Quiet zones.** Park where the vehicle usually lives and say
`carnode zone add home` — the node stores the current (median-filtered) spot
under that name, persisted in `/carnode`. From then on, parking within the
zone's radius (default 100 m) stops **forwarding** immediately — no waiting for
the `sleep` timer — on the logic that a place you park at every day already has
fixed coverage and doesn't need a mobile repeater idling in the driveway. Inside
a zone the node also goes **radio-quiet**: the park burst, the MeshCore
re-advert, *and* the periodic presence are all suppressed, so that location is
never put on the air (an explicit `carnode send` still transmits if you ask for
it). Driving away restores forwarding and the normal beacon behaviour, exactly
like waking from sleep. Losing the fix inside a zone (garage) keeps you in it; only
driving away clears it.

Up to **4 zones** can be stored — home, work, a regular customer site — each
with its own radius. `carnode zone` lists them and marks the one you're sitting
in with `*`; `carnode status` and the OLED show the zone's *name* as the state,
so "which of my places is it at" is answerable at a glance. Names are up to 8
characters. Re-running `carnode zone add <name>` for an existing name moves that
zone to where you are now, keeping its radius.

`carnode home`, `carnode home clear` and `carnode home radius <m>` still work —
they are shorthand for the zone named `home`, so nothing an existing operator
typed before v0.2.6 has changed. A home set on an older build is **migrated into
the zone table automatically** on first boot, keeping its radius.

Defaults: US LongFast, 5-min park / 30 m radius, 30-min presence, 22 dBm
(region-capped), 20 h repeat sleep, no zones until set, disabled until
`mtbeacon on`. The OLED home screen shows `CarNode driving` / `parked` /
`sleeping` / `<zone name>`.

## Reliability (nRF52 boards)

On nRF52 builds (T114, T1000-E) the firmware arms the **hardware watchdog**
(90 s, fed every loop pass): if the node ever hard-hangs, it reboots itself
instead of riding around dead until someone power-cycles it. DFU updates are
unaffected (the UF2 bootloader feeds a running watchdog), and the dual-mode
build keeps feeding it across the repeater/companion mode switch.

Every boot prints its **reset reason** to serial, and it's remotely queryable:

```
get pwrmgt.bootreason
> Reset: Watchdog
```

`Watchdog` there means the node hung and rescued itself — worth a bug report.

## Scope / etiquette

Same as the beacon: one-way presence only (it does not route or rebroadcast
Meshtastic traffic), shared public spectrum — respect your region's limits and
don't beacon onto channels you don't operate. Because a park floods a MeshCore
advert mesh-wide, keep `park` reasonable so frequent short stops don't spam the
mesh.
