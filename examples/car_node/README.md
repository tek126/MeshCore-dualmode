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
| Location source | a location that you set (fixed) | **the live GPS location** when the vehicle stops |
| Trigger | a fixed interval (minutes) | **the vehicle stops** (more than `park` seconds); a small presence also goes out each `interval` minutes |
| Networks updated | Meshtastic only | **Meshtastic and MeshCore together** (one location, one event) |
| While driving | it sends a beacon at intervals | it sends a presence only (NodeInfo, no location) each `interval` |
| Power use | low (shared spectrum) | more visibility (still LBT, region cap, and EU duty cycle) |
| CLI commands | `mtbeacon ...` | `mtbeacon ...` (beacon RF) and `carnode ...` (park) |
| Configuration file | `/mtbeacon` | `/carnode` |

`CarNodeControl.h` is header-only. It goes into `simple_repeater` with
`-D WITH_CAR_NODE`, like the beacon's `-D WITH_MT_BEACON`. The two can operate
together (separate configuration, separate commands). The interop headers stay in
one place in `meshtastic_beacon`, with `-I examples/meshtastic_beacon`.

## Building

The Heltec boards have GPS UART pins in the base variant. Thus an external NMEA
GPS module is the only extra hardware. The T1000-E has a GPS on the board. The GPS
is **on** by default for these builds.

```
pio run -e heltec_v4_carnode          # Heltec V4     (ESP32-S3, SX1262)
pio run -e Heltec_t114_carnode        # Heltec T114   (nRF52840, SX1262)
pio run -e t1000e_dualmode_carnode    # Seeed T1000-E (nRF52840, LR1110)
```

The T1000-E build is the [dual-mode](../dualmode/) firmware. The build includes
`companion_radio` with the repeater. A 5-press of the button restarts the node in
the other mode. The car node operates in **repeater mode**. In companion mode, the
node is a standard MeshCore node with a phone. It has a GPS, a button, and a buzzer
on the board. No external hardware is necessary.

This is the V4 env. (The T114 env is the same, plus `-D ENV_INCLUDE_GPS=1`,
because the T114 repeater base does not enable GPS by default.)

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

To add another board, use the same shape on its `*_repeater` env. Set
`MT_HW_MODEL` to the Meshtastic HardwareModel of the board. Add
`-D ENV_INCLUDE_GPS=1` if the base does not. Make sure that the variant makes a GPS
`LocationProvider` and **passes `&rtc_clock`** — for example
`MicroNMEALocationProvider(Serial1, &rtc_clock)`. The clock pointer lets the GPS
set the time. The map-point and telemetry times come from the RTC. On a board with
no hardware RTC, that clock is wrong until the first fix sets it. (Also declare
`extern MomentaryButton user_btn;` if you want hold-to-hibernate.)

## Hibernate: hold the user button (near 3 s)

Hold the user button on the board for `CAR_NODE_HOLD_OFF_MILLIS` (default
**3000 ms**; change it with `-D`). The node then hibernates with
`board.powerOff()`:

- **T114** — system-off (`sd_power_system_off`). The GPS is off. The user button or
  reset wakes it.
- **T1000-E** — the same system-off with `T1000eBoard::powerOff()` (the GPS and the
  sensor power are off; the button SENSE wakes it). Repeater mode only — a 5-press
  still changes the mode; a hold of near 3 seconds hibernates. In companion mode,
  the standard companion button behaviour applies.
- **V4** — button-only deep sleep (`HeltecV4Board::hibernateButtonWake`). The node
  stays asleep during LoRa traffic. Only the user button wakes it. This button is
  GPIO0 (the BOOT pin). Thus the firmware waits for you to release it before it
  sleeps. To wake the node, a short **push** is best. If you hold GPIO0 down through
  the reset, the chip can go into serial-download mode and not start.

The function uses the shared `MomentaryButton user_btn` (it manages the button
polarity of each board). Thus it operates on any board that declares one. The OLED
shows "Hibernating..." before the board powers down.

## Runtime control

Use the serial port or an admin remote-CLI session. There are two command groups,
but one beacon engine (no second transmitter):

**`mtbeacon`** — the Meshtastic beacon (RF and appearance):

| Command | Function |
| --- | --- |
| `mtbeacon` / `mtbeacon status` | Show the beacon RF configuration and the node id. |
| `mtbeacon on` / `mtbeacon off` | Set the beacon to on / off. |
| `mtbeacon send` | Send a location update now (both networks). |
| `mtbeacon interval <min>` | The presence interval between parks (0–1440; 0 = park only; default 30). |
| `mtbeacon preset <name>` | The modem preset (LongFast, MediumFast, and others). |
| `mtbeacon region <name>` | The region and band (US, EU_868, and others). |
| `mtbeacon freq <MHz\|auto>` | Set the frequency manually. `auto` calculates it again. |
| `mtbeacon power <dBm>` | The TX power (−9 to 22). The region limit applies. |
| `mtbeacon hops <0-3>` | The Meshtastic hop limit for the presence and text (default 0 = direct neighbours only). |
| `mtbeacon text <string>` | The chat text (63 characters maximum). |
| `mtbeacon text.mult <N>` | The text schedule. The text goes with each flood advert, and N−1 more times each period (0 = never). |
| `mtbeacon short <str\|auto>` | The Meshtastic short name / map label (4 characters maximum). `auto` = `MC` and 2 hex characters. |
| `mtbeacon nodeinfo on/off` | Include NodeInfo (the node name). Default on. |
| `mtbeacon position on/off` | Include Position (the live map point). Default on. |
| `mtbeacon telemetry on/off` | Include Telemetry (battery and run time). Default on. |
| `mtbeacon stats` / `mtbeacon stats clear` | Show the transmit health from boot / Set the counters to zero. |
| `mtbeacon presets` / `mtbeacon regions` | Show the available values. |

**`carnode`** — the park behaviour:

| Command | Function |
| --- | --- |
| `carnode` / `carnode status` | Show the drive state (nofix/driving/parked/sleeping/*zone*) and the park configuration. |
| `carnode park <sec>` | The stopped time before an update (30–86400; default 300). |
| `carnode radius <m>` | Movement in this distance is "stopped" (5–2000; default 30). |
| `carnode advertdelay <sec>` | The time from the Meshtastic burst to the MeshCore advert (0–600; default 10). |
| `carnode sleep <hours>` | Parked this many hours → stop repeating until it drives again (0–720; 0 = never; default 20). |
| `carnode zone` | Show the quiet zones (up to 4). |
| `carnode zone add <name>` | Make a quiet zone at the current location. Park in it → repeat off immediately, radio quiet. |
| `carnode zone del <name>` | Delete that zone. |
| `carnode zone radius <name> <m>` | The zone radius (5–2000; default 100). |
| `carnode home [clear\|radius <m>]` | A short form for the zone with the name `home`. |

**`block`** — channels this repeater does not repeat (a standard repeater
function, in every build, not only for the car node):

| Command | Function |
| --- | --- |
| `block` / `block list` | Show the blocked channels and their hash byte. |
| `block #dispatches` | Stop the repeat of that `#hashtag` channel (up to 8). |
| `unblock #dispatches` | Start the repeat of it again. |

MeshCore `#hashtag` channels calculate their key from the name
(`SHA-256("#name")[:16]`). Thus the firmware calculates the on-air channel hash,
and you type the name only — no key is necessary. Two limits come from the wire
format. First, it operates for `#`-name channels only (the firmware cannot
calculate the key of a private channel with a random key). Second, the channel id
on the air is one byte, so a block sometimes also stops a different channel with
the same byte (near 1 in 256; the hash in the list makes a collision easy to see).
Traffic that is not a group message (adverts, DMs, ACKs) does not change.

**Park model.** The node monitors its GPS location. When the location moves more
than `radius` metres, the node is *driving* and sends no location. When the
location stays in `radius` for `park` seconds (default 5 minutes), the vehicle is
*parked* and the node sends one **update**. Two defenses make this reliable with a
GPS in a bad position (in the trunk, or behind metallized glass). First, the park
logic and the transmitted location use the **median of the last 9 locations**
(near 27 s), so one location that moves hundreds of metres is ignored. Second,
movement must then continue more than the radius for near 15 s before it is
driving. `carnode status` shows the live data (`d<n>m` = the distance of the
filtered location from the park point; `(raw)` while the filter starts; `<n>min` =
the time the vehicle has been stopped). If the node changes to *driving* while the
vehicle is stopped, increase `carnode radius`. The update:

1. The node sends a **Meshtastic** beacon (NodeInfo, Position, and text).
2. The node sends a **MeshCore** update. It writes the location to its `NodePrefs`
   and **sends a new advert**, so all MeshCore nodes know where you stopped.

The two updates use the *same* location at the *same* time. If you stop in the
same place again, the node does not transmit again (`radius` removes duplicates);
`mtbeacon send` sends an update immediately. The node keeps the MeshCore location
in flash, so it stays after a restart while parked.

The two parts are independent after they start. If the mesh cannot queue the flood
advert (the packet pool or the send queue is full while the repeater forwards a
burst), the car node tries the advert again a few times with a delay. It does not
lose the park. It does not change the periodic flood-advert timer, so the next
advert still includes the location. `carnode status` shows `adv!N` if the node
stopped a park advert fully — usually it is not shown.

**Periodic presence.** Between park events, the node sends a small presence (like
the base mtbeacon) each `interval` minutes (default 30, `0` = park only). This
keeps the node in the Meshtastic node lists. When the vehicle is parked, the
presence includes Position and refreshes the point. When the vehicle is driving,
or there is no location, the node sends NodeInfo only. Thus the map point does not
move to a location in the middle of a drive. The location updates *only* when the
vehicle parks.

**Chat text.** The `mtbeacon text` message is **event-driven**. When the repeater
sends a MeshCore advert (the park advert, the periodic flood advert, or a CLI
`advert`), the text goes out on a Meshtastic burst a few seconds later. Thus each
park at a new location sends its text, and the `advert` command is also a good
test. (`text.mult > 1` adds more texts between adverts; `text.mult 0` stops the
text fully.) `mtbeacon status` shows the schedule, for example `txt1x~47h(due 13h)`,
or `(due now)` when a text is ready for the next burst. In a quiet zone, the node
transmits nothing; a ready text waits until you drive away.

**Hop limit.** Meshtastic packets go out at `hops 0` by default. Direct neighbours
receive them; other nodes do not repeat them. A car node is mobile and transmits
from where it parks, so it is not good manners to send its location across another
person's mesh. `mtbeacon hops <0-3>` increases the limit if you need more range.
(This is the base beacon default since mtbeacon v0.2.0.)

**Map label.** Meshtastic shows a node's **short name** on the map marker. The car
node default is `MC` and 2 hex characters of its node id (for example `MC7a`). This
shows as MeshCore, and it is different from other beacons near it. Set your own
with `mtbeacon short <str>` (4 characters maximum, for example a callsign). Use
`mtbeacon short auto` to go back. The long name stays `MC <node name>`.
(mtbeacon v0.2.5 parity.)

**Transmit confirmation.** Each burst changes the radio to the Meshtastic PHY and
back. The TxDone interrupt (the "transmit complete" signal) can be lost during this
change. The airtime of the packet limits the wait, so a lost interrupt cannot stop
the loop. If the interrupt does not arrive, the node reads the **TxDone flag of the
radio chip** to find out if the packet transmitted. It does not assume that the
packet transmitted. If a park burst did not transmit, the node reports a failure
and sends it again. An undelivered chat text stays ready; the node does not mark it
as sent:

```
carnode: TxDone IRQ missed on 1/3 packet(s) - checked the chip instead
carnode: 1/3 packet(s) did NOT transmit (chip reports no TxDone)
```

After **3 dead transmits together**, the node does more than report it. It starts
the radio again — standby, clear the latched IRQ flags, arm the driver TxDone
interrupt again, program the MeshCore PHY again, and enter receive. A car node that
stopped transmitting is a vehicle that you cannot find. Before this function, a
stopped modem stayed stopped until the ignition cycled.

```
carnode: 3 dead transmits in a row - reinitialising the radio
```

`mtbeacon stats` shows the totals. Thus you can read a drive test after the drive,
and you do not read through the serial output:

```
mtbeacon stats
> carnode tx 61/64 ok | dead 3 irq-miss 7 nostart 0 | burst 22 lbt 1 | recov 1 (last fail 12m ago)
```

`dead` is part of `irq-miss`. `irq-miss` is each transmit with no interrupt. `dead`
is the part that the chip confirmed did not transmit. A high `irq-miss` with
`dead 0` is good — the interrupts are unreliable, but the packets transmit. The
counters are in RAM only (a restart sets them to zero). `mtbeacon stats clear` sets
them to zero when you want.

(mtbeacon v0.2.4 limited the wait; v0.2.5 added the hardware check; v0.2.6 added
the recovery and the counters.)

**Battery reporting.** Each park burst and each periodic presence includes a
Meshtastic **Telemetry** packet with the board battery percentage, the voltage, and
the run time. Thus a vehicle parked for three days shows its battery decrease in a
Meshtastic client. Use `mtbeacon telemetry off` to stop it. A board with no battery
sense reports nothing, not a wrong 0%.

> The MeshCore side uses `advert_loc_policy = prefs` (the repeater default), which
> this build keeps. The car node writes the location to prefs itself; it does not
> use the `gps advert share` policy. Thus MeshCore updates *only* when the vehicle
> parks, not on each periodic advert.

**Repeat sleep.** A vehicle parked for a day is usually in a place where no person
needs a mobile repeater. After the vehicle is stopped for `sleep` hours (default
20), the node stops **forwarding**. Remote status shows it as disabled, and it is
not in node-discover. The node still receives, answers its own commands, sends
adverts, and beacons. Forwarding starts again when the node detects movement (the
parked location moves more than `radius`). A GPS loss while parked (an underground
garage) does *not* wake it; only movement wakes it. `carnode sleep 0` stops the
function.

Suppression is a **runtime condition over** the `set repeat on|off` setting; it is
not a write to it. The firmware does not change your setting. `get repeat` always
shows your selection. No sleep state writes to flash. The node always starts awake
after a restart. `carnode status` shows `rpt:off` while suppression is active. This
is the one place where the setting and the actual state can be different.

> Before v0.2.7, the sleep wrote the setting directly and restored it on wake. Any
> other `set …` command writes the full prefs block. Thus a change to a setting
> while parked in a quiet zone saved the suppressed value to flash. After that, the
> wake function read it as "the operator wants this off" and did not start
> forwarding again. If a node has been quiet since then, use `set repeat on` one
> time (parked outside all zones). This corrects it.

**Quiet zones.** Park where the vehicle usually stays and use
`carnode zone add home`. The node stores the current location (median-filtered)
with that name, in `/carnode`. After this, when you park in the zone radius
(default 100 m), the node stops **forwarding** immediately — it does not wait for
the `sleep` timer. A place where you park every day already has fixed coverage; it
does not need a mobile repeater in the driveway. In a zone, the node is also
**radio-quiet**: the park burst, the MeshCore advert, *and* the periodic presence
are all off. Thus that location does not go on the air (a manual `carnode send`
still transmits if you ask). When you drive away, forwarding and the normal beacon
start again, like a wake from sleep. A lost fix in a zone (a garage) keeps you in
the zone; only a drive away clears it.

You can store up to **4 zones** — home, work, a frequent customer location — each
with its own radius. `carnode zone` lists them and marks the zone you are in with
`*`. `carnode status` and the OLED show the zone *name* as the state, so you can see
"which of my places is it at". Names are up to 8 characters. To move a zone to your
current location, run `carnode zone add <name>` again for that name; it keeps its
radius.

`carnode home`, `carnode home clear`, and `carnode home radius <m>` still operate —
they are a short form for the zone with the name `home`. Thus nothing that an
operator typed before v0.2.6 changed. A home from an older build **moves into the
zone table automatically** on the first boot, and it keeps its radius.

Defaults: US LongFast, 5-minute park / 30 m radius, 30-minute presence, 22 dBm
(region limit applies), 20-hour repeat sleep, no zones until you set one, off until
`mtbeacon on`. The OLED home screen shows `CarNode driving` / `parked` /
`sleeping` / `<zone name>`.

## Reliability (nRF52 boards)

On nRF52 builds (T114, T1000-E), the firmware starts the **hardware watchdog**
(90 s, fed each loop pass). If the node hangs, it restarts itself; it does not
continue dead until a person power-cycles it. DFU updates are not affected (the UF2
bootloader feeds a running watchdog). The dual-mode build feeds it across the
repeater/companion mode change.

Each boot prints its **reset reason** to serial, and you can query it remotely:

```
get pwrmgt.bootreason
> Reset: Watchdog
```

`Watchdog` there means the node hung and recovered itself — send a bug report.

## Scope and etiquette

The same as the beacon: a one-way presence only (it does not route or repeat
Meshtastic traffic), on shared public spectrum. Obey the limits of your region. Do
not beacon onto channels that you do not operate. A park sends a MeshCore advert to
all nodes, so keep `park` at a sensible value; thus frequent short stops do not
fill the mesh.
