# meshtastic_beacon (prototype)

Make a MeshCore repeater periodically announce itself **on a Meshtastic network**,
so Meshtastic users see a text line like `MeshCore repeater in range` on their
default public channel.

## Why a native advert won't work

A MeshCore advert and a Meshtastic packet share nothing above the raw radio:

| Layer | MeshCore | Meshtastic LongFast |
| --- | --- | --- |
| Frequency | region default (e.g. 869.618 MHz) | region slot (e.g. 906.875 MHz US) |
| Bandwidth / SF / CR | 62.5 kHz / 8 / 4-5 | 250 kHz / 11 / 4-5 |
| **Sync word** | **0x12 (private)** | **0x2B (public)** |
| Framing | pubkey + timestamp + signature | 16-byte header + AES-CTR protobuf |

Even on the same frequency the differing **sync word** means the radios never
hear each other; and even if they did, Meshtastic drops anything that isn't its
own framing. So to be seen we must *become* a Meshtastic transmitter for one
packet.

## What this prototype does

`MeshtasticBeacon.h` is self-contained and chip-agnostic:

1. `buildTextPacket()` — assembles a Meshtastic broadcast text packet: the
   16-byte header, then an AES-CTR–encrypted `Data{portnum=TEXT_MESSAGE_APP,
   payload="..."}` protobuf, keyed by the channel PSK. Uses the rweather
   `Crypto` lib (`CTR<AES128>`) already vendored by MeshCore.
2. `channelHash()` — Meshtastic's XOR channel hash (default channel == `0x08`).
3. `transmitOnce()` — retunes the radio to the Meshtastic modem preset + sync
   word via MeshCore's `RadioLibWrapper`, transmits, then restores the MeshCore
   PHY so the repeater resumes normally.

`main.cpp` is a minimal demo: boot the board/radio, then beacon every 5 minutes.

## The single-radio constraint

LoRa is one channel at a time. While `transmitOnce()` runs (a few hundred ms at
SF11), the radio is **off the MeshCore channel** and any MeshCore traffic in
that window is missed. This is the core trade-off — keep the interval long
(minutes). For a real repeater, call `sendBeacon()` from the idle path of
`simple_repeater` rather than running this stripped-down loop.

## Status

**Verified on-air.** The text packet has been confirmed against a live Meshtastic
node — it decodes and displays on the default channel. The interop-critical math
(channel-hash `0x08`, the region/preset frequency calculation, the AES-CTR nonce,
and the NodeInfo/Position protobuf encoders) is pinned by host unit tests in
`test/` (run `test/run.sh`). The multi-block AES-CTR counter is implicitly
confirmed: the default text is >16 bytes and decodes correctly.

Still worth confirming on your own bench: that the **NodeInfo/Position** packets
render as a named pin on the Meshtastic map (the bytes are validated, but map
display hasn't been eyeballed) — and, obviously, that you've set the right
`region`/`preset` for where you are.

## Building it

Beacon envs are defined in the variant `platformio.ini` files. Verified builds:

```
pio run -e t1000e_meshtastic_beacon         # Seeed T1000-E  (LR1110)
pio run -e Heltec_t114_meshtastic_beacon    # Heltec T114    (SX1262)
pio run -e Xiao_nrf52_meshtastic_beacon     # Seeed XIAO nRF52840 (SX1262)
```

Each env is the same shape — extend the board's base block and add this folder
to the source filter:

```ini
[env:<board>_meshtastic_beacon]
extends = <board base>
build_flags = ${<board base>.build_flags}
  -I examples/meshtastic_beacon
build_src_filter = ${<board base>.build_src_filter}
  +<../examples/meshtastic_beacon>
```

The radio helpers are templated on the concrete radio, so the same code binds to
`CustomLR1110` (T1000-E) or `CustomSX1262` (T114 / XIAO) unchanged. On the
repeater build, region/preset/frequency are runtime `mtbeacon` settings — no
per-board source edits needed (only `MESHCORE_SYNC_WORD` if a board's MeshCore
sync word differs from the default `0x12`).

## Runtime control via the MeshCore CLI (repeater integration)

`MtBeaconControl.h` adds a persisted, CLI-configurable beacon to a real repeater
(not the standalone demo). It's integrated into `simple_repeater` behind
`-D WITH_MT_BEACON`, so stock repeater builds are unaffected (the guards compile
to nothing). Build the integrated firmware with:

```
pio run -e Heltec_t114_without_display_repeater_mtbeacon
```

Then, over serial or via an admin remote-CLI session, use the `mtbeacon` verbs:

| Command | Effect |
| --- | --- |
| `mtbeacon` / `mtbeacon status` | show current config + derived node id |
| `mtbeacon stats` / `mtbeacon stats clear` | transmit health since boot / reset the counters |
| `mtbeacon help` | list all commands (detailed list prints to serial) |
| `mtbeacon on` / `mtbeacon off` | enable / disable periodic beaconing |
| `mtbeacon send` | transmit one beacon now, text included (works even when off) |
| `mtbeacon interval <min>` | presence period (1–1440 min, default 30) |
| `mtbeacon preset <name>` | modem preset: LongFast, LongMod, LongSlow, MediumFast, MediumSlow, ShortFast, ShortSlow, ShortTurbo |
| `mtbeacon region <name>` | region/country band: US, EU_868, EU_433, ANZ, CN, JP, KR, TW, RU, IN, NZ_865, TH, UA_433, UA_868 (alias `country`) |
| `mtbeacon freq <MHz\|auto>` | manual frequency override; `auto` re-derives from region+preset |
| `mtbeacon power <dBm>` | set TX power (−9…22), auto-capped to the region limit |
| `mtbeacon hops <0-3>` | Meshtastic hop limit for presence + text (**default 0**) |
| `mtbeacon text <string>` | set the announced text (≤63 chars) |
| `mtbeacon text.mult <N>` | text pacing: rides each flood advert, plus N−1 timer-paced extras per period (0 = never) |
| `mtbeacon short <str\|auto>` | Meshtastic short name / map label (≤4 chars); `auto` = `MC` + 2 hex |
| `mtbeacon nodeinfo on/off` | also emit a NodeInfo (named node) — default on |
| `mtbeacon position on/off` | also emit a Position (map pin) — default on |
| `mtbeacon telemetry on/off` | also emit Telemetry (battery + uptime) — default on |
| `mtbeacon presets` / `mtbeacon regions` | list available preset / region names |

**Shows up as a node on the Meshtastic map.** The periodic beacon is a silent
*presence*: a **NodeInfo** (so the repeater appears as a named node — long name
`MC <repeater name>`, short name `MC` + 2 hex of the node id, e.g. `MC7a`), a
**Position** (a map pin, using the repeater's configured lat/lon; skipped if
unset), and a **Telemetry** packet (battery + uptime). At slow presets these
go out one per cycle, round-robin, to bound the off-channel window; only the
enabled ones take part, so switching one off speeds up the rest of the cycle.
Toggle them with `nodeinfo` / `position` / `telemetry`.

**Battery and uptime (v0.2.6).** The Telemetry packet carries Meshtastic
`DeviceMetrics`, which is what phone clients render as the node's battery ring
and uptime — so a solar or battery repeater shows its state of charge on someone
else's app without any extra hardware. The percentage is mapped from the board's
raw millivolt reading across the usual 1S Li-ion range (3.3 V empty → 4.2 V
full); boards with no battery sense report nothing at all rather than a
confident 0%. `channel_utilization` and `air_util_tx` are deliberately **left
empty**: our airtime figures describe the MeshCore channel this node actually
repeats on, not the Meshtastic one, and publishing them would put a confidently
wrong load number on someone else's network map.

The **short name is the label Meshtastic draws on the map marker**, which is why
it isn't a flat `MC` — that would make every beacon an indistinguishable pin.
The 2 hex digits give 256-way local uniqueness while still reading as MeshCore
at a glance; the long name disambiguates on tap. Operators with a callsign or
site code can pin their own with `mtbeacon short <str>` (≤4 chars), or return to
the derived one with `mtbeacon short auto`.

**Chat text.** The `text` message is deliberately rarer than the presence, and
it's **event-driven**: whenever the repeater floods a MeshCore advert — the
periodic flood advert or an explicit CLI `advert` — the text rides a beacon
burst a few seconds later. So the chat announcement tracks the repeater's real
advert cadence (default 47 h), and running `advert` doubles as a live test.
`text.mult > 1` adds timer-paced extra texts between adverts (N per advert
period); `text.mult 0` disables the text, leaving a silent presence-only
beacon. `mtbeacon status` shows the pacing live, e.g. `txt1x~47h(due 13h)` —
or `(due now)` when a text is armed and waiting for the next burst;
`txt1x(noadv)` means the repeater's flood advert is off, which disables the
timer pacing (an explicit `advert` still carries the text).

**Region + preset auto-tune the frequency.** Setting `region` or `preset`
computes the exact Meshtastic default-channel frequency the same way Meshtastic
firmware does (the preset bandwidth sets how many channel slots fit in the region
band; the channel name hashes to a slot — e.g. US LongFast → 906.875, EU_868
LongFast → 869.525). `mtbeacon freq` is only for a manual override; a `*` in
`status` indicates an override is active.

**Region power cap.** Each region carries a legal TX-power ceiling (e.g. EU_433
12 dBm, JP 16 dBm); the configured power is clamped to it, shown as `(cap)` in
`status`.

**Hop limit.** The beacon's Meshtastic packets go out at **0 hops by default**, so
they're heard only by direct neighbors and are **never rebroadcast** across the
Meshtastic network — a deliberately light-footprint default for a foreign beacon.
Raise it with `mtbeacon hops <0-3>` (capped at 3) if you want limited relaying.
Shown as `h<N>` in `status`.

Settings persist to `/mtbeacon` in the internal filesystem (separate from
`NodePrefs` — core prefs are untouched). Defaults are US LongFast, 30-min
presence, text once per flood advert, disabled until you `mtbeacon on`. The
node number is derived from the repeater's identity, so it's stable across
reboots.

The beacon only fires when the mesh is idle (`!hasPendingWork() && !receiving`
&& not mid-transmit), deferring a few seconds otherwise, so it never retunes
mid-transaction. It restores the repeater's *current* radio params afterward
(including any active `tempradio` override) and re-arms receive immediately, so
power-saving nodes don't sleep deaf. To add it to another board, copy the
`*_repeater_mtbeacon` env shape: extend that board's repeater env and add
`-D WITH_MT_BEACON -I examples/meshtastic_beacon`.

## Reliability

Each burst retunes the radio to the Meshtastic PHY and back. Completion of a
transmit is normally signalled by the radio's **TxDone interrupt** — but that
edge can occasionally be lost across a retune, even though the packet went out
perfectly well. Rather than sit on a fixed multi-second timeout (which freezes
the whole main loop while the node looks dead), the send is bounded by the
packet's **estimated airtime**: once that has elapsed, a transmit that started
has physically finished. Normal sends are unaffected — the interrupt arrives
first and the wait ends immediately.

When the interrupt *doesn't* arrive, the beacon **asks the radio chip directly**
whether it transmitted (v0.2.5). MeshCore's `isSendComplete()` only reads a flag
an ISR sets, so from there "sent, interrupt lost" and "never transmitted" are
indistinguishable — v0.2.4 assumed the former and so could report a genuine
failure as a success. RadioLib maps every family's TxDone bit onto one generic
flag, so a single register read settles it on SX126x, SX127x, LR11x0 and
STM32WLx alike. The read happens *before* `finishTransmit()`, which clears the
flags.

The two conditions are now reported separately, so a misbehaving ISR and a real
transmit failure no longer look the same:

```
beacon: TxDone IRQ missed on 1/2 packet(s) - checked the chip instead
beacon: 1/2 packet(s) did NOT transmit (chip reports no TxDone)
```

A burst reports failure only when nothing left the antenna — the caller then
retries it rather than recording a delivery that never happened.

### Self-repair and `mtbeacon stats` (v0.2.6)

Knowing a transmit is genuinely dead is only useful if something acts on it.
After **3 dead transmits in a row** (`MT_BEACON_FAIL_STREAK`, overridable by
`-D`) the beacon **re-initialises the radio** instead of merely restoring it:
standby, clear any latched IRQ flags, re-arm the driver — which re-registers the
TxDone interrupt action, the thing most likely to have been lost across a retune
— then reprogram the MeshCore PHY and re-enter receive. Before this, a wedged
modem stayed wedged until somebody power-cycled the node.

It deliberately stops short of a chip reset: that needs per-family `begin()`
parameters the wrapper doesn't expose, and a half-reset radio is worse than the
wedge being cleared. A recovery restarts noise-floor calibration, so the
interference threshold is re-learned over the next few seconds.

```
beacon: 3 dead transmits in a row - reinitialising the radio
```

The counters behind all of this are readable at any time, which turns a drive
test or a site visit into data instead of serial scrollback:

```
mtbeacon stats
> beacon tx 61/64 ok | dead 3 irq-miss 7 nostart 0 | burst 22 lbt 1 | recov 1 (last fail 12m ago)
```

Read `dead` and `irq-miss` as **nested, not disjoint**: `irq-miss` counts every
transmit where the TxDone interrupt never arrived, and `dead` is the subset the
chip then confirmed had not gone out. A high `irq-miss` with `dead 0` is the
healthy case — the interrupt path is flaky but the packets are flying, which is
exactly the situation v0.2.4 could not distinguish from failure. `lbt` counts
bursts skipped because listen-before-talk found the channel busy; `recov` counts
radio re-inits. The counters live in RAM only — they are diagnostics, not worth
a flash write per packet, and a reboot is exactly when you want them to reset.
`mtbeacon stats clear` zeroes them without rebooting.

### nRF52 boards

nRF52 builds (RAK4631, ProMicro/Faketec, T114, T1000-E, XIAO nRF52, …) arm the
**hardware watchdog** (90 s, fed every loop pass): if the node ever hard-hangs,
it reboots itself instead of sitting dead until someone power-cycles it. DFU
updates are unaffected (the UF2 bootloader feeds a running watchdog).

Every boot prints its **reset reason** to serial, and it's remotely queryable
on any nRF52 board:

```
get pwrmgt.bootreason
> Reset: Watchdog
```

`Watchdog` there means the node hung and rescued itself — worth a bug report.

## Scope / etiquette

- **One-way only.** It appears as a named node + map pin and posts text, but it
  does not route Meshtastic traffic or rebroadcast.
- **Shared spectrum.** You're adding airtime to someone else's public channel.
  Beacons use listen-before-talk, interval jitter, and EU duty-cycle hold, but
  keep the interval long, respect your region's limits, and don't
  beacon onto channels you don't operate. Private channels need the operator's
  PSK.
