# MeshCore-dualmode

A fork of [MeshCore](https://github.com/meshcore-dev/MeshCore) that adds a
**dual-mode build for the Seeed T1000-E**: one firmware that is *both* a BLE
Companion and a Repeater, and switches between them with a **five-press of the
user button**. Optionally, the repeater side also runs the
[Meshtastic beacon](examples/meshtastic_beacon/).

> The launcher lives in [`examples/dualmode/`](examples/dualmode/). Everything
> else is unmodified MeshCore plus the bundled beacon add-on — see the
> [upstream project](https://github.com/meshcore-dev/MeshCore) for the base firmware.

## What it does

Normally a T1000-E runs one role. This build contains both `companion_radio` and
`simple_repeater` and chooses which to run at boot from a flag in flash:

- **Companion mode** (default) — BLE companion radio, paired from the MeshCore app.
- **Repeater mode** — a standard MeshCore repeater with serial CLI.

Press the user button **5×** to toggle: the buzzer confirms (2 beeps → Repeater,
1 beep → Companion) and the device reboots into the other mode. The two apps'
identically-named classes coexist because the repeater's are renamed
(`MyMesh→RptMesh`, `UITask→RptUITask`) when built with `-D DUALMODE`.

See [`examples/dualmode/README.md`](examples/dualmode/README.md) for how it
co-compiles.

## Firmware

Two images are attached to the [latest release](../../releases/latest):

- **`t1000e-dualmode.zip`** — the plain Companion/Repeater switcher
- **`t1000e-dualmode-mtbeacon.zip`** — same, but the **repeater side runs the
  Meshtastic beacon** (appears on a Meshtastic map; configured via the
  repeater-mode serial CLI — see the [beacon docs](examples/meshtastic_beacon/README.md))

## Build it yourself

```
pio run -e t1000e_dualmode            # Companion + Repeater
pio run -e t1000e_dualmode_mtbeacon   # ...repeater side also beacons onto Meshtastic
```

## License

MIT, same as upstream MeshCore (see [`license.txt`](license.txt)). Not affiliated
with the MeshCore or Meshtastic projects.

## Built with generative AI

The dual-mode launcher, the bundled beacon add-on, and this fork's docs were
developed with the assistance of generative AI (Anthropic's Claude), then
reviewed and tested by a human. Read the source, verify behavior, and test
before relying on it.
