# T1000-E dual-mode build

One firmware that contains both `companion_radio` and `simple_repeater` and
picks which to run at boot. A 5x user-button press toggles a persisted flag and
reboots into the other mode.

## How it co-compiles

Three `main.cpp` files are linked together:

- `examples/dualmode/main.cpp` owns the real Arduino `setup()` / `loop()`. It
  reads the mode flag from `/dualmode` and dispatches to `rpt_*` or `cmp_*`.
- `examples/simple_repeater/main.cpp` — under `-D DUALMODE` its entry points are
  renamed to `rpt_setup()` / `rpt_loop()` (see the bottom of that file), and
  `dualmode_rename.h` (included first) renames its `MyMesh`/`UITask` classes to
  `RptMesh`/`RptUITask` so they don't collide with the companion's.
- `examples/companion_radio/main.cpp` — under `-D DUALMODE` its entry points
  become `cmp_setup()` / `cmp_loop()`; it keeps the `MyMesh`/`UITask` names.

The repeater UI is compiled out in dual-mode (`#if defined(DISPLAY_CLASS) &&
!defined(DUALMODE)`), so only the companion's display path is active. On the
screenless T1000-E that means no UI either way.

## PlatformIO env

The env is the union of the companion-BLE and repeater builds plus the launcher:

```ini
[env:t1000e_dualmode]
extends = t1000-e
board_build.ldscript = boards/nrf52840_s140_v7_extrafs.ld
board_upload.maximum_size = 708608
build_flags = ${t1000-e.build_flags}
  -D DUALMODE
  -I examples/companion_radio/ui-orig
  ; companion side
  -D MAX_CONTACTS=350
  -D MAX_GROUP_CHANNELS=40
  -D BLE_PIN_CODE=123456
  -D BLE_TX_POWER=0
  -D OFFLINE_QUEUE_SIZE=256
  -D DISPLAY_CLASS=NullDisplayDriver
  -D PIN_BUZZER=25
  -D PIN_BUZZER_EN=37
  ; repeater side
  -D ADMIN_PASSWORD='"password"'
  -D MAX_NEIGHBOURS=50
  ; shared
  -D ADVERT_NAME='"t1000-e DualMode"'
  -D ADVERT_LAT=0.0
  -D ADVERT_LON=0.0
build_src_filter = ${t1000-e.build_src_filter}
  +<helpers/nrf52/SerialBLEInterface.cpp>
  +<helpers/ui/buzzer.cpp>
  +<../examples/companion_radio/*.cpp>
  +<../examples/companion_radio/ui-orig/*.cpp>
  +<../examples/simple_repeater/*.cpp>
  +<../examples/dualmode/*.cpp>
lib_deps = ${t1000-e.lib_deps}
  densaugeo/base64 @ ~1.4.0
  stevemarple/MicroNMEA @ ^2.0.6
  end2endzone/NonBlockingRTTTL@^1.3.0

; repeater side = the Meshtastic-beacon variant
[env:t1000e_dualmode_mtbeacon]
extends = env:t1000e_dualmode
build_flags = ${env:t1000e_dualmode.build_flags}
  -D WITH_MT_BEACON
  -I examples/meshtastic_beacon
lib_deps = ${env:t1000e_dualmode.lib_deps}
```

In the `+mtbeacon` build the beacon is active only in Repeater mode; configure it
over the repeater's serial CLI (`mtbeacon ...`). See the beacon project for the
add-on source and full docs.
