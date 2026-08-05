# T1000-E dual-mode build

This is one firmware that contains both `companion_radio` and `simple_repeater`.
It selects which one to run at boot. A 5-press of the user button changes a saved
flag and restarts the node in the other mode.

## How it compiles together

The build links three `main.cpp` files together:

- `examples/dualmode/main.cpp` has the real Arduino `setup()` and `loop()`. It
  reads the mode flag from `/dualmode` and calls `rpt_*` or `cmp_*`.
- `examples/simple_repeater/main.cpp` — with `-D DUALMODE`, its entry points change
  to `rpt_setup()` and `rpt_loop()` (see the end of that file). `dualmode_rename.h`
  (included first) changes its `MyMesh` and `UITask` classes to `RptMesh` and
  `RptUITask`, so they do not collide with the companion classes.
- `examples/companion_radio/main.cpp` — with `-D DUALMODE`, its entry points change
  to `cmp_setup()` and `cmp_loop()`. It keeps the `MyMesh` and `UITask` names.

The build removes the repeater UI in dual-mode (`#if defined(DISPLAY_CLASS) &&
!defined(DUALMODE)`). Thus only the companion display path is active. The T1000-E
has no screen, so there is no UI in either mode.

## PlatformIO env

The env is the union of the companion-BLE build and the repeater build, plus the
launcher:

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

In the `+mtbeacon` build, the beacon is active in Repeater mode only. Configure it
over the repeater serial CLI (`mtbeacon ...`). See the beacon project for the
add-on source and the full documentation.

There is also `[env:t1000e_dualmode_carnode]`. Here the repeater side is the
[car node](../car_node/): a park-triggered live-GPS Meshtastic beacon, a periodic
presence, repeat sleep, and hold-to-hibernate (near 3 seconds; a 5-press still
changes the mode). The GPS on the board is on by default in repeater mode.
