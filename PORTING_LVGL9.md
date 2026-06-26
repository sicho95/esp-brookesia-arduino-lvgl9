# LVGL 9 Porting Plan

This fork starts from ESP-Brookesia `0.4.2`, because that release still ships as an Arduino library. The port borrows LVGL 9-compatible changes from ESP-Brookesia `0.5.0` while preserving Arduino IDE packaging.

## Current State

- Imported ESP-Brookesia `0.5.0` `src/`, `docs/`, `test_apps/`, `tools/` and ESP-IDF examples.
- Preserved Arduino library metadata in `library.properties`.
- Ported the Arduino `examples/arduino/Phone` sketch to a minimal LVGL 9 display/input port.
- The Arduino `Phone` example compiles with Arduino ESP32 core `3.3.8`, LVGL `9.5.0`, and `PartitionScheme=huge_app`.
- Added `examples/arduino/Phone_Waveshare_CO5300_CST9220`, a hardware-specific Arduino example for the Waveshare ESP32-S3 Touch AMOLED 2.16 using Arduino_GFX for CO5300 and SensorLib for CST9220.
- The Waveshare example compiles with Arduino ESP32 core `3.3.8`, LVGL `9.5.0`, GFX Library for Arduino `1.6.5`, SensorLib `0.4.1`, and `PartitionScheme=custom`.
- The Waveshare example includes its own `partitions.csv`; the full Phone demo plus Arduino_GFX and SensorLib exceeds Arduino-ESP32's 3 MB `huge_app` slot.
- Runtime validation on real hardware is still pending.

## Constraints

- Keep `library.properties` and Arduino examples usable.
- Target `lvgl >= 9.0 && < 10`.
- Avoid introducing ESP-IDF-only dependencies into the Arduino build.
- Keep the original Apache-2.0 license.

## Main API Areas

- Replace LVGL 8 display types and APIs:
  - `lv_disp_t`
  - `lv_disp_drv_t`
  - `lv_disp_draw_buf_t`
  - `lv_disp_get_*`
- Replace LVGL 8 input-device driver APIs:
  - `lv_indev_drv_t`
  - `lv_indev_drv_init`
  - `lv_indev_drv_register`
- Replace renamed object APIs:
  - `lv_obj_del` -> `lv_obj_delete`
  - `lv_scr_act` / screen APIs where LVGL 9 equivalents are required
- Recheck snapshots, animations and timer cleanup against LVGL 9 internals.
- Recheck `lv_conf.h` requirements for Arduino examples.

## Milestones

1. Compile the library headers and core sources with LVGL 9. Done.
2. Compile the Arduino `Phone` example with LVGL 9. Done.
3. Compile a Waveshare ESP32-S3 Touch AMOLED 2.16 display/touch example. Done.
4. Validate app launch, close, recents, status bar and navigation gestures.
5. Tune runtime stability, memory pressure and touch/display rotation on real hardware.

## Non-Goals

- Do not migrate this fork to ESP-IDF-only packaging.
- Do not remove the original Arduino examples.
- Do not claim runtime compatibility until at least one Arduino LVGL 9 example runs on real hardware.
