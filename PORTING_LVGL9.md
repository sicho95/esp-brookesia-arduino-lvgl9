# LVGL 9 Porting Plan

This fork starts from ESP-Brookesia `0.4.2`, because that release still ships as an Arduino library. The port should borrow LVGL 9-compatible changes from ESP-Brookesia `0.5.0` while preserving Arduino IDE packaging.

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

1. Compile the library headers and core sources with LVGL 9.
2. Compile the Arduino `Phone` example with LVGL 9.
3. Run a minimal ESP32-S3 display/touch example.
4. Validate app launch, close, recents, status bar and navigation gestures.
5. Add a Waveshare ESP32-S3 Touch AMOLED 2.16 example if the generic Phone example is stable.

## Non-Goals

- Do not migrate this fork to ESP-IDF-only packaging.
- Do not remove the original Arduino examples.
- Do not claim compatibility until at least one Arduino LVGL 9 example compiles.
