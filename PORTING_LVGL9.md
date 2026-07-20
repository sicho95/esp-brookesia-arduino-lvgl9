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
- Runtime validation has been completed on the Waveshare target for app launch, app close, recents carousel, status bar updates, home/recents gestures, close by upward swipe, close by trash, and automatic return to the launcher when the last app is closed.
- The 480 x 480 Waveshare path uses icon-based recents instead of saved live snapshots on Arduino to avoid unstable 480 x 480 snapshot allocation on this target.
- The Waveshare path also documents the display render-mode tradeoff explicitly: `LV_DISPLAY_RENDER_MODE_FULL` is safer on boards that need software rotation or show partial-refresh artifacts, while `PARTIAL` remains preferable on boards that render correctly with it.
- The Waveshare example now includes QMI8658 automatic rotation, with a
  rotation-lock control in the control centre. All four orientations have been
  validated on the physical board.
- The example also includes board services for notifications, scheduled jobs,
  NTP, physical keys, display-idle policy, display leases, AXP2101 charging and
  status-bar indicators. See
  `examples/arduino/Phone_Waveshare_CO5300_CST9220/SYSTEM_SERVICES.md`.

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
4. Validate app launch, close, recents, status bar and navigation gestures. Done on the Waveshare CO5300/CST9220 Arduino example.
5. Tune runtime stability, memory pressure and touch/display rotation on real hardware. Done for the current Waveshare baseline, including QMI8658 automatic rotation in all four orientations.

## Waveshare CO5300 rotation architecture

The validated Arduino path deliberately uses one software-owned coordinate
pipeline:

1. `Arduino_CO5300` is always kept at `setRotation(0)`.
2. The QMI8658 chooses a logical LVGL orientation only after stable samples.
3. The LVGL flush callback rotates pixels into a dedicated PSRAM buffer.
4. CST9220 raw touch points are mapped with the inverse transform.
5. A rotation change resets the active input state, invalidates the complete
   display and marks LVGL's refresh timer ready. It never calls a synchronous
   `lv_refr_now()` from the rotation path.

Do not combine this pipeline with `lcd->setRotation(1/2/3)`. The Arduino_GFX
CO5300 driver explicitly has no true 90/270-degree panel rotation; its MADCTL
settings are flips. Composing those flips with the software transform caused a
mirrored 180-degree view and made one quarter-turn unreliable.

Two performance details are mandatory with the 480 x 480 `FULL` framebuffer:

- Iterate the rotated destination by columns so writes into the PSRAM rotation
  buffer remain contiguous. Strided destination writes made a full transpose
  visibly scan down the screen while the LVGL task was occupied.
- Call the RAM/non-const `Arduino_GFX::draw16bitRGBBitmap()` overload. The
  `const uint16_t *` overload writes pixels individually, whereas the non-const
  overload reaches the contiguous `_bus->writePixels()` QSPI transfer. The
  port uses a small `draw_bitmap()` wrapper with a documented `const_cast`;
  Arduino_GFX does not modify the source buffer.

`LV_DISPLAY_RENDER_MODE_FULL` remains intentional for this board. Earlier
`PARTIAL` tests were faster and smaller but produced incomplete SquareLine
redraws after widget changes. This is a board-profile choice through
`LVGL_PORT_RENDER_MODE`, not a requirement imposed on every Brookesia target.

## Non-Goals

- Do not migrate this fork to ESP-IDF-only packaging.
- Do not remove the original Arduino examples.
- Do not claim runtime compatibility for every board just because the Waveshare Arduino example is validated.
