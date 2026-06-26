# ESP-Brookesia Phone Example for Waveshare ESP32-S3 Touch AMOLED 2.16

The example demonstrates how to use `esp-brookesia` directly with the Waveshare CO5300 AMOLED display and CST9220 touch controller. It does not use `ESP32_Display_Panel`.

The example is suitable for touchscreens with a resolution of `240 x 240` or higher. Otherwise, the functionality may not work properly. The default style ensures compatibility across resolutions, but the display effect may not be optimal on many resolutions. It is recommended to use a UI stylesheet with the same resolution as the screen. If no suitable stylesheet is available, it needs to be adjusted manually.

## How to Use

To use this example, please firstly install the following dependent libraries:

- GFX Library for Arduino
- SensorLib
- lvgl (>= v9.0, < v10)

Then, follow the steps below to configure the libraries and upload the example:

1. For **esp-brookesia**:

    - [optional] Follow the [steps](../../../docs/how_to_use.md#configuration-instructions-1) to configure the library.

2. For **lvgl**:

    - [mandatory] Enable the `LV_USE_SNAPSHOT` macro in the *lv_conf.h* file.
    - [optional] Modify the macros in the [lvgl_port_waveshare.h](./lvgl_port_waveshare.h) file to configure the lvgl porting parameters.

3. Navigate to the `Tools` menu in the Arduino IDE to choose ESP32-S3 and configure its parameters:
    - `Flash Size`: `16MB`
    - `Partition Scheme`: `Custom`
    - `PSRAM`: `OPI PSRAM`
    - `USB Mode`: `Hardware CDC and JTAG`
    - `USB CDC On Boot`: either `Enabled` or `Disabled`

    This sketch includes a `partitions.csv` with a large factory app slot because the full Phone example plus Arduino_GFX and SensorLib is larger than Arduino-ESP32's 3 MB `huge_app` slot. Do not use `QSPI PSRAM`; the Waveshare ESP32-S3 Touch AMOLED 2.16 uses OPI PSRAM, and the wrong mode can reboot immediately with `quad_psram: PSRAM chip is not connected, or wrong PSRAM line mode`.
4. Verify and upload the example to the ESP board.

Do not copy the library `src` directory into this sketch folder. Install or link the whole `esp-brookesia-arduino-lvgl9` repository as an Arduino library under `Documents/Arduino/libraries`; otherwise Arduino may compile two copies of Brookesia and fail with class redefinition errors.

## Technical Support and Feedback

Please use the following feedback channels:

- For technical queries, go to the [esp32.com](https://esp32.com/viewforum.php?f=22) forum.
- For a feature request or bug report, create a [GitHub issue](https://github.com/espressif/esp-brookesia/issues).

We will get back to you as soon as possible.
