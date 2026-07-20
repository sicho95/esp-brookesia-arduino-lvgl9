# Waveshare system services

This example contains a board-specific services layer in `waveshare_system.cpp`.
It is intentionally kept out of the generic Brookesia core: the CO5300 display,
AXP2101 PMU, physical keys and their safe wake sources belong to this board port.

## Test now

- Swipe down from the top edge to open the control centre. Use its controls,
  notifications and settings pages; swipe upward to close it. While open, Brookesia
  launcher/navigation gestures are blocked so sliders cannot move the screen
  underneath.
- Toggle Wi-Fi, BLE, airplane mode and rotation-lock preference, then change brightness.
  Values are stored in NVS and survive restart.
  Airplane mode stops Wi-Fi globally and makes BLE unavailable to application
  transports through `waveshare_system_ble_is_allowed()`. BLE applications own
  the clean start/stop lifecycle of their NimBLE or Bluedroid host stack.
- With rotation unlocked, turn the device. The QMI8658 changes the software
  display/touch orientation only after four stable samples. The initial upright
  orientation remains the validated 270 degree CO5300 port orientation.
- Leave the device untouched for one minute. The display turns off; the next
  touch or PWR press wakes it. A foreground application may call
  `waveshare_system_acquire_display_lease("gps")` while it has information that
  must remain visible, and releases it when that condition stops.
- On the home screen, short `+` selects the next icon and short `-` selects the
  previous icon. The selected icon and its title scale to 115 percent without
  changing the launcher grid geometry. Long `+` starts it.
  Long `-` returns an active app to Home.
- The example schedules a background job at boot and every five minutes. It
  posts a notification in the control centre; tapping it starts or resumes the
  `Simple Conf` app. Notifications can be removed individually with their `X`
  or cleared together from the notifications page.
- When Wi-Fi is connected by an application, the service synchronises system
  time using NTP and the selected timezone. Europe/Paris remains the default.
  NTP can be disabled and date/time can be set manually from settings.
- Short PWR moves an active device to connected standby and wakes it from a
  screen-off state. Long PWR saves settings and requests an AXP2101 shutdown.

## Settings and audio

The third control-centre page persists these choices in NVS:

- screen timeout from 30 seconds to 10 minutes, plus an always-on mode;
- timezone, automatic NTP, and manual date/time;
- safe battery capacity/current presets and the 4.1 V battery-care target;
- dual-microphone enable/mute, ES7210 hardware gain up to 37.5 dB, and ES8311
  speaker volume. AEC and noise reduction are always enabled while the
  microphones are active.

`waveshare_audio.hpp` is the board audio API. It initializes the shared 16 kHz,
16-bit I2S bus, powers the ES7210 input channels and ES8311 output path, and
provides bounded `read()`/`write()` functions. Muting the microphones powers
down their ES7210 channels rather than merely discarding samples.

`waveshare_audio_read()` returns mono 16-bit/16 kHz audio processed by the
Espressif AFE bundled with Arduino-ESP32 3.3.8. Its `MMR` pipeline receives both
microphones plus a reference copied from `waveshare_audio_write()`, and keeps
AEC, dual-microphone speech enhancement, noise suppression, VAD and conservative
AGC active. Applications must send stereo 16-bit/16 kHz playback through
`waveshare_audio_write()` so AEC receives the exact speaker reference. The
ES7210 is the multichannel ADC; the effective DSP runs on ESP32-S3 core 0 and
uses PSRAM rather than being a hidden ES7210 register.

## Application API

- `waveshare_system_post_notification()` copies a notification into a bounded
  six-entry queue. An app ID makes the notification actionable.
- `waveshare_system_schedule_job()` registers one of eight periodic background
  callbacks. Callbacks execute on a FreeRTOS task outside LVGL and may post
  notifications, but must not manipulate LVGL objects directly.
- `waveshare_system_get_next_job_delay_ms()` provides the next requested wake
  delay to the sleep coordinator.
- `waveshare_system_request_ntp_sync()` restarts NTP synchronisation after an
  application establishes or changes Wi-Fi connectivity.

## Battery profile

The default profile is appropriate for the supplied AT103030 1000 mAh, 1S LiPo:

- 4.2 V maximum charge voltage, or 4.1 V when battery-care is selected later;
- 300 mA maximum charge current, because no manufacturer maximum current is
  known;
- 50 mA precharge and termination configured internally, not exposed as user
  settings;
- PMU thermal regulation at 80 C. At 70 C charging is reduced to 100 mA; at
  80 C charging is disabled; it resumes only at or below 65 C.

The AXP2101 reports its own die temperature. It is not a battery-cell sensor,
so this does not replace an NTC attached to the cell for a future production
power policy.

## Real sleep and scheduled wake

After the screen timeout, the display turns off. At ten minutes the device uses
ESP32-S3 light sleep; at twenty minutes it enters real deep sleep. Before deep
sleep the service stores each registered job's remaining delay in RTC memory,
then selects the earlier of the next job and the 30-minute fallback wake. After
the reset-style wake, an app registers its callback normally and receives the
preserved deadline instead of an unconditional immediate run.

The ESP32 wake inputs configured by this board port are CST9220 INT
(`GPIO11`), `+` (`GPIO18`), `-`/BOOT (`GPIO0`), and the RTC timer. The AXP2101
PWR key has no documented direct ESP32 wake GPIO in the Waveshare schematic
interface, so connected standby wakes every five seconds to poll its IRQ. A
long PWR press remains an AXP2101 hardware shutdown. Deep-sleep PWR-only wake
must be validated on the physical board; use touch or either side key as the
guaranteed software wake source meanwhile.

## Validated implementation summary (2026-07-20)

The current hardware-tested baseline includes:

- a three-page control centre opened from the status bar, with icon-only Wi-Fi,
  BLE, airplane and rotation-lock controls, brightness, page dots and upward
  close gesture;
- a notification page with a bounded queue, individual deletion, clear-all,
  actionable notifications, a status-bar count and a temporary foreground
  banner;
- status indicators whose layout does not overlap: notification count, Wi-Fi
  off/disconnected/connected, BLE enabled, airplane mode, battery level and USB
  power/charging state;
- physical-key launcher navigation, a full-tile 115 percent selection scale,
  long-press launch and long-press return to Home;
- protection against opening an empty recents screen and automatic return to
  Home after the final background app is closed;
- configurable screen idle, real connected light sleep and deep sleep,
  touch/PWR wake handling, 30-second notification wake and application display
  leases for always-visible use cases such as GPS;
- RTC-persisted periodic background callbacks outside the LVGL task, actionable
  notifications, selectable timezone and NTP synchronisation when Wi-Fi is available;
- conservative AXP2101 settings for the supplied 1000 mAh 1S LiPo, PMU thermal
  throttling, USB/VBUS detection and long-PWR shutdown request;
- QMI8658 automatic rotation with debouncing and a persistent rotation lock;
- ES7210 dual-microphone mute/gain, ES8311 speaker volume, and an always-on
  Espressif AFE pipeline for AEC, speech enhancement, noise suppression, VAD
  and AGC.

## CO5300 rotation solution

This board must not use mixed hardware and software rotation. The working
pipeline is:

- keep `Arduino_CO5300` permanently at `lcd->setRotation(0)`;
- use the QMI8658 only to select the logical `display_rotation`;
- rotate the LVGL framebuffer in `flush_rotated()`;
- apply the inverse mapping to CST9220 raw coordinates in
  `rotate_touch_point()`;
- reset the LVGL input state, invalidate the full display and wake the normal
  refresh timer when orientation changes.

The CO5300 implementation in Arduino_GFX states that the controller does not
support rotation. Its non-zero `setRotation()` modes only apply MADCTL axis
flips. A previous attempt to combine `setRotation(2)` with software rotation
made 180 degrees look mirrored and did not fix the problematic quarter-turn.

For `LV_DISPLAY_RENDER_MODE_FULL`, the port rotates 480 x 480 pixels through a
PSRAM buffer. The transpose loops therefore iterate destination columns first,
making writes contiguous. The final transfer goes through the RAM/non-const
`draw16bitRGBBitmap()` overload so Arduino_GFX calls the QSPI
`writePixels()` bulk path. Accidentally passing a `const uint16_t *` selects the
slow overload that writes each pixel separately; the visible symptom is a
multi-second vertical sweep and an apparently frozen UI while the rest of the
firmware continues to run.

Keep `LVGL_PORT_RENDER_MODE` as the single board-level choice. `FULL` is the
validated Waveshare value because it fixes SquareLine partial-redraw artifacts.
Other boards may keep `PARTIAL` if their hardware orientation and refresh path
are reliable.
