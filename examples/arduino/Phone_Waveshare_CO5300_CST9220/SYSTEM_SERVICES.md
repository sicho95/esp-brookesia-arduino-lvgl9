# Waveshare system services

This example contains a board-specific services layer in `waveshare_system.cpp`.
It is intentionally kept out of the generic Brookesia core: the CO5300 display,
AXP2101 PMU, physical keys and their safe wake sources belong to this board port.

## Test now

- Swipe down from the top edge to open the control centre. Use its `Controles`
  and `Notifications` pages; swipe upward to close it. While open, Brookesia
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
  time using NTP and the Europe/Paris daylight-saving rules. The status-bar
  clock already reads this system time.
- Short PWR moves an active device to connected standby and wakes it from a
  screen-off state. Long PWR saves settings and requests an AXP2101 shutdown.

## Application API

- `waveshare_system_post_notification()` copies a notification into a bounded
  six-entry queue. An app ID makes the notification actionable.
- `waveshare_system_schedule_job()` registers one of eight periodic background
  callbacks. Callbacks execute on a FreeRTOS task outside LVGL and may post
  notifications, but must not manipulate LVGL objects directly.
- `waveshare_system_get_next_job_delay_ms()` provides the next requested wake
  delay for the future deep-sleep coordinator.
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

## Deliberately not enabled yet

After 10 minutes the service enters a connected-standby state and after
20 minutes it reports `Deep sleep pret`, but it does not call
`esp_deep_sleep_start()`. The exact PWR/PMU interrupt wiring and wake sources
must be tested first. Enabling deep sleep before that validation can leave a
battery-powered board apparently dead and unable to wake from the intended
button.

Scheduled jobs currently run while the ESP32 is awake. Persisting job deadlines
across reboot and using the next deadline as an RTC/PMU deep-sleep wake source
require the next hardware-validation pass.

## Validated implementation summary (2026-07-20)

The current hardware-tested baseline includes:

- a two-page control centre opened from the status bar, with icon-only Wi-Fi,
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
- one-minute screen idle, connected-standby and deep-sleep-pending states,
  touch/PWR wake handling, 30-second notification wake and application display
  leases for always-visible use cases such as GPS;
- periodic background callbacks outside the LVGL task, actionable
  notifications and NTP synchronisation when Wi-Fi is available;
- conservative AXP2101 settings for the supplied 1000 mAh 1S LiPo, PMU thermal
  throttling, USB/VBUS detection and long-PWR shutdown request;
- QMI8658 automatic rotation with debouncing and a persistent rotation lock.

Real ESP32 deep sleep is still intentionally disabled. The framework reaches a
`DEEP_SLEEP_PENDING` state, but PMU/button wake wiring and persistence of
scheduled deadlines must be validated before calling `esp_deep_sleep_start()`.

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
