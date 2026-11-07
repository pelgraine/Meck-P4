# Switching between WiFi and Bluetooth builds

This guide is for anyone who builds Meck-P4 from source with ESP-IDF and switches between a WiFi build and a Bluetooth build. If you only flash the release files, you don't need it: pick the file for your board from the [releases page](https://github.com/pelgraine/Meck-P4/releases), and see the [C6 flashing guide](c6flashingguide.md) if you want Bluetooth.

## In short

- A WiFi build and a Bluetooth build need different firmware on the C6 chip and different menuconfig settings.
- Bluetooth build to WiFi build: three menuconfig changes (section 2).
- WiFi build back to Bluetooth build: undo those three, then six more changes (section 3). Undoing the three alone leaves Bluetooth on ESP-IDF's defaults, which are not Meck-P4's settings, and hides the Garmin switch.
- `sdkconfig` is not tracked by git, and `sdkconfig.defaults` has no Bluetooth, ESP-Hosted or Meck settings, so these settings live only in your own `sdkconfig`.

## 1. The C6 firmware has to match the build

- **WiFi build:** Meck-P4 starts the C6 with LilyGo's AT driver (`main/examples/lvgl_9_ui/main.cpp`, `ESP32C6_AT->begin()`), so the C6 must run ESP-AT.
- **Bluetooth build:** Meck-P4 starts the C6 through ESP-Hosted, so the C6 must run ESP-Hosted 2.0.17.
- **Changing the C6 firmware** means the cable procedure in the [C6 flashing guide](c6flashingguide.md) every time. Its **Going back to ESP-AT later** section covers putting ESP-AT back.
- **Boot log check after flashing:**
  - WiFi build: `esp32c6 init success` means ESP-AT answered. `esp32c6 init fail` means it didn't; that fits ESP-Hosted on the C6 but doesn't prove it.
  - Bluetooth build: one line starting `C6:`, for example `C6: ESP-Hosted link up after ... ms, the C6 reports ESP-Hosted firmware 2.0.17`.

## 2. Bluetooth build to WiFi build

In `idf.py menuconfig`:

1. Example Configuration: untick **Meck: ESP-Hosted C6 test build**. The Garmin switch disappears with it.
2. Component config > Wi-Fi: tick **Host WiFi Enable**.
3. Component config > Bluetooth: untick **Bluetooth**.
4. Save, quit, run `idf.py build`, then flash as usual.

Saving these settings removes every Bluetooth option from `sdkconfig`, and **Host WiFi Enable** switches Wi-Fi Remote off (the `esp_wifi_remote` component's Kconfig: `default n if ESP_HOST_WIFI_ENABLED`). That is why the way back needs more than undoing these three.

## 3. WiFi build back to Bluetooth build

In `idf.py menuconfig`, in this order:

1. Example Configuration: tick **Meck: ESP-Hosted C6 test build**.
2. Component config > Wi-Fi: untick **Host WiFi Enable**. With it ticked, the Bluetooth build does not compile.
3. Component config > Bluetooth: tick **Bluetooth**.
4. Component config > Bluetooth > Host: choose **NimBLE - BLE only**.
5. Component config > Bluetooth > NimBLE Options > Memory allocation strategy: choose **External SPIRAM**.
6. In NimBLE Options: tick **Persist the BLE Bonding keys in NVS**.
7. NimBLE Options > Host-controller Transport: untick **Enable Uart Transport**.
8. Component config > ESP-Hosted config > Bluetooth Support: tick **Enable Hosted Nimble Bluetooth support**. It only appears after step 7. Leave **BT Nimble HCI Type** on VHCI.
9. Garmin build only: Example Configuration: tick **Meck: Garmin watch support (BLE companion without a PIN)**. It appears again after step 4. Leave it unticked for the PIN build.
10. Save, quit, run `idf.py build`, then flash as usual.

Why each of steps 4 to 9 is needed:

- **Step 4:** after step 3, the Bluetooth host is Bluedroid, ESP-IDF's default (`components/bt/Kconfig`: `default BT_BLUEDROID_ENABLED`). The Garmin switch is hidden until the host is NimBLE (`main/Kconfig.projbuild`: `depends on MECK_C6_ESP_HOSTED_TEST && BT_NIMBLE_ENABLED`).
- **Step 5:** without it, NimBLE takes its memory from internal RAM instead of PSRAM.
- **Step 6:** without it, pairing isn't kept. The option's help text: "Enable this flag to make bonding persistent across device reboots". Phones then need the PIN again after every restart.
- **Steps 7 and 8:** without them, NimBLE is set to talk over UART and ESP-Hosted's Bluetooth support is off. Step 8 only appears once the UART transport is off (`overrides/esp_hosted/Kconfig`: `if BT_ENABLED && BT_NIMBLE_ENABLED && !BT_CONTROLLER_ENABLED && !BT_NIMBLE_TRANSPORT_UART`).
- **Step 9:** the Garmin switch is off after the WiFi round trip, like everything else in the Bluetooth settings.

The ESP-Hosted link settings (SDIO, with the ESP32-C6 as the slave chip) come back correctly by themselves after step 2.

## 4. How these steps were checked

- **When and with what:** 7 November 2026, with ESP-IDF v5.4.1, ESP-Hosted 2.0.17 and Meck-P4 `main` at `19101bc`, by reading the Kconfig files named above and running `idf.py reconfigure` on a copy of the settings.
- **The round trip:** a working Bluetooth + Garmin `sdkconfig` was taken through section 2, saved, then through steps 1 to 3 of section 3:
  - the Bluetooth host was Bluedroid and the Garmin switch was hidden;
  - after step 4 alone, the memory, bonding, transport and ESP-Hosted Bluetooth settings above were still different from the working build, and the Garmin switch was off;
  - after steps 4 to 9, the settings file was byte-for-byte the same as the working Bluetooth + Garmin one.
- **WiFi settings:** a build with the section 2 settings finished and contains `esp32c6 init success`, so it uses the AT path.
- **Not checked:** an `sdkconfig` with other changes of your own (the copy used was set up for the TFT SX1262 with no keyboard, and no board setting changed during the round trip); what a build made with step 4 alone would do on a device.
