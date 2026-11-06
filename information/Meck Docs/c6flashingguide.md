# Flashing the ESP-Hosted firmware onto the C6 chip: AMOLED LR2021 board

A step-by-step guide for your AMOLED LR2021 T-Display P4 and your Mac. The method was first used on a TFT SX1262 board in September 2026, and this version was checked for the AMOLED LR2021 board on 6 November 2026.

## What this does and why

The T-Display P4 has two chips that matter here. The ESP32-P4 is the main chip that runs Meck. The smaller ESP32-C6 chip provides WiFi and Bluetooth. The C6 has its own separate 4 MB of memory, so flashing Meck onto the P4 never changes what is on the C6. It has to be flashed through its own small UART socket on the case.

At the moment the C6 runs Espressif's ESP-AT firmware, which Meck-P4 talks to with text commands. This guide replaces it with Espressif's ESP-Hosted network-adapter firmware, version 2.0.17. That is the firmware the Bluetooth work needs: with it, the Bluetooth software runs on the P4 itself and the C6 acts as its radio, so the P4 can offer the standard MeshCore Bluetooth service the phone app looks for. Version 2.0.17 is the one pinned in Meck-P4's build files, so the P4 side and the C6 side match.

The short version: put a small helper program from LilyGo onto the P4, connect the USB adapter to the C6's socket, use two buttons to put the C6 into download mode, back up the C6, write the new firmware, then put Meck back onto the P4.

## Read this before you start

**Take the keyboard attachment off first.** The C6 flash only works with the T-Display P4 keyboard attachment removed, so take it off before you start, and leave it off until Part 7 is finished.

**Test the WiFi features on this board first.** Meck-P4's WiFi Companion and Web Reader both work by sending commands to the C6's ESP-AT firmware. After this guide they will stop working on this board until Meck-P4's WiFi is rewritten for ESP-Hosted. So do any testing that needs the app over WiFi on this board first.

**This can be undone.** Part 5 saves an exact copy of the C6 as it is now. Writing that copy back with the same steps returns the board to ESP-AT and working WiFi.

**Back up your Meck settings to the SD card.** LilyGo's helper program is written over the part of the P4's memory where Meck keeps its settings, channels, contacts and identity. When that memory is empty, Meck restores it from the backup on the SD card. Before you start, put an SD card in the P4, open **Settings**, tap **Backup to SD**, and wait until it shows OK with a count. Without a backup on an SD card, those settings will not come back.

## What you need

- **A USB-to-serial adapter.** This guide uses the SparkFun Serial Basic Breakout, CH340C and USB-C (product code DEV-15096). Leave the jumper on the back as it is, so it stays on 3.3 V signals.
- **The PiicoDev Prototyping Cable (Male) 200mm** (product code CE07796): a tiny 4-pin plug on one end for the C6 socket, four loose pins on the other for the adapter.
- **A USB cable for the adapter**, with USB-C at the adapter end.
- **The USB cable you normally use to flash the P4.** The P4 and the adapter are plugged into the Mac at the same time.
- **Your Mac with ESP-IDF set up** for building Meck-P4.
- **An SD card in the P4**, for the settings backup.

If you ever use a different adapter, LilyGo's rule is that it must use **3.3 V logic**, never 5 V.

## Where things are on the P4

- **C6 UART socket:** on the top edge of the case, beside the two gold antenna sockets (MMCX1 and MMCX2), with "C6 UART" printed next to it.
- **Buttons:** four small buttons on the right edge. From the top: **C6 BOOT**, **C6 Reset**, **P4 BOOT**, **P4 Reset**.
- **The P4's USB port:** the right-side USB-C port (the data port), the same one you use to flash Meck.

## Wiring

| Cable wire | Connect it to adapter pin | What it carries from the P4 |
|---|---|---|
| Black | GND | Ground |
| Blue | RXI | The C6's transmit line |
| Yellow | TXO | The C6's receive line |
| Red | Nothing, leave it loose | 3.3 V power |

The adapter's other pins (DTR, VCC and CTS) stay empty. The loose red wire carries power from the P4, so keep its metal tip away from the adapter and anything else metal. A small piece of tape over it does the job.

The C6 socket is wired the same way on the TFT and AMOLED versions of the board (page 7 of LilyGo's V1.0 schematics for both). LilyGo publishes the LR2021 as a separate radio schematic; it covers the LoRa radio, not the C6 or its socket.

## Part 1: Get your Mac ready

**Step 1. Load the ESP-IDF tools.** Open Terminal and run:

```
. $HOME/esp/esp-idf/export.sh
```

**Step 2. Check esptool is available.**

```
esptool.py version
```

It should print a version number. If Terminal says the command is not found, run `brew install esptool` and try again.

**Step 3. Download the two firmware files into their own folder.** If you already have a `c6flash` folder with both files in it, you can skip to Step 4 and just check them. Otherwise copy and paste these three lines:

```
mkdir -p ~/Downloads/c6flash && cd ~/Downloads/c6flash
curl -fL -o p4_prep.bin "https://raw.githubusercontent.com/Xinyuan-LilyGO/T-Display-P4/main/firmware/%5BT-Display-P4%5D%5Bcoprocessor_download_mode%5D/%5BT-Display-P4%5D%5Bcoprocessor_download_mode%5D_firmware_202607301512.bin"
curl -fL -o c6_esp_hosted_2.0.17.bin "https://raw.githubusercontent.com/pelgraine/Meck-P4/main/firmware/%5BT-Display-P4%5D%5Besp32c6_slave_esp_hosted_mcu_network_adapter%5D/%5BT-Display-P4%5D%5Besp32c6_slave_esp_hosted_mcu_network_adapter_v2.0.17%5D%5Bsdio%5D_firmware_202510081157.bin"
```

- `p4_prep.bin` is LilyGo's helper program for the P4 (`[T-Display-P4][coprocessor_download_mode]_firmware_202607301512.bin` in LilyGo's T-Display-P4 repo).
- `c6_esp_hosted_2.0.17.bin` is the new firmware for the C6 (`[T-Display-P4][esp32c6_slave_esp_hosted_mcu_network_adapter_v2.0.17][sdio]_firmware_202510081157.bin` in the Meck-P4 repo).

Use this exact C6 file, not the iperf file in Meck-P4's firmware folder.

**Step 4. Check the files are the right ones.**

```
cd ~/Downloads/c6flash
shasum -a 256 p4_prep.bin c6_esp_hosted_2.0.17.bin
```

The two long codes must match these exactly:

```
07eb134ec6f428d7681795bd369590af523cd44c90234a5d3bfe87f9a919391b  p4_prep.bin
de83d1c2816d90e0627d1d2544597488f82119edb6c7215d71150940acc6460d  c6_esp_hosted_2.0.17.bin
```

If either code is different, or a file is missing, stop and get it checked before flashing anything.

**Step 5. Find your two port names.** Your Mac gives each USB connection a name starting with `/dev/cu.`.

Unplug the P4 and the adapter from the Mac, then run:

```
ls /dev/cu.*
```

Plug the P4 in (right-side USB-C port) and run `ls /dev/cu.*` again. The new name is the P4's port, for example `/dev/cu.wchusbserial5B910814331`.

Plug the adapter in and run `ls /dev/cu.*` once more. The new name is the adapter's port, for example `/dev/cu.usbserial-240`.

Every command below uses those two example names. If yours are different, change them in each command before pressing Return.

## Part 2: Put LilyGo's helper program on the P4

The helper program hands control of the C6 over to the C6's own BOOT and Reset buttons, prints a message to say so, and then waits. LilyGo says not to flash the C6 until that message has appeared.

**Step 1.** Close anything that could be using the P4's port, such as a serial monitor or a web flasher tab.

**Step 2.** In Terminal, in the `c6flash` folder, write the helper program to the P4:

```
esptool.py --chip esp32p4 --port /dev/cu.wchusbserial5B910814331 write_flash 0x0 p4_prep.bin
```

Wait until it finishes. Use esptool for this file, not the MeshCore web flasher: the web flasher has left the P4 showing an `invalid header` error with this file.

**Step 3.** Open a second Terminal window (Command and N together) to watch the P4's messages. In that window, run these. If your Meck-P4 folder (the folder you run `idf.py build` in) is not `~/T-Display-P4`, change the `cd` line to point at it:

```
. $HOME/esp/esp-idf/export.sh
cd ~/T-Display-P4
idf.py -p /dev/cu.wchusbserial5B910814331 monitor
```

**Step 4.** Watch for this line:

```
Coprocessor preparation completed
```

If it has not appeared after a few seconds, press **P4 Reset** (the bottom button) and watch again. If you see `Coprocessor preparation failed` or `Coprocessor control hardware initialization failed` instead, stop here.

**Step 5.** Close the monitor with Control and ] together.

## Part 3: Connect the adapter

**Step 1.** Turn the P4 fully off: unplug its USB cable and switch its battery switch off.

**Step 2.** Connect the cable's loose pins to the adapter: black to GND, blue to RXI, yellow to TXO, red left loose with its tip taped over.

**Step 3.** Push the tiny plug straight into the C6 UART socket until it is fully in.

**Step 4.** Check the adapter's USB cable is plugged into the Mac.

**Step 5.** Switch the P4's battery switch back on, and plug the P4's USB cable back into the Mac.

**Step 6.** In the second Terminal window, start the monitor again:

```
idf.py -p /dev/cu.wchusbserial5B910814331 monitor
```

Check that `Coprocessor preparation completed` appears again (press **P4 Reset** if needed). Do not go on until it does. Leave this window open.

## Part 4: Put the C6 into download mode and check the connection

**Step 1.** Press and hold **C6 BOOT** (the top button).

**Step 2.** While still holding it, press and release **C6 Reset** (the second button).

**Step 3.** Let go of **C6 BOOT**.

**Step 4.** In the first Terminal window (in the `c6flash` folder), run:

```
esptool.py --chip esp32c6 --port /dev/cu.usbserial-240 chip_id
```

It has worked if you see lines like these (the MAC address and other details differ per board):

```
Chip is ESP32-C6FH4 (QFN32) (revision v0.2)
Features: Wi-Fi 6, BT 5 (LE), IEEE802.15.4, Single Core + LP Core, 160MHz, Embedded Flash 4MB
```

Two lines that look worrying are normal: `Warning: ESP32-C6 has no Chip ID. Reading MAC instead.` and, at the end, `Hard resetting via RTS pin...`. The C6 stays in download mode, so carry straight on.

If it cannot connect:

- Check the second window still shows `Coprocessor preparation completed`.
- Do the three button steps again, then run the command again straight away.
- Check each wire against the wiring table, and that the pins sit firmly on the adapter.
- Check you typed the adapter's port name, not the P4's.

## Part 5: Back up what is on the C6 now

Do this for this board. It is the only exact copy of what this board's C6 came with, and it is how you get back to working WiFi later.

**This step takes several minutes.** It reads the C6's whole 4 MB of memory piece by piece, and Terminal prints a line for each of the 64 pieces as it goes. Let it run to the end.

Reading all 4 MB in one go has failed with `Corrupt data`, so this block reads it in 64 small pieces and retries any piece that fails. It starts by clearing out pieces left over from a previous board, so a backup can never be built from another board's pieces.

Paste this whole block into the first Terminal window in one go:

```
cd ~/Downloads/c6flash && rm -rf c6chunks && mkdir c6chunks && for i in $(seq 0 63); do
  off=$((i*65536)); f=c6chunks/chunk_$(printf %02d $i).bin
  until [ -f "$f" ] && [ "$(stat -f%z "$f")" -eq 65536 ]; do
    echo "== piece $i of 63 =="
    esptool.py --chip esp32c6 --port /dev/cu.usbserial-240 read_flash $off 65536 "$f" || { rm -f "$f"; sleep 1; }
  done
done && cat c6chunks/chunk_*.bin > c6_backup.bin && ls -la c6_backup.bin && xxd -l 1 c6_backup.bin
```

Leave it running until Terminal is ready for a new command, which takes several minutes. It has worked if `c6_backup.bin` is listed at exactly `4194304` bytes and the last line shows `e9`.

Straight away, give the backup this board's name so no later backup can replace it:

```
mv c6_backup.bin c6_backup_amoled_lr2021.bin
```

If one piece keeps failing over and over, check the pins are firmly on the adapter, especially GND. If the C6 seems to have dropped out of download mode, press Control and C to stop the block, do the three button steps from Part 4 again, and paste the block again. It starts over from piece 0.

## Part 6: Write the new firmware to the C6

In the first Terminal window, in the `c6flash` folder, run:

```
esptool.py --chip esp32c6 --port /dev/cu.usbserial-240 write_flash 0x0 c6_esp_hosted_2.0.17.bin
```

Do not add `--baud 921600`. That speed has failed on this adapter, and the normal speed has worked.

It is done when it prints `Hash of data verified`. If it gets stuck on `Connecting...`, do the three C6 button steps from Part 4 again and rerun the command.

## Part 7: Put Meck back on the P4

**Step 1.** In the second Terminal window, close the monitor (Control and ] together) so the P4's port is free.

**Step 2.** Flash Meck onto the P4 from your Meck-P4 folder, the same way you do after a build:

```
cd ~/T-Display-P4
idf.py -p /dev/cu.wchusbserial5B910814331 flash
```

**Step 3.** Pull the tiny plug out of the C6 UART socket.

**Step 4.** Power-cycle the whole device: unplug the P4's USB cable, switch the battery switch off, wait a few seconds, then switch it back on.

**Step 5.** Check that Meck starts and that your node name, contacts and channels are back. Expect WiFi Companion and the Web Reader not to connect on this board from now on.

## Going back to ESP-AT later

Do Parts 1 to 4 again, then in Part 6 write the backup instead of the ESP-Hosted file:

```
esptool.py --chip esp32c6 --port /dev/cu.usbserial-240 write_flash 0x0 c6_backup_amoled_lr2021.bin
```

Then finish with Part 7.

## If something goes wrong

- **`No such file or directory`:** Terminal is not in the folder with the files. Run `cd ~/Downloads/c6flash` and try again.
- **esptool says it cannot open the P4's port, or that it is busy:** close the monitor (Control and ] together) and any web flasher tab, then try again.
- **`Corrupt data` during the backup:** the piece-by-piece block in Part 5 retries automatically.
- **`Unable to verify flash chip connection (No serial data received.)` just after `Changing baud rate`:** take `--baud 921600` out of the command.
- **The P4 shows `invalid header` after the web flasher was used for LilyGo's helper file:** write that file with esptool instead, as in Part 2.
- **The P4 keeps starting in download mode:** unplug USB and any other power, switch the battery switch off, press **P4 Reset** to help drain leftover power, wait until the device is fully off, then switch it back on.
- **`command not found` for `esptool.py` or `idf.py`:** run `. $HOME/esp/esp-idf/export.sh` in that Terminal window first. Each new window needs it.
- **`idf.py` says the directory is not buildable:** that window is not in your Meck-P4 folder. `cd` into it and try again.
- **Warning lines saying something is deprecated:** these say a command has a newer name in newer esptool versions. esptool has carried on after them.

## Checklist

1. Take the keyboard attachment off.
2. Do any WiFi app testing on this board first.
3. SD card in the P4, then **Settings**, **Backup to SD**.
4. Terminal: `. $HOME/esp/esp-idf/export.sh`, `cd ~/Downloads/c6flash`, then check both files with `shasum -a 256` (Part 1, Step 4).
5. Find both port names with `ls /dev/cu.*`.
6. Write `p4_prep.bin` to the P4 with esptool (Part 2).
7. Monitor: wait for `Coprocessor preparation completed`, then close it.
8. P4 off (USB out, battery switch off). Wire the adapter: black GND, blue RXI, yellow TXO, red loose. Plug into C6 UART.
9. P4 on. Monitor: `Coprocessor preparation completed` again.
10. Hold C6 BOOT, press and release C6 Reset, let go of C6 BOOT.
11. `chip_id` on the adapter's port.
12. Backup (Part 5, several minutes), then rename it `c6_backup_amoled_lr2021.bin`.
13. `write_flash 0x0 c6_esp_hosted_2.0.17.bin` on the adapter's port; wait for `Hash of data verified`.
14. Close the monitor, `idf.py -p <P4 port> flash` from your Meck-P4 folder, unplug the C6 cable, power-cycle, check your settings are back.

## Where the facts in this guide come from

- Checked on 6 November 2026: both download links above (each file downloaded and its SHA-256 matched the codes printed here); LilyGo's T-Display-P4 repo at `de2e33d` (2 Nov 2026), where the README's only change since commit `ba014d6` is new rows in its schematics table, and where `LR2021_868_915_V1.0.pdf` is a two-page radio schematic; Meck-P4 `main` at `0063017` (5 Nov 2026): `esp_hosted` pinned at 2.0.17 in `main/idf_component.yml`, `partitions.csv` unchanged since September 2026, the **Backup to SD** row in `MeckUI.cpp`, and `SerialC6WiFiInterface.cpp` driving the C6 with AT commands, with no ESP-Hosted calls anywhere in Meck's code.
- Found on an AMOLED LR2021 board on 6 November 2026: the C6 flash only works with the keyboard attachment removed, and the Part 5 backup takes several minutes.
- From work on a TFT SX1262 board in September 2026 (not re-checked on 6 November 2026): the wiring, button order, connector location and the TFT/AMOLED schematic comparison; the helper program's messages; the web flasher `invalid header` result; the failed full-size backup and `--baud 921600` result; the helper file overwriting Meck's settings area.