# Flashing the ESP-Hosted firmware onto the C6 chip

A step-by-step guide for your T-Display P4 and a Mac or Windows PC. The steps are the same for every version of the board, TFT or AMOLED, SX1262 or LR2021: the C6's socket and buttons are wired the same way on all of them (see Wiring). Where the Mac and Windows steps differ, both are given.

## What this does and why

The T-Display P4 has two chips that matter here. The ESP32-P4 is the main chip that runs Meck. The smaller ESP32-C6 chip provides WiFi and Bluetooth. The C6 has its own separate 4 MB of memory, so flashing Meck onto the P4 never changes what is on the C6. It has to be flashed through its own small UART socket on the case.

At the moment the C6 runs Espressif's ESP-AT firmware, which Meck-P4 talks to with text commands. This guide replaces it with Espressif's ESP-Hosted network-adapter firmware, version 2.0.17. That is the firmware Meck-P4's Bluetooth builds need: with it, the Bluetooth software runs on the P4 itself and the C6 acts as its radio, so the P4 can offer the standard MeshCore Bluetooth service the phone app looks for. Version 2.0.17 is the one pinned in Meck-P4's build files, so the P4 side and the C6 side match.

The short version: put a small helper program from LilyGo onto the P4, connect the USB adapter to the C6's socket, use two buttons to put the C6 into download mode, back up the C6, write the new firmware, then put a Meck-P4 Bluetooth build onto the P4.

## Read this before you start

**If your P4 has the keyboard attachment, take it off first.** The C6 flash only works with the T-Display P4 keyboard attachment removed, so take it off before you start, and leave it off until Part 7 is finished.

**Test the WiFi features on this board first.** Meck-P4's WiFi Companion and Web Reader both work by sending commands to the C6's ESP-AT firmware. After this guide they will stop working on this board until Meck-P4's WiFi is rewritten for ESP-Hosted. So do any testing that needs the app over WiFi on this board first.

**This can be undone.** Part 5 saves an exact copy of the C6 as it is now. Writing that copy back with the same steps returns the board to ESP-AT and working WiFi.

**Back up your Meck settings to the SD card.** LilyGo's helper program is written over the part of the P4's memory where Meck keeps its settings, channels, contacts and identity. When that memory is empty, Meck restores it from the backup on the SD card. Before you start, put an SD card in the P4, open **Settings**, tap **Backup to SD**, and wait until it shows OK with a count. Without a backup on an SD card, those settings will not come back.

## What you need

- **A USB-to-serial adapter.** This guide uses the SparkFun Serial Basic Breakout, CH340C and USB-C (product code DEV-15096). Leave the jumper on the back as it is, so it stays on 3.3 V signals.
- **The PiicoDev Prototyping Cable (Male) 200mm** (product code CE07796): a tiny 4-pin plug on one end for the C6 socket, four loose pins on the other for the adapter.
- **A USB cable for the adapter**, with USB-C at the adapter end.
- **The USB cable you normally use to flash the P4.** The P4 and the adapter are plugged into the computer at the same time.
- **Python 3.10 or newer on your computer.** Part 1 checks for it and installs esptool, the flashing tool, with it.
- **The Meck-P4 Bluetooth build for your board**, from the Meck-P4 releases page (Part 1, Step 3).
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

## Part 1: Get your computer ready

You type the commands in this guide into a terminal window:

- **Mac:** open **Terminal** (in Applications > Utilities).
- **Windows:** open **PowerShell** (Start menu, type PowerShell). All the Windows commands in this guide are for PowerShell.

**Step 1. Check Python.**

**Mac:** run:

```
python3 --version
```

It should print `Python 3.10` or a higher number. If the number is lower, or Terminal says `python3` is not found, install the current Python 3 for macOS from <https://www.python.org/downloads/macos/>, open a new Terminal window, and check again.

**Windows:** run:

```
python --version
```

It should print `Python 3.10` or a higher number. If Windows says `python` is not recognised, install the **Python install manager** from <https://www.python.org/downloads/windows/> (or from the Microsoft Store), open a new PowerShell window, and run `python --version` again. If no Python is installed yet, this first run installs the latest version before printing its number.

**Step 2. Install esptool.**

**Mac:** copy and paste these three lines:

```
python3 -m venv ~/c6flash-venv
source ~/c6flash-venv/bin/activate
pip install esptool
```

**Every new Terminal window needs the `source` line again** before you use esptool or the monitor:

```
source ~/c6flash-venv/bin/activate
```

**Windows:** first allow PowerShell to run the small start-up script Python uses (you only do this once; type `Y` and press Enter if it asks):

```
Set-ExecutionPolicy -ExecutionPolicy RemoteSigned -Scope CurrentUser
```

Then copy and paste these three lines:

```
python -m venv "$HOME\c6flash-venv"
& "$HOME\c6flash-venv\Scripts\Activate.ps1"
pip install esptool
```

**Every new PowerShell window needs the `Activate.ps1` line again** before you use esptool or the monitor:

```
& "$HOME\c6flash-venv\Scripts\Activate.ps1"
```

On both: this puts esptool, and the serial monitor that comes with it, in a folder of their own (`c6flash-venv` in your home folder), apart from the rest of your computer. You only install it once. Then check esptool works:

```
esptool version
```

It should print a version number.

**Step 3. Download the firmware files into their own folder.** If you already have a `c6flash` folder with the files in it, you can skip to Step 4 and just check them.

**Mac:** copy and paste these three lines:

```
mkdir -p ~/Downloads/c6flash && cd ~/Downloads/c6flash
curl -fL -o p4_prep.bin "https://raw.githubusercontent.com/Xinyuan-LilyGO/T-Display-P4/main/firmware/%5BT-Display-P4%5D%5Bcoprocessor_download_mode%5D/%5BT-Display-P4%5D%5Bcoprocessor_download_mode%5D_firmware_202607301512.bin"
curl -fL -o c6_esp_hosted_2.0.17.bin "https://raw.githubusercontent.com/pelgraine/Meck-P4/main/firmware/%5BT-Display-P4%5D%5Besp32c6_slave_esp_hosted_mcu_network_adapter%5D/%5BT-Display-P4%5D%5Besp32c6_slave_esp_hosted_mcu_network_adapter_v2.0.17%5D%5Bsdio%5D_firmware_202510081157.bin"
```

**Windows:** copy and paste these four lines:

```
mkdir "$HOME\Downloads\c6flash" -Force | Out-Null
cd "$HOME\Downloads\c6flash"
Invoke-WebRequest -UseBasicParsing -OutFile p4_prep.bin -Uri "https://raw.githubusercontent.com/Xinyuan-LilyGO/T-Display-P4/main/firmware/%5BT-Display-P4%5D%5Bcoprocessor_download_mode%5D/%5BT-Display-P4%5D%5Bcoprocessor_download_mode%5D_firmware_202607301512.bin"
Invoke-WebRequest -UseBasicParsing -OutFile c6_esp_hosted_2.0.17.bin -Uri "https://raw.githubusercontent.com/pelgraine/Meck-P4/main/firmware/%5BT-Display-P4%5D%5Besp32c6_slave_esp_hosted_mcu_network_adapter%5D/%5BT-Display-P4%5D%5Besp32c6_slave_esp_hosted_mcu_network_adapter_v2.0.17%5D%5Bsdio%5D_firmware_202510081157.bin"
```

What the two files are:

- `p4_prep.bin` is LilyGo's helper program for the P4 (`[T-Display-P4][coprocessor_download_mode]_firmware_202607301512.bin` in LilyGo's T-Display-P4 repo).
- `c6_esp_hosted_2.0.17.bin` is the new firmware for the C6 (`[T-Display-P4][esp32c6_slave_esp_hosted_mcu_network_adapter_v2.0.17][sdio]_firmware_202510081157.bin` in the Meck-P4 repo).

Use this exact C6 file, not the iperf file in Meck-P4's firmware folder.

Then download the Meck-P4 Bluetooth build for your board from <https://github.com/pelgraine/Meck-P4/releases> and move it into the same `c6flash` folder (in your Downloads folder). There is one file for each board type: pick the one for your screen (TFT or AMOLED), your LoRa radio (SX1262 or LR2021) and whether you use the keyboard attachment, for example `meck-p4-0.9-amoled-LR2021-ble-merged.bin`. Part 7 writes it to the P4.

**Step 4. Check the files are the right ones.**

**Mac:**

```
cd ~/Downloads/c6flash
shasum -a 256 p4_prep.bin c6_esp_hosted_2.0.17.bin
```

**Windows:**

```
cd "$HOME\Downloads\c6flash"
Get-FileHash p4_prep.bin, c6_esp_hosted_2.0.17.bin
```

The two long codes must match these exactly (Windows shows them in capital letters, which is fine):

```
07eb134ec6f428d7681795bd369590af523cd44c90234a5d3bfe87f9a919391b  p4_prep.bin
de83d1c2816d90e0627d1d2544597488f82119edb6c7215d71150940acc6460d  c6_esp_hosted_2.0.17.bin
```

If either code is different, or a file is missing, stop and get it checked before flashing anything.

**Step 5. Find your two port names.** Your computer gives each USB connection a name: on a Mac it starts with `/dev/cu.`, on Windows it is `COM` and a number.

**Mac:** unplug the P4 and the adapter, then run:

```
ls /dev/cu.*
```

Plug the P4 in (right-side USB-C port) and run `ls /dev/cu.*` again. The new name is the P4's port, for example `/dev/cu.wchusbserial5B910814331`. Plug the adapter in and run `ls /dev/cu.*` once more. The new name is the adapter's port, for example `/dev/cu.usbserial-240`.

**Windows:** unplug the P4 and the adapter, then run (it comes with esptool, so the `Activate.ps1` line must have been run in this window):

```
pyserial-ports
```

Plug the P4 in (right-side USB-C port) and run `pyserial-ports` again. The new name is the P4's port, for example `COM5`. Plug the adapter in and run `pyserial-ports` once more. The new name is the adapter's port, for example `COM4`. Device Manager shows the same names under **Ports (COM & LPT)**, where the adapter appears as **USB-SERIAL CH340**. Windows normally sets the adapter up by itself; if it never appears, SparkFun's [CH340 driver guide](https://learn.sparkfun.com/tutorials/how-to-install-ch340-drivers/all) has the driver.

Every command below uses those example names. If yours are different, change them in each command before pressing Return or Enter.

## Part 2: Put LilyGo's helper program on the P4

The helper program hands control of the C6 over to the C6's own BOOT and Reset buttons, prints a message to say so, and then waits. LilyGo says not to flash the C6 until that message has appeared.

**Step 1.** Close anything that could be using the P4's port, such as a serial monitor or a web flasher tab.

**Step 2.** In the terminal window, in the `c6flash` folder, write the helper program to the P4.

**Mac:**

```
esptool --chip esp32p4 --port /dev/cu.wchusbserial5B910814331 write-flash 0x0 p4_prep.bin
```

**Windows:**

```
esptool --chip esp32p4 --port COM5 write-flash 0x0 p4_prep.bin
```

Wait until it finishes. Use esptool for this file, not the MeshCore web flasher: the web flasher has left the P4 showing an `invalid header` error with this file.

**Step 3.** Open a second terminal window to watch the P4's messages (on a Mac, Command and N together in Terminal; on Windows, open PowerShell again from the Start menu). In that window, run these two lines.

**Mac:**

```
source ~/c6flash-venv/bin/activate
pyserial-miniterm --dtr 0 --rts 0 /dev/cu.wchusbserial5B910814331 115200
```

**Windows:**

```
& "$HOME\c6flash-venv\Scripts\Activate.ps1"
pyserial-miniterm --dtr 0 --rts 0 COM5 115200
```

This is the serial monitor that came with esptool. Opening it does not restart the P4, so the message in Step 4 may already have gone by: press **P4 Reset** (the bottom button) to see it again.

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

**Step 4.** Check the adapter's USB cable is plugged into the computer.

**Step 5.** Switch the P4's battery switch back on, and plug the P4's USB cable back into the computer.

**Step 6.** In the second terminal window, start the monitor again.

**Mac:**

```
pyserial-miniterm --dtr 0 --rts 0 /dev/cu.wchusbserial5B910814331 115200
```

**Windows:**

```
pyserial-miniterm --dtr 0 --rts 0 COM5 115200
```

Check that `Coprocessor preparation completed` appears again (press **P4 Reset** if needed). Do not go on until it does. Leave this window open.

## Part 4: Put the C6 into download mode and check the connection

**Step 1.** Press and hold **C6 BOOT** (the top button).

**Step 2.** While still holding it, press and release **C6 Reset** (the second button).

**Step 3.** Let go of **C6 BOOT**.

**Step 4.** In the first terminal window (in the `c6flash` folder), run:

**Mac:**

```
esptool --chip esp32c6 --port /dev/cu.usbserial-240 chip-id
```

**Windows:**

```
esptool --chip esp32c6 --port COM4 chip-id
```

It has worked if you see lines like these (the MAC address and other details differ per board):

```
Chip type:          ESP32-C6FH4 (QFN32) (revision v0.2)
Features:           Wi-Fi 6, BT 5 (LE), IEEE802.15.4, Single Core + LP Core, 160MHz, Embedded Flash 4MB
```

Older esptool versions start the first line with `Chip is` instead of `Chip type:`.

Some lines that look worrying are normal: `ESP32-C6 has no chip ID. Reading MAC address instead.` and, at the end, `Hard resetting via RTS pin...`. The C6 stays in download mode, so carry straight on.

If it cannot connect:

- Check the second window still shows `Coprocessor preparation completed`.
- Do the three button steps again, then run the command again straight away.
- Check each wire against the wiring table, and that the pins sit firmly on the adapter.
- Check you typed the adapter's port name, not the P4's.

## Part 5: Back up what is on the C6 now

Do this for this board. It is the only exact copy of what this board's C6 came with, and it is how you get back to working WiFi later.

**This step takes several minutes.** It reads the C6's whole 4 MB of memory piece by piece, and the terminal prints a line for each of the 64 pieces as it goes. Let it run to the end.

Reading all 4 MB in one go has failed with `Corrupt data`, so this block reads it in 64 small pieces and retries any piece that fails. It starts by clearing out pieces left over from a previous board, so a backup can never be built from another board's pieces.

Paste the whole block for your computer into the first terminal window in one go.

**Mac:**

```
cd ~/Downloads/c6flash && rm -rf c6chunks && mkdir c6chunks && for i in $(seq 0 63); do
  off=$((i*65536)); f=c6chunks/chunk_$(printf %02d $i).bin
  until [ -f "$f" ] && [ "$(stat -f%z "$f")" -eq 65536 ]; do
    echo "== piece $i of 63 =="
    esptool --chip esp32c6 --port /dev/cu.usbserial-240 read-flash $off 65536 "$f" || { rm -f "$f"; sleep 1; }
  done
done && cat c6chunks/chunk_*.bin > c6_backup.bin && ls -la c6_backup.bin && xxd -l 1 c6_backup.bin
```

It has worked if `c6_backup.bin` is listed at exactly `4194304` bytes and the last line shows `e9`.

**Windows:**

```
cd "$HOME\Downloads\c6flash"; Remove-Item c6chunks -Recurse -Force -ErrorAction SilentlyContinue; mkdir c6chunks | Out-Null
for ($i = 0; $i -lt 64; $i++) {
  $off = $i * 65536; $f = "c6chunks\chunk_{0:D2}.bin" -f $i
  while (-not ((Test-Path $f) -and ((Get-Item $f).Length -eq 65536))) {
    "== piece $i of 63 =="
    esptool --chip esp32c6 --port COM4 read-flash $off 65536 $f
    if ($LASTEXITCODE -ne 0) { Remove-Item $f -ErrorAction SilentlyContinue; Start-Sleep 1 }
  }
}
$out = [System.IO.File]::Create((Join-Path $PWD "c6_backup.bin"))
foreach ($c in (Get-ChildItem c6chunks -Filter "chunk_*.bin" | Sort-Object Name)) { $b = [System.IO.File]::ReadAllBytes($c.FullName); $out.Write($b, 0, $b.Length) }
$out.Close()
(Get-Item c6_backup.bin).Length
"{0:x2}" -f [System.IO.File]::ReadAllBytes((Join-Path $PWD "c6_backup.bin"))[0]
```

It has worked if the last two lines it prints are `4194304` and `e9`.

Leave it running until the terminal is ready for a new command, which takes several minutes. Straight away, give the backup its own name so no later backup can replace it.

**Mac:**

```
mv c6_backup.bin c6_backup_myp4.bin
```

**Windows:**

```
Rename-Item c6_backup.bin c6_backup_myp4.bin
```

If you flash more than one board, give each backup a different name, for example by changing `myp4` to something that tells your boards apart.

If one piece keeps failing over and over, check the pins are firmly on the adapter, especially GND. If the C6 seems to have dropped out of download mode, press Control and C to stop the block, do the three button steps from Part 4 again, and paste the block again. It starts over from piece 0.

## Part 6: Write the new firmware to the C6

In the first terminal window, in the `c6flash` folder, run:

**Mac:**

```
esptool --chip esp32c6 --port /dev/cu.usbserial-240 write-flash 0x0 c6_esp_hosted_2.0.17.bin
```

**Windows:**

```
esptool --chip esp32c6 --port COM4 write-flash 0x0 c6_esp_hosted_2.0.17.bin
```

Do not add `--baud 921600`. That speed has failed on this adapter, and the normal speed has worked.

It is done when it prints `Hash of data verified`. If it gets stuck on `Connecting...`, do the three C6 button steps from Part 4 again and rerun the command.

## Part 7: Put a Meck-P4 Bluetooth build on the P4

**Step 1.** In the second terminal window, close the monitor (Control and ] together) so the P4's port is free.

**Step 2.** In the first terminal window, in the `c6flash` folder, write the Bluetooth build you downloaded in Part 1 to the P4. Change the file name to the one for your board.

**Mac:**

```
esptool --chip esp32p4 --port /dev/cu.wchusbserial5B910814331 write-flash 0x0 meck-p4-0.9-amoled-LR2021-ble-merged.bin
```

**Windows:**

```
esptool --chip esp32p4 --port COM5 write-flash 0x0 meck-p4-0.9-amoled-LR2021-ble-merged.bin
```

Wait until it finishes. If you build Meck-P4 yourself with ESP-IDF and the Bluetooth settings switched on, you can flash your own build instead with `idf.py -p <P4 port> flash` from your Meck-P4 folder.

**Step 3.** Pull the tiny plug out of the C6 UART socket.

**Step 4.** Power-cycle the whole device: unplug the P4's USB cable, switch the battery switch off, wait a few seconds, then switch it back on.

**Step 5.** Check that Meck starts and that your node name, contacts and channels are back. WiFi Companion and the Web Reader do not work in the Bluetooth builds.

**Step 6. Turn Bluetooth on and pair your phone.** Bluetooth is off until you turn it on:

1. Open **Settings** and tap **BLE Companion (tap to toggle)** to turn it on.
2. The home screen now shows **BLE PIN:** followed by six digits.
3. In the MeshCore app, connect to **MeshCore-** followed by your node name, and type that PIN when your phone asks for it.

Your phone remembers the pairing, so it reconnects without the PIN after the P4 restarts.

If you want to check the C6 link, start the monitor from Part 2, Step 3 and press **P4 Reset**. A line starting `C6: ESP-Hosted link up` means the P4 and the C6 are talking.

## Going back to ESP-AT later

Do Parts 1 to 4 again, then in Part 6 write the backup instead of the ESP-Hosted file.

**Mac:**

```
esptool --chip esp32c6 --port /dev/cu.usbserial-240 write-flash 0x0 c6_backup_myp4.bin
```

**Windows:**

```
esptool --chip esp32c6 --port COM4 write-flash 0x0 c6_backup_myp4.bin
```

Use the name you gave that board's backup in Part 5. Then finish with Part 7, but in Step 2 write the normal Meck-P4 build for your board, not the Bluetooth one, so WiFi Companion and the Web Reader work again.

## If something goes wrong

- **`No such file or directory`, or a file "does not exist":** the terminal is not in the folder with the files. Run `cd ~/Downloads/c6flash` (Mac) or `cd "$HOME\Downloads\c6flash"` (Windows) and try again.
- **esptool says it cannot open the P4's port, or that it is busy:** close the monitor (Control and ] together) and any web flasher tab, then try again.
- **`Corrupt data` during the backup:** the piece-by-piece block in Part 5 retries automatically.
- **`Unable to verify flash chip connection (No serial data received.)` just after `Changing baud rate`:** take `--baud 921600` out of the command.
- **The P4 shows `invalid header` after the web flasher was used for LilyGo's helper file:** write that file with esptool instead, as in Part 2.
- **The P4 keeps starting in download mode:** unplug USB and any other power, switch the battery switch off, press **P4 Reset** to help drain leftover power, wait until the device is fully off, then switch it back on.
- **`command not found` (Mac) or "is not recognized" (Windows) for `esptool`, `pyserial-miniterm` or `pyserial-ports`:** run the activate line from Part 1, Step 2 in that window first. Each new window needs it.
- **Windows says "running scripts is disabled on this system" when you run the `Activate.ps1` line:** run the `Set-ExecutionPolicy` line from Part 1, Step 2, then try again.
- **Windows: the adapter never shows up in `pyserial-ports`:** install the driver from SparkFun's [CH340 driver guide](https://learn.sparkfun.com/tutorials/how-to-install-ch340-drivers/all), then unplug the adapter and plug it in again.

## Checklist

1. Take the keyboard attachment off, if your P4 has one.
2. Do any WiFi app testing on this board first.
3. SD card in the P4, then **Settings**, **Backup to SD**.
4. In a terminal window, run the activate line (after installing esptool once, Part 1, Step 2), go to the `c6flash` folder, then check both files (Part 1, Step 4). Have your board's Meck-P4 Bluetooth build in the same folder.
5. Find both port names (Part 1, Step 5).
6. Write `p4_prep.bin` to the P4 with esptool (Part 2).
7. Monitor (`pyserial-miniterm`, press **P4 Reset** if needed): wait for `Coprocessor preparation completed`, then close it.
8. P4 off (USB out, battery switch off). Wire the adapter: black GND, blue RXI, yellow TXO, red loose. Plug into C6 UART.
9. P4 on. Monitor: `Coprocessor preparation completed` again.
10. Hold C6 BOOT, press and release C6 Reset, let go of C6 BOOT.
11. `chip-id` on the adapter's port.
12. Backup (Part 5, several minutes), then rename it, for example `c6_backup_myp4.bin`.
13. `write-flash 0x0 c6_esp_hosted_2.0.17.bin` on the adapter's port; wait for `Hash of data verified`.
14. Close the monitor, write your board's Meck-P4 Bluetooth build to the P4 at `0x0` with esptool (Part 7), unplug the C6 cable, power-cycle, check your settings are back.
15. **Settings**, **BLE Companion (tap to toggle)**, then pair from the MeshCore app with the PIN shown on the home screen.

## Where the facts in this guide come from

- Checked on 6 November 2026: both download links above (each file downloaded and its SHA-256 matched the codes printed here); LilyGo's T-Display-P4 repo at `de2e33d` (2 Nov 2026), where the README's only change since commit `ba014d6` is new rows in its schematics table, and where `LR2021_868_915_V1.0.pdf` is a two-page radio schematic; Meck-P4 `main` at `0063017` (5 Nov 2026): `esp_hosted` pinned at 2.0.17 in `main/idf_component.yml`, `partitions.csv` unchanged since September 2026, the **Backup to SD** row in `MeckUI.cpp`, and `SerialC6WiFiInterface.cpp` driving the C6 with AT commands, with no ESP-Hosted calls anywhere in Meck's code.
- Checked on 7 November 2026: esptool 5.6.0 from PyPI needs Python 3.10 or newer; it installs the `esptool` command (with the commands `version`, `chip-id`, `read-flash` and `write-flash` used here), the `pyserial-miniterm` monitor, whose exit key is Control and ], and the `pyserial-ports` port list; its chip ID check prints `Chip type:` and the "no chip ID" warning as quoted in Part 4; `--dtr 0 --rts 0` opens the port with both reset lines in the state `idf.py monitor` leaves them in once it has opened the port (ESP-IDF's esp-idf-monitor, `serial_reader.py`); python.org lists macOS installers and, for Windows, the Python install manager (Python 3.15.2, 25 Oct 2026); in the Meck-P4 Bluetooth build, BLE Companion is off by default (`MeckDataStore.h`), the Settings row is **BLE Companion (tap to toggle)**, the home screen shows **BLE PIN:**, the P4 advertises as `MeshCore-` plus the node name, and the boot log prints one `C6:` line about the ESP-Hosted link.
- Windows steps, checked on 7 November 2026 against the documentation only: the venv and `Activate.ps1` commands and the `Set-ExecutionPolicy` line (Python 3.15.2 documentation, `venv`); the Python install manager and the `python` command installing the latest version when none is installed ("Using Python on Windows", Python 3.15.2 documentation); `Get-FileHash` using SHA-256 by default (Microsoft Learn); `Invoke-WebRequest -UseBasicParsing -OutFile` in Windows PowerShell 5.1 (Microsoft Learn); the adapter's name in Device Manager and the CH340 driver (SparkFun's CH340 driver guide).
- Found on an AMOLED LR2021 board on 6 November 2026: the C6 flash only works with the keyboard attachment removed, and the Part 5 backup takes several minutes.
- Found on an AMOLED LR2021 board on 6 and 7 November 2026: the MeshCore app pairs with the Bluetooth build using the PIN, and reconnects without it after a restart.
- Not yet tried on a P4: any of the Windows steps; the `esptool` command names in this version of the guide (earlier runs used `esptool.py` with `chip_id`, `read_flash` and `write_flash`); watching the P4 with `pyserial-miniterm` instead of `idf.py monitor`; writing a Meck-P4 Bluetooth build with esptool at `0x0` as in Part 7; the Bluetooth build on TFT boards and on the AMOLED SX1262 board.
- From work on a TFT SX1262 board in September 2026 (not re-checked on 6 November 2026): the wiring, button order, connector location and the TFT/AMOLED schematic comparison; the helper program's messages; the web flasher `invalid header` result; the failed full-size backup and `--baud 921600` result; the helper file overwriting Meck's settings area.