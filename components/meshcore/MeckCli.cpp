/*
 * MeckCli.cpp -- Companion CLI (MeshCore v1.18 companion command set)
 *
 * A port of upstream MeshCore v1.18's companion MyMesh::handleCommand() and
 * CommonRadioPrefs::handleCommand() (including upstream's later companion CLI
 * fixes e3d0f9fc and 5d82ed35), by way of the Meck Watch port, with the
 * replies word for word. Values map onto P4NodePrefs and are saved through
 * meck_cli_save_prefs() (NVS + SD backup).
 *
 * Shared by Settings > Experimental Features > Companion CLI and the app's
 * Command Line (command 66, MeckCompanion.cpp). Safe to call from the UI task
 * or meck_task: nothing here touches the radio or the C6 directly. A TX power
 * change goes through radio_request_reconfig(), which meck_task applies.
 *
 * P4 differences from upstream:
 *   - radio.rxgain and cad work on the LR2021 build only. On the SX1262
 *     build, LilyGo's cpp_bus_driver has no call for RX boosted gain or for
 *     starting a CAD scan, so their set commands answer "Error: unsupported"
 *     and change nothing, and get answers "> off". On the LR2021, "off" for
 *     radio.rxgain keeps RadioLib's own per-band gain levels (see
 *     P4LR2021Radio::setRxBoostedGain), and the gain change is queued through
 *     radio_request_rx_boosted_gain(), which meck_task applies.
 *   - poweroff / shutdown answer "Error: unsupported" (no software power-off
 *     on P4), as the Meck Watch does on boards without one.
 *   - set tx takes 0..22: P4 stores TX power unsigned and its radio code caps
 *     it at 22 dBm.
 *   - board answers "T-Display P4", the same name as the app's device info.
 *   - No variant hook (no MainBoard::handleCommand on P4), and no remote CLI
 *     from contacts.
 */

#include "MeckCli.h"
#include "MeckDataStore.h"
#include "meck.h"
#include "target.h"
#include <helpers/TxtDataHelpers.h>
#include <helpers/AdvertDataHelpers.h>
#include "esp_system.h"   // esp_restart()
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// Same firmware identity as the app's device info (MeckCompanion.cpp).
#ifndef FIRMWARE_BUILD_DATE
#define FIRMWARE_BUILD_DATE __DATE__
#endif
#ifndef FIRMWARE_VERSION
#define FIRMWARE_VERSION "v" MECK_FIRMWARE_VERSION "-p4"
#endif

#define MECK_CLI_BOARD_NAME "T-Display P4"

bool meck_cli_handle(const char* command, char* reply) {
    P4NodePrefs* p = meck_cli_prefs();
    if (!p || !command || !reply) return false;

    while (*command == ' ') command++; // skip leading spaces

    if (strlen(command) > 4 && command[2] == '|') { // optional prefix (for companion radio CLI)
        memcpy(reply, command, 3);                    // reflect the prefix back
        reply += 3;
        *reply = 0;
        command += 3;
    }

    // --- radio settings (upstream CommonRadioPrefs::handleCommand) ---
    if (strcmp(command, "get radio") == 0) {
        char freq[16], bw[16];
        strcpy(freq, StrHelper::ftoa(p->freq));
        strcpy(bw, StrHelper::ftoa3(p->bw));
        sprintf(reply, "> %s,%s,%d,%d", freq, bw, (int)p->sf, (int)p->cr);
        return true;
    }
    if (strncmp(command, "set radio ", 10) == 0) {
        char tmp[132];
        StrHelper::strncpy(tmp, &command[10], sizeof(tmp));   // bounded: the app frame can be longer than tmp
        const char *parts[4];
        int num = mesh::Utils::parseTextParts(tmp, parts, 4);
        float freq  = num > 0 ? strtof(parts[0], nullptr) : 0.0f;
        float bw    = num > 1 ? strtof(parts[1], nullptr) : 0.0f;
        uint8_t sf  = num > 2 ? atoi(parts[2]) : 0;
        uint8_t cr  = num > 3 ? atoi(parts[3]) : 0;
        // meck_radio_bw_allowed: on the LR2021 only its own bandwidth steps
        // pass, and bw becomes the exact step; SX1262 unchanged.
        if (freq >= 150.0f && freq <= 2500.0f && sf >= 5 && sf <= 12 && cr >= 5 && cr <= 8 && bw >= 7.0f && bw <= meck_radio_max_bw_khz() &&
            meck_radio_bw_allowed(bw, &bw)) {
            p->sf = sf;
            p->cr = cr;
            p->freq = freq;
            p->bw = bw;
            // LR2021 on 2.4 GHz: lower a stored TX above the band's limit
            // (as Settings does on the device) so the radio never gets a
            // power it refuses. TX is stored unsigned; compare it signed.
            int8_t max_tx = meck_radio_max_tx_dbm(freq);
            bool tx_lowered = (int8_t)p->tx_power_dbm > max_tx;
            if (tx_lowered) p->tx_power_dbm = (uint8_t)max_tx;
            meck_cli_save_prefs();
            if (tx_lowered) {
                sprintf(reply, "OK - TX lowered to %d dBm, reboot to apply", (int)max_tx);
            } else {
                strcpy(reply, "OK - reboot to apply");
            }
        } else {
            strcpy(reply, "Error, invalid radio params");
        }
        return true;
    }

    if (strcmp(command, "get freq") == 0) {
        sprintf(reply, "> %s", StrHelper::ftoa(p->freq));
        return true;
    }

    if (strcmp(command, "get af") == 0) {
        sprintf(reply, "> %s", StrHelper::ftoa(p->airtime_factor));
        return true;
    }
    if (strncmp(command, "set af ", 7) == 0) {
        char* end;
        float af = strtof(&command[7], &end);
        if (end == &command[7] || af < 0 || af > 9) {
            strcpy(reply, "ERROR: af must be 0-9");
        } else {
            p->airtime_factor = af;
            meck_cli_save_prefs();
            strcpy(reply, "OK");
        }
        return true;
    }

    if (strcmp(command, "get dutycycle") == 0) {
        float dc = 100.0f / (p->airtime_factor + 1.0f);
        int dc_int = (int)dc;
        int dc_frac = (int)((dc - dc_int) * 10.0f + 0.5f);
        sprintf(reply, "> %d.%d%%", dc_int, dc_frac);
        return true;
    }
    if (strncmp(command, "set dutycycle ", 14) == 0) {
        float dc = atof(&command[14]);
        if (dc < 1 || dc > 100) {
            strcpy(reply, "ERROR: dutycycle must be 1-100");
        } else {
            p->airtime_factor = (100.0f / dc) - 1.0f;
            meck_cli_save_prefs();
            float actual = 100.0f / (p->airtime_factor + 1.0f);
            int a_int = (int)actual;
            int a_frac = (int)((actual - a_int) * 10.0f + 0.5f);
            sprintf(reply, "OK - %d.%d%%", a_int, a_frac);
        }
        return true;
    }

    if (strcmp(command, "get int.thresh") == 0) {
        sprintf(reply, "> %d", (int)p->interference_threshold);
        return true;
    }
    if (strncmp(command, "set int.thresh ", 15) == 0) {
        p->interference_threshold = atoi(&command[15]);
        meck_cli_save_prefs();
        strcpy(reply, "OK");
        return true;
    }

    // cad and radio.rxgain: upstream CommonRadioPrefs::handleCommand,
    // LR2021 build only (meck_radio_has_cad_rxgain).
    if (strcmp(command, "get cad") == 0) {
        sprintf(reply, "> %s", p->cad_enabled ? "on" : "off");
        return true;
    }
    if (strncmp(command, "set cad ", 8) == 0) {
        if (!meck_radio_has_cad_rxgain()) {
            strcpy(reply, "Error: unsupported");
            return true;
        }
        p->cad_enabled = (strncmp(&command[8], "on", 2) == 0) ? 1 : 0;
        meck_cli_save_prefs();
        strcpy(reply, "OK");
        return true;
    }

    if (strcmp(command, "get radio.rxgain") == 0) {
        sprintf(reply, "> %s", p->rx_boosted_gain != 0 ? "on" : "off");
        return true;
    }
    if (strncmp(command, "set radio.rxgain ", 17) == 0) {
        bool enabled = strncmp(&command[17], "on", 2) == 0;
        if (!meck_radio_has_cad_rxgain()) {
            strcpy(reply, "Error: unsupported");
            return true;
        }
        p->rx_boosted_gain = enabled ? 1 : 0;
        meck_cli_save_prefs();
        radio_request_rx_boosted_gain(enabled);
        strcpy(reply, "OK");
        return true;
    }

    if (strncmp(command, "get tx", 6) == 0 && (command[6] == 0 || command[6] == ' ')) {
        sprintf(reply, "> %d", (int)(int8_t)p->tx_power_dbm);   // stored unsigned; the app can set -9..-1
        return true;
    }
    if (strncmp(command, "set tx ", 7) == 0) {
        int dbm = atoi(&command[7]);
        int max_tx = meck_radio_max_tx_dbm(p->freq);   // 22, or 12 on the LR2021 at 2.4 GHz
        if (dbm < 0 || dbm > max_tx) {
            sprintf(reply, "Error, must be 0-%d", max_tx);
        } else {
            p->tx_power_dbm = (uint8_t)dbm;
            meck_cli_save_prefs();
            radio_request_reconfig(p->freq, p->bw, p->sf, p->cr, p->tx_power_dbm);
            strcpy(reply, "OK");
        }
        return true;
    }

    if (strcmp(command, "get rxdelay") == 0) {
        sprintf(reply, "> %s", StrHelper::ftoa(p->rx_delay_base));
        return true;
    }
    if (strncmp(command, "set rxdelay ", 12) == 0) {
        float db = atof(&command[12]);
        if (db >= 0 && db <= 20.0f) {
            p->rx_delay_base = db;
            meck_cli_save_prefs();
            strcpy(reply, "OK");
        } else {
            strcpy(reply, "Error, must be 0-20");
        }
        return true;
    }

    if (strcmp(command, "get agc.reset.interval") == 0) {
        sprintf(reply, "> %d", (int)p->agc_reset_interval * 4);
        return true;
    }
    if (strncmp(command, "set agc.reset.interval ", 23) == 0) {
        int secs = atoi(&command[23]);
        if (secs < 0) secs = 0;
        if (secs > 255 * 4) secs = 255 * 4;
        p->agc_reset_interval = (uint8_t)(secs / 4);   // stored as seconds / 4
        meck_cli_save_prefs();
        sprintf(reply, "OK - interval rounded to %d", (int)p->agc_reset_interval * 4);
        return true;
    }

    if (strcmp(command, "get path.hash.mode") == 0) {
        sprintf(reply, "> %d", (int)p->path_hash_mode);
        return true;
    }
    if (strncmp(command, "set path.hash.mode ", 19) == 0) {
        const char* config = command + 19;
        uint8_t mode = atoi(config);
        if (mode < 3) {
            p->path_hash_mode = mode;
            meck_cli_save_prefs();
            strcpy(reply, "OK");
        } else {
            strcpy(reply, "Error, must be 0,1, or 2");
        }
        return true;
    }

    if (strcmp(command, "get multi.acks") == 0) {
        sprintf(reply, "> %d", (int)p->multi_acks);
        return true;
    }
    if (strncmp(command, "set multi.acks ", 15) == 0) {
        p->multi_acks = atoi(&command[15]);
        meck_cli_save_prefs();
        strcpy(reply, "OK");
        return true;
    }

    if (strcmp(command, "get txdelay") == 0) {
        sprintf(reply, "> %s", StrHelper::ftoa(p->tx_delay_factor));
        return true;
    }
    if (strncmp(command, "set txdelay ", 12) == 0) {
        float f = atof(&command[12]);
        if (f >= 0 && f <= 2.0f) {
            p->tx_delay_factor = f;
            meck_cli_save_prefs();
            strcpy(reply, "OK");
        } else {
            strcpy(reply, "Error, must be 0-2");
        }
        return true;
    }

    if (strcmp(command, "get direct.txdelay") == 0) {
        sprintf(reply, "> %s", StrHelper::ftoa(p->direct_tx_delay_factor));
        return true;
    }
    if (strncmp(command, "set direct.txdelay ", 19) == 0) {
        float f = atof(&command[19]);
        if (f >= 0 && f <= 2.0f) {
            p->direct_tx_delay_factor = f;
            meck_cli_save_prefs();
            strcpy(reply, "OK");
        } else {
            strcpy(reply, "Error, must be 0-2");
        }
        return true;
    }

    // --- device commands (upstream MyMesh::handleCommand) ---
    if (strcmp(command, "reboot") == 0) {
        esp_restart(); // doesn't return
        return true;
    }

    if (strcmp(command, "poweroff") == 0 || strcmp(command, "shutdown") == 0) {
        strcpy(reply, "Error: unsupported");   // no software power-off on P4
        return true;
    }

    if (strncmp(command, "set name ", 9) == 0) {
        if (AdvertDataParser::isValidName(&command[9])) {
            StrHelper::strncpy(p->node_name, &command[9], sizeof(p->node_name));
            meck_cli_save_prefs();
            strcpy(reply, "OK");
        } else {
            strcpy(reply, "Error, bad chars");
        }
        return true;
    }
    if (strcmp(command, "get name") == 0) {
        sprintf(reply, "> %s", p->node_name);
        return true;
    }

    if (strncmp(command, "set pin ", 8) == 0) {
        p->ble_pin = atoi(&command[8]);
        meck_cli_save_prefs();
        sprintf(reply, "> pin is now %06d", (int)p->ble_pin);
        return true;
    }

    // WiFi (upstream's ENABLE_WIFI_INTERFACE block). P4 starts WiFi at boot
    // when wifi_enabled is set and an SSID is stored (meck_app_init).
    if (strncmp(command, "set wifi.ssid ", 14) == 0) {
        StrHelper::strncpy(p->wifi_ssid, &command[14], sizeof(p->wifi_ssid));
        meck_cli_save_prefs();
        sprintf(reply, "> wifi.ssid is now %s (set wifi.pwd too, then reboot)", p->wifi_ssid);
        return true;
    }
    if (strncmp(command, "set wifi.pwd ", 13) == 0) {
        StrHelper::strncpy(p->wifi_password, &command[13], sizeof(p->wifi_password));
        meck_cli_save_prefs();
        strcpy(reply, "> wifi.pwd updated (reboot to apply)");
        return true;
    }
    if (strcmp(command, "get wifi.pwd") == 0) {
        sprintf(reply, "> %s", p->wifi_password);
        return true;
    }
    if (strcmp(command, "set wifi.clear") == 0) {
        p->wifi_ssid[0] = 0;
        p->wifi_password[0] = 0;
        meck_cli_save_prefs();
        strcpy(reply, "> wifi config cleared (reboot to apply)");
        return true;
    }
    if (strcmp(command, "get wifi.ssid") == 0) {
        sprintf(reply, "> %s", p->wifi_ssid[0] ? p->wifi_ssid : "(not set)");
        return true;
    }
    if (strncmp(command, "set wifi.enabled ", 17) == 0) {
        p->wifi_enabled = atoi(&command[17]) ? 1 : 0;
        meck_cli_save_prefs();
        sprintf(reply, "> wifi.enabled is now %d (reboot to apply)", (int)p->wifi_enabled);
        return true;
    }
    if (strcmp(command, "get wifi.enabled") == 0) {
        sprintf(reply, "> %d", (int)p->wifi_enabled);
        return true;
    }
    if (strcmp(command, "get wifi.status") == 0) {
        strcpy(reply, meck_wifi_is_connected() ? "> connected" : "> disconnected");
        return true;
    }
    if (strcmp(command, "get wifi.ip") == 0) {
        if (meck_wifi_is_connected()) {
            const char* ip = meck_wifi_get_ip();
            sprintf(reply, "> %s", ip ? ip : "");
        } else {
            strcpy(reply, "> (not connected)");
        }
        return true;
    }

    if (strcmp(command, "board") == 0) {
        strcpy(reply, MECK_CLI_BOARD_NAME);
        return true;
    }

    if (strcmp(command, "ver") == 0) {
        sprintf(reply, "%s (Build: %s)", FIRMWARE_VERSION, FIRMWARE_BUILD_DATE);
        return true;
    }

    if (strcmp(command, "get tz.offset") == 0) {
        sprintf(reply, "> %d", (int)p->utc_offset_hours);
        return true;
    }
    if (strncmp(command, "set tz.offset ", 14) == 0) {
        int8_t tz = atof(&command[14]);
        if (tz < -12 || tz > 14) {
            strcpy(reply, "Error, must be from -12 to +14");
        } else {
            p->utc_offset_hours = tz;
            meck_cli_save_prefs();
            strcpy(reply, "OK");
        }
        return true;
    }

    return false;  // not handled
}
