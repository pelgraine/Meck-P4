/*
 * MeckCompanion.cpp -- MeshCore companion protocol handler for T-Display P4
 *
 * Ported from upstream MyMesh::handleCmdFrame() / checkSerialInterface().
 * Implements the subset of the companion protocol needed for the phone app
 * to connect, sync contacts/channels, and exchange messages.
 */

#include "MeckCompanion.h"
#include "MeckCli.h"
#include "meck.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <helpers/TxtDataHelpers.h>

// strncpy with guaranteed NUL termination (replaces strzcpy)
static void strzcpy(char* dest, const char* src, size_t n) {
    strncpy(dest, src, n - 1);
    dest[n - 1] = '\0';
}

// ---- Client repeat mode (upstream MeshCore v9), as the Meck Watch has it ----
// Frequencies (kHz ranges) on which the app may switch client repeat on.
// Upstream's three exact rows, plus the Meck Watch's band rows so it can be
// switched on at the preset frequencies, including the 2.4 GHz ISM band
// (LR2021 build).
struct FreqRange {
    uint32_t lower_freq, upper_freq;
};

static const FreqRange repeat_freq_ranges[] = {
    { 433000, 433000 },
    { 869495, 869495 },
    { 918000, 918000 },
    { 433050, 434790 },    // 433 MHz ISM -- covers the 433.375 / 433.650 presets
    { 863000, 870000 },    // EU 863-870 SRD -- covers 869.432 / 869.525 / 869.618
    { 902000, 928000 },    // 902-928 MHz ISM (AU/NZ/US) -- covers 910.525 through 923.125, incl. Sydney 916.575 / 915.075
    { 2400000, 2500000 }   // 2.4 GHz ISM band (LR2021 build)
};

static bool isValidClientRepeatFreq(uint32_t f) {
    for (size_t i = 0; i < sizeof(repeat_freq_ranges) / sizeof(repeat_freq_ranges[0]); i++) {
        if (f >= repeat_freq_ranges[i].lower_freq && f <= repeat_freq_ranges[i].upper_freq) return true;
    }
    return false;
}

// ---- Command codes (from upstream MyMesh.cpp) ----
#define CMD_APP_START                 1
#define CMD_SEND_TXT_MSG              2
#define CMD_SEND_CHANNEL_TXT_MSG      3
#define CMD_GET_CONTACTS              4
#define CMD_GET_DEVICE_TIME           5
#define CMD_SET_DEVICE_TIME           6
#define CMD_SEND_SELF_ADVERT          7
#define CMD_SET_ADVERT_NAME           8
#define CMD_SET_ADVERT_LATLON         14
#define CMD_SET_OTHER_PARAMS          38
#define CMD_GET_CUSTOM_VARS           40
#define CMD_SET_CUSTOM_VAR            41
#define CMD_GET_ADVERT_PATH           42
#define CMD_ADD_UPDATE_CONTACT        9
#define CMD_SYNC_NEXT_MESSAGE         10
#define CMD_SET_RADIO_PARAMS          11
#define CMD_SET_RADIO_TX_POWER        12
#define CMD_RESET_PATH                13
#define CMD_REMOVE_CONTACT            15
#define CMD_EXPORT_PRIVATE_KEY        23
#define CMD_IMPORT_PRIVATE_KEY        24
#define CMD_GET_CONTACT_BY_KEY        30
#define CMD_REBOOT                    19
#define CMD_GET_BATT_AND_STORAGE      20
#define CMD_DEVICE_QEURY              22
#define CMD_GET_CHANNEL               31
#define CMD_SET_CHANNEL               32
#define CMD_GET_AUTOADD_CONFIG        59
#define CMD_GET_ALLOWED_REPEAT_FREQ   60   // v9+, client repeat frequency whitelist
#define CMD_SET_PATH_HASH_MODE        61
#define CMD_SET_DEFAULT_FLOOD_SCOPE   63
#define CMD_GET_DEFAULT_FLOOD_SCOPE   64
#define CMD_RUN_CLI_COMMAND           66   // v14+, the app's Command Line
#define CMD_SHARE_CONTACT             16   // re-send a contact's advert zero-hop
#define CMD_SET_TUNING_PARAMS         21   // rx delay base + airtime factor (x1000)
#define CMD_GET_TUNING_PARAMS         43
#define CMD_EXPORT_CONTACT            17
#define CMD_IMPORT_CONTACT            18
#define CMD_SEND_RAW_DATA             25
#define CMD_SIGN_START                33
#define CMD_SIGN_DATA                 34
#define CMD_SIGN_FINISH               35
#define CMD_SEND_TRACE_PATH           36
#define CMD_SEND_TELEMETRY_REQ        39
#define CMD_SEND_PATH_DISCOVERY_REQ   52
#define CMD_SET_AUTOADD_CONFIG        58
#define CMD_SEND_CHANNEL_DATA         62
#define CMD_SEND_RAW_PACKET           65   // as Meck Watch: transmit a complete packet from the app

// ---- Response codes ----
#define RESP_CODE_OK                  0
#define RESP_CODE_ERR                 1
#define RESP_CODE_CONTACTS_START      2
#define RESP_CODE_CONTACT             3
#define RESP_CODE_END_OF_CONTACTS     4
#define RESP_CODE_SELF_INFO           5
#define RESP_CODE_SENT                6
#define RESP_CODE_CURR_TIME           9
#define RESP_CODE_NO_MORE_MESSAGES    10
#define RESP_CODE_BATT_AND_STORAGE    12
#define RESP_CODE_DEVICE_INFO         13
#define RESP_CODE_DISABLED            15
#define RESP_CODE_PRIVATE_KEY         14
#define RESP_CODE_CUSTOM_VARS         21
#define RESP_CODE_ADVERT_PATH         22
#define RESP_CODE_CHANNEL_INFO        18
#define RESP_CODE_AUTOADD_CONFIG      25
#define RESP_ALLOWED_REPEAT_FREQ      26
#define RESP_CODE_DEFAULT_FLOOD_SCOPE 28
#define RESP_CODE_CLI_REPLY           29   // v14+, a reply to CMD_RUN_CLI_COMMAND
#define RESP_CODE_TUNING_PARAMS       23   // reply to CMD_GET_TUNING_PARAMS
#define RESP_CODE_EXPORT_CONTACT      11
#define RESP_CODE_SIGN_START          19
#define RESP_CODE_SIGNATURE           20
#define RESP_CODE_CHANNEL_DATA_RECV   27   // queued like a message

#define MAX_SIGN_DATA_LEN             (8 * 1024) // 8K
#define MAX_CHANNEL_DATA_LENGTH       (MAX_FRAME_SIZE - 9)

// ---- Error codes ----
#define ERR_CODE_UNSUPPORTED_CMD      1
#define ERR_CODE_NOT_FOUND            2
#define ERR_CODE_TABLE_FULL           3
#define ERR_CODE_BAD_STATE            4
#define ERR_CODE_ILLEGAL_ARG          6

// ---- Push codes ----
#define PUSH_CODE_ADVERT              0x80
#define PUSH_CODE_PATH_UPDATED        0x81
#define PUSH_CODE_SEND_CONFIRMED      0x82
#define PUSH_CODE_MSG_WAITING         0x83
#define PUSH_CODE_LOG_RX_DATA         0x88
#define PUSH_CODE_NEW_ADVERT          0x8A
#define PUSH_CODE_RAW_DATA            0x84
#define PUSH_CODE_TRACE_DATA          0x89
#define PUSH_CODE_TELEMETRY_RESPONSE  0x8B
#define PUSH_CODE_PATH_DISCOVERY_RESPONSE 0x8D
#define PUSH_CODE_CONTACT_DELETED     0x8F // oldest contact overwritten
#define PUSH_CODE_CONTACTS_FULL       0x90 // contacts storage is full

// ---- Response codes for received messages (queued in offline queue) ----
#define RESP_CODE_CONTACT_MSG_RECV    7
#define RESP_CODE_CHANNEL_MSG_RECV    8
#define RESP_CODE_CONTACT_MSG_RECV_V3 16
#define RESP_CODE_CHANNEL_MSG_RECV_V3 17

// ---- Additional commands ----
#define CMD_SET_FLOOD_SCOPE_KEY       54
#define CMD_SEND_CONTROL_DATA         55   // v8+, e.g. the app's Discover nodes
#define CMD_SEND_ANON_REQ             57   // v13+, requests to nodes that aren't contacts
#define PUSH_CODE_CONTROL_DATA        0x8E // v8+, received control packet

#if MECK_BLE_GARMIN
// ---- Garmin watch app (Meck-Garmin): the full Meck watch extension, same
// codes and layouts as Meck's MECK_WATCH_EXT (T-Deck Max Garmin build), so
// the watch treats the P4 as a Meck node.
#define CMD_WATCH_HELLO               0x70  // [ver] -> RESP_CODE_WATCH_INFO; marks this client as the watch
#define CMD_WATCH_GET_CANNED          0x71  // [slot] -> RESP_CODE_WATCH_CANNED
#define CMD_WATCH_GET_SENT_TRACK      0x72  // [n: 0 = most recent] -> RESP_CODE_WATCH_SENT_TRACK
#define CMD_WATCH_GET_SCOPE           0x73  // [idx] -> RESP_CODE_WATCH_SCOPE
#define CMD_WATCH_DISCOVER_START      0x74  // -> OK; runs the P4's own 30 s discovery scan
#define CMD_WATCH_DISCOVER_GET        0x75  // [n] -> RESP_CODE_WATCH_DISCOVER
#define CMD_WATCH_GET_FAV             0x76  // [n] -> RESP_CODE_WATCH_FAV (n-th favourite contact)
#define CMD_WATCH_RESOLVE             0x77  // [plen][pubkey prefix] -> RESP_CODE_WATCH_RESOLVE
#define RESP_CODE_WATCH_INFO          0x60  // [ext_ver][canned_slots][bph][scope_count]
#define RESP_CODE_WATCH_CANNED        0x61  // [slot][text] (empty text = unused slot)
#define RESP_CODE_WATCH_SENT_TRACK    0x62  // [n][active][fingerprint*12][repeats][bph][count][hash*bph*count][snr*count]
#define RESP_CODE_WATCH_CHANNEL_MSG   0x63  // [snr*4][scope_idx][path bytes count][channel_idx][txt_type][timestamp:4][path bytes][text]
#define RESP_CODE_WATCH_SCOPE         0x64  // [idx][name] (no name = no such scope)
#define RESP_CODE_WATCH_DISCOVER      0x65  // [n][active][count] then, if n < count: [type][snr*4][path_len][known][pubkey:2][name]
#define RESP_CODE_WATCH_FAV           0x66  // [n][count] then, if n < count: [type][last_advert:4][pubkey:32][name]
#define RESP_CODE_WATCH_RESOLVE       0x67  // [plen][prefix][name] (no name = not found)
#define WATCH_EXT_VER                 1
#endif

// ---- Repeater admin commands (over-the-mesh login + status + session) ----
#define CMD_SEND_LOGIN                26
#define CMD_SEND_STATUS_REQ           27
#define CMD_HAS_CONNECTION            28
#define CMD_LOGOUT                    29
#define CMD_SEND_BINARY_REQ           50

// ---- Repeater admin push codes ----
#define PUSH_CODE_LOGIN_SUCCESS       0x85
#define PUSH_CODE_LOGIN_FAIL          0x86
#define PUSH_CODE_STATUS_RESPONSE     0x87
#define PUSH_CODE_BINARY_RESPONSE     0x8C

// ---- Firmware identity ----
// Must match MeckMesh.h / upstream FIRMWARE_VER_CODE so the app
// negotiates the right protocol level.
#ifndef FIRMWARE_VER_CODE
#define FIRMWARE_VER_CODE 14   // v14: the app's Command Line (CMD_RUN_CLI_COMMAND)
#endif
#ifndef FIRMWARE_BUILD_DATE
#define FIRMWARE_BUILD_DATE __DATE__
#endif
#ifndef FIRMWARE_VERSION
#define FIRMWARE_VERSION "v" MECK_FIRMWARE_VERSION "-p4"
#endif

// ---------------------------------------------------------------------------
// Response helpers
// ---------------------------------------------------------------------------

void MeckCompanion::writeOKFrame() {
    uint8_t buf[1] = { RESP_CODE_OK };
    _serial->writeFrame(buf, 1);
}

void MeckCompanion::writeErrFrame(uint8_t err_code) {
    uint8_t buf[2] = { RESP_CODE_ERR, err_code };
    _serial->writeFrame(buf, 2);
}

void MeckCompanion::writeContactRespFrame(uint8_t code, const ContactInfo& contact) {
    int i = 0;
    _out[i++] = code;
    memcpy(&_out[i], contact.id.pub_key, PUB_KEY_SIZE); i += PUB_KEY_SIZE;
    _out[i++] = contact.type;
    _out[i++] = contact.flags;
    _out[i++] = contact.out_path_len;
    memcpy(&_out[i], contact.out_path, MAX_PATH_SIZE); i += MAX_PATH_SIZE;
    strzcpy((char*)&_out[i], contact.name, 32); i += 32;
    memcpy(&_out[i], &contact.last_advert_timestamp, 4); i += 4;
    memcpy(&_out[i], &contact.gps_lat, 4); i += 4;
    memcpy(&_out[i], &contact.gps_lon, 4); i += 4;
    memcpy(&_out[i], &contact.lastmod, 4); i += 4;
    _serial->writeFrame(_out, i);
}

// Fill a contact from a CMD_ADD_UPDATE_CONTACT frame (as Meck's
// MyMesh::updateContactFromFrame): type, flags, out path, name, last advert
// time, then optionally position and last-modified time.
static void updateContactFromFrame(ContactInfo& contact, uint32_t& last_mod,
                                   const uint8_t* frame, int len) {
    int i = 1;  // frame[0] is the command code
    memcpy(contact.id.pub_key, &frame[i], PUB_KEY_SIZE); i += PUB_KEY_SIZE;
    contact.type = frame[i++];
    contact.flags = frame[i++];
    contact.out_path_len = frame[i++];
    memcpy(contact.out_path, &frame[i], MAX_PATH_SIZE); i += MAX_PATH_SIZE;
    memcpy(contact.name, &frame[i], 32); i += 32;
    memcpy(&contact.last_advert_timestamp, &frame[i], 4); i += 4;
    if (len >= i + 8) {  // optional fields
        memcpy(&contact.gps_lat, &frame[i], 4); i += 4;
        memcpy(&contact.gps_lon, &frame[i], 4); i += 4;
        if (len >= i + 4) {
            memcpy(&last_mod, &frame[i], 4);
        }
    }
}

// ---------------------------------------------------------------------------
// check() -- main entry point, called every meck_task iteration
// ---------------------------------------------------------------------------

void MeckCompanion::check() {
    if (!_mesh || !_serial) return;
    if (!_serial->isEnabled()) return;

    size_t len = _serial->checkRecvFrame(_cmd);
    if (len > 0) {
        handleCmdFrame(len);
    } else if (_iter_started && !_serial->isWriteBusy()) {
        // Continue contact iteration (streaming, one per loop tick)
        ContactInfo contact;
        if (_iter.hasNext(_mesh, contact)) {
            if (contact.lastmod > _iter_filter_since) {
                writeContactRespFrame(RESP_CODE_CONTACT, contact);
                if (contact.lastmod > _most_recent_lastmod) {
                    _most_recent_lastmod = contact.lastmod;
                }
            }
        } else {
            // End of contacts
            _out[0] = RESP_CODE_END_OF_CONTACTS;
            memcpy(&_out[1], &_most_recent_lastmod, 4);
            _serial->writeFrame(_out, 5);
            _iter_started = false;
            _serial->setFastMode(false);
        }
    }
}

// ---------------------------------------------------------------------------
// handleCmdFrame() -- dispatch incoming command
// ---------------------------------------------------------------------------

void MeckCompanion::handleCmdFrame(size_t len) {
    P4NodePrefs* prefs = _mesh->getNodePrefs();
    if (!prefs) return;

    uint8_t cmd = _cmd[0];

    // ---- CMD_DEVICE_QEURY (22) ----
    // App sends this first to identify firmware capabilities.
    if (cmd == CMD_DEVICE_QEURY && len >= 2) {
        _app_target_ver = _cmd[1];

        int i = 0;
        _out[i++] = RESP_CODE_DEVICE_INFO;
        _out[i++] = FIRMWARE_VER_CODE;
        _out[i++] = (uint8_t)((MAX_CONTACTS / 2) > 255 ? 255 : MAX_CONTACTS / 2);
        _out[i++] = MAX_GROUP_CHANNELS;
        uint32_t pin = prefs->ble_pin;
        memcpy(&_out[i], &pin, 4); i += 4;
        memset(&_out[i], 0, 12);
        strncpy((char*)&_out[i], FIRMWARE_BUILD_DATE, 12); i += 12;
        strzcpy((char*)&_out[i], "T-Display P4", 40); i += 40;
        strzcpy((char*)&_out[i], FIRMWARE_VERSION, 20); i += 20;
        _out[i++] = prefs->client_repeat ? 1 : 0;  // client_repeat (v9+)
        _out[i++] = prefs->path_hash_mode;  // v10+
        _serial->writeFrame(_out, i);
        printf("Companion: DEVICE_QUERY from app ver %d\n", _app_target_ver);
        return;
    }

    // ---- CMD_APP_START (1) ----
    // App sends this after DEVICE_QUERY. Respond with our identity.
    if (cmd == CMD_APP_START && len >= 8) {
        char* app_name = (char*)&_cmd[8];
        _cmd[len] = 0;
        printf("Companion: APP_START from '%s'\n", app_name);

        _iter_started = false;
#if MECK_BLE_GARMIN
        _watch_client = false;   // new client; the watch re-declares itself with CMD_WATCH_HELLO
#endif

        int i = 0;
        _out[i++] = RESP_CODE_SELF_INFO;
        _out[i++] = 1;  // ADV_TYPE_CHAT
        _out[i++] = prefs->tx_power_dbm;
        _out[i++] = prefs->tx_power_dbm;  // MAX_LORA_TX_POWER = same on P4
        memcpy(&_out[i], _mesh->getIdentity().pub_key, PUB_KEY_SIZE); i += PUB_KEY_SIZE;

        int32_t lat = prefs->position_lat_e7 / 10;  // e7 -> e6
        int32_t lon = prefs->position_lon_e7 / 10;
        memcpy(&_out[i], &lat, 4); i += 4;
        memcpy(&_out[i], &lon, 4); i += 4;
        _out[i++] = prefs->multi_acks;
        _out[i++] = prefs->position_mode > 0 ? 1 : 0;  // advert_loc_policy
        _out[i++] = 0;  // telemetry mode
        _out[i++] = prefs->manual_add_contacts;

        uint32_t freq = (uint32_t)(prefs->freq * 1000);
        memcpy(&_out[i], &freq, 4); i += 4;
        uint32_t bw = (uint32_t)(prefs->bw * 1000);
        memcpy(&_out[i], &bw, 4); i += 4;
        _out[i++] = prefs->sf;
        _out[i++] = prefs->cr;

        int nlen = strlen(prefs->node_name);
        memcpy(&_out[i], prefs->node_name, nlen); i += nlen;
        _serial->writeFrame(_out, i);
        return;
    }

    // ---- CMD_GET_CONTACTS (4) ----
    if (cmd == CMD_GET_CONTACTS) {
        if (_iter_started) {
            writeErrFrame(ERR_CODE_BAD_STATE);
        } else {
            if (len >= 5) {
                memcpy(&_iter_filter_since, &_cmd[1], 4);
            } else {
                _iter_filter_since = 0;
            }

            uint8_t reply[5];
            reply[0] = RESP_CODE_CONTACTS_START;
            uint32_t count = _mesh->getNumContacts();
            memcpy(&reply[1], &count, 4);
            _serial->writeFrame(reply, 5);

            _iter = _mesh->startContactsIterator();
            _iter_started = true;
            _most_recent_lastmod = 0;
            _serial->setFastMode(true);
        }
        return;
    }

    // ---- CMD_GET_CHANNEL (31) ----
    if (cmd == CMD_GET_CHANNEL && len >= 2) {
        _serial->setFastMode(true);
        uint8_t channel_idx = _cmd[1];
        ChannelDetails channel;
        if (_mesh->getChannel(channel_idx, channel)) {
            int i = 0;
            _out[i++] = RESP_CODE_CHANNEL_INFO;
            _out[i++] = channel_idx;
            memset(&_out[i], 0, 32);
            strncpy((char*)&_out[i], channel.name, 31);
            i += 32;
            memcpy(&_out[i], channel.channel.secret, 16); i += 16;
            _serial->writeFrame(_out, i);
        } else {
            writeErrFrame(ERR_CODE_NOT_FOUND);
        }
        return;
    }

    // ---- CMD_SYNC_NEXT_MESSAGE (10) ----
    if (cmd == CMD_SYNC_NEXT_MESSAGE) {
        int out_len = getFromOfflineQueue(_out);
        if (out_len > 0) {
#if MECK_BLE_GARMIN
            if (_out[0] == RESP_CODE_WATCH_CHANNEL_MSG && out_len >= 10) {
                // Queued while the watch was the client. The txt_type slot
                // holds the packet's encoded path_len until now (see
                // pushChannelMessage).
                uint8_t enc_path_len = _out[5];
                if (_watch_client) {
                    _out[5] = TXT_TYPE_PLAIN;
                } else {
                    // Rebuild as a standard frame for this client (V3 if it
                    // supports it, else V1) and drop the hop list, as Meck.
                    uint8_t snr = _out[1];
                    uint8_t plen_byte = _out[3];
                    uint8_t ch = _out[4];
                    uint8_t ts[4];
                    memcpy(ts, &_out[6], 4);
                    int plen = (plen_byte != 0xFF) ? plen_byte : 0;
                    int text_off = 10 + plen;
                    int tlen = out_len - text_off;
                    if (tlen < 0) tlen = 0;
                    // Move the text to where it sits in the standard frame
                    // (header: 11 bytes for V3, 8 for V1), then write the header.
                    int hdr = (_app_target_ver >= 3) ? 11 : 8;
                    if (hdr + tlen > MAX_FRAME_SIZE) tlen = MAX_FRAME_SIZE - hdr;
                    memmove(&_out[hdr], &_out[text_off], tlen);
                    int i = 0;
                    if (_app_target_ver >= 3) {
                        _out[i++] = RESP_CODE_CHANNEL_MSG_RECV_V3;
                        _out[i++] = snr;
                        _out[i++] = 0;  // reserved1
                        _out[i++] = 0;  // reserved2
                    } else {
                        _out[i++] = RESP_CODE_CHANNEL_MSG_RECV;
                    }
                    _out[i++] = ch;
                    _out[i++] = (plen_byte != 0xFF) ? enc_path_len : 0xFF;
                    _out[i++] = TXT_TYPE_PLAIN;
                    memcpy(&_out[i], ts, 4); i += 4;
                    out_len = i + tlen;
                }
            }
#endif
            _serial->writeFrame(_out, out_len);
        } else {
            _out[0] = RESP_CODE_NO_MORE_MESSAGES;
            _serial->writeFrame(_out, 1);
#if MECK_BLE_ENABLED
            // The app now has every message that was waiting for it.
            if (_on_app_synced && _serial->isConnected()) _on_app_synced();
#endif
        }
        return;
    }

    // ---- CMD_GET_DEVICE_TIME (5) ----
    if (cmd == CMD_GET_DEVICE_TIME) {
        _out[0] = RESP_CODE_CURR_TIME;
        uint32_t now = _mesh->getRTCClock()->getCurrentTime();
        memcpy(&_out[1], &now, 4);
        _serial->writeFrame(_out, 5);
        return;
    }

    // ---- CMD_SET_DEVICE_TIME (6) ----
    if (cmd == CMD_SET_DEVICE_TIME && len >= 5) {
        uint32_t secs;
        memcpy(&secs, &_cmd[1], 4);
        uint32_t curr = _mesh->getRTCClock()->getCurrentTime();
        printf("Companion: app sent its time %lu, P4 clock %lu: %s\n",
               (unsigned long)secs, (unsigned long)curr,
               secs >= curr ? "taken" : "refused (earlier than the P4 clock)");
        if (secs >= curr) {
            _mesh->getRTCClock()->setCurrentTime(secs);
            writeOKFrame();
        } else {
            writeErrFrame(ERR_CODE_ILLEGAL_ARG);
        }
        return;
    }

    // ---- CMD_GET_BATT_AND_STORAGE (20) ----
    if (cmd == CMD_GET_BATT_AND_STORAGE) {
        int i = 0;
        _out[i++] = RESP_CODE_BATT_AND_STORAGE;
        uint16_t batt_mv = meck_battery_get_mv();
        uint32_t used = 0, total = 0;  // TODO: wire SD card usage
        memcpy(&_out[i], &batt_mv, 2); i += 2;
        memcpy(&_out[i], &used, 4); i += 4;
        memcpy(&_out[i], &total, 4); i += 4;
        _serial->writeFrame(_out, i);
        return;
    }

    // ---- CMD_SEND_TXT_MSG (2) ---- send a direct message
    if (cmd == CMD_SEND_TXT_MSG && len >= 14) {
        int i = 1;
        uint8_t txt_type = _cmd[i++];
        uint8_t attempt = _cmd[i++];
        uint32_t msg_timestamp;
        memcpy(&msg_timestamp, &_cmd[i], 4); i += 4;
        uint8_t* pub_key_prefix = &_cmd[i]; i += 6;
        ContactInfo* recipient = _mesh->lookupContactByPubKey(pub_key_prefix, 6);
        if (recipient && (txt_type == TXT_TYPE_PLAIN || txt_type == TXT_TYPE_CLI_DATA || txt_type == TXT_TYPE_CLI_COMMAND)) {
            char* text = (char*)&_cmd[i];
            _cmd[len] = 0;  // ensure NUL
            uint32_t est_timeout = 0;
            uint32_t expected_ack = 0;
            int result;
            if (txt_type == TXT_TYPE_CLI_DATA || txt_type == TXT_TYPE_CLI_COMMAND) {
                msg_timestamp = _mesh->getRTCClock()->getCurrentTimeUnique();
                result = _mesh->sendCommandData(*recipient, msg_timestamp, attempt, txt_type, text, est_timeout);
                expected_ack = 0;
            } else {
                result = _mesh->sendMessage(*recipient, msg_timestamp, attempt, text, expected_ack, est_timeout);
            }
            if (result == MSG_SEND_FAILED) {
                writeErrFrame(ERR_CODE_TABLE_FULL);
            } else {
                _out[0] = RESP_CODE_SENT;
                _out[1] = (result == MSG_SEND_SENT_FLOOD) ? 1 : 0;
                memcpy(&_out[2], &expected_ack, 4);
                memcpy(&_out[6], &est_timeout, 4);
                _serial->writeFrame(_out, 10);
            }
        } else {
            writeErrFrame(recipient == NULL ? ERR_CODE_NOT_FOUND : ERR_CODE_UNSUPPORTED_CMD);
        }
        return;
    }

    // ---- CMD_SEND_CHANNEL_TXT_MSG (3) ----
    if (cmd == CMD_SEND_CHANNEL_TXT_MSG && len >= 8) {
        int i = 1;
        uint8_t txt_type = _cmd[i++];
        uint8_t channel_idx = _cmd[i++];
        uint32_t msg_timestamp;
        memcpy(&msg_timestamp, &_cmd[i], 4); i += 4;
        const char* text = (char*)&_cmd[i];
        _cmd[len] = 0;

        if (txt_type != TXT_TYPE_PLAIN) {
            writeErrFrame(ERR_CODE_UNSUPPORTED_CMD);
        } else {
            ChannelDetails channel;
            bool ok = _mesh->getChannel(channel_idx, channel);
            if (ok) {
                meck_tables_begin_outgoing();
                bool sent = _mesh->sendGroupMessage(msg_timestamp, channel.channel,
                                                     _mesh->getNodeName(), text, len - i);
                meck_tables_end_outgoing();
                if (sent) {
                    // Record so isOurOwnEcho matches flood echoes from repeaters
                    _mesh->recordOutgoingSend(msg_timestamp, channel_idx);
                    // Inject local echo into ring with matching timestamp so
                    // the echo handler can update heard_count on this entry
                    char echo_buf[200];
                    snprintf(echo_buf, sizeof(echo_buf), "%s: %s",
                             _mesh->getNodeName(), text);
                    _mesh->injectChannelMessage(channel_idx, echo_buf, msg_timestamp);
                    writeOKFrame();
                } else {
                    writeErrFrame(ERR_CODE_NOT_FOUND);
                }
            } else {
                writeErrFrame(ERR_CODE_NOT_FOUND);
            }
        }
        return;
    }

    // ---- CMD_SEND_SELF_ADVERT (7) ----
    if (cmd == CMD_SEND_SELF_ADVERT) {
        mesh::Packet* pkt = _mesh->createSelfAdvert(prefs->node_name);
        if (pkt) {
            _mesh->sendZeroHop(pkt);
            writeOKFrame();
        } else {
            writeErrFrame(ERR_CODE_TABLE_FULL);
        }
        return;
    }

    // ---- CMD_SET_ADVERT_NAME (8) ----
    if (cmd == CMD_SET_ADVERT_NAME && len >= 2) {
        int nlen = len - 1;
        if (nlen > (int)sizeof(prefs->node_name) - 1) nlen = sizeof(prefs->node_name) - 1;
        memcpy(prefs->node_name, &_cmd[1], nlen);
        prefs->node_name[nlen] = 0;
        _mesh->savePrefs();
        writeOKFrame();
        return;
    }

    // ---- CMD_RESET_PATH (13) ----
    if (cmd == CMD_RESET_PATH && len >= 1 + PUB_KEY_SIZE) {
        ContactInfo* r = _mesh->lookupContactByPubKey(&_cmd[1], PUB_KEY_SIZE);
        if (r) {
            r->out_path_len = OUT_PATH_UNKNOWN;
            _mesh->scheduleLazyContactSave();
            writeOKFrame();
        } else {
            writeErrFrame(ERR_CODE_NOT_FOUND);
        }
        return;
    }

    // ---- CMD_SET_ADVERT_LATLON (14) ----
    if (cmd == CMD_SET_ADVERT_LATLON && len >= 9) {
        int32_t lat, lon;
        memcpy(&lat, &_cmd[1], 4);
        memcpy(&lon, &_cmd[5], 4);
        if (lat == 0 && lon == 0) {
            // Clear position
            prefs->position_lat_e7 = 0;
            prefs->position_lon_e7 = 0;
            prefs->position_mode = 0;  // off
        } else if (lat <= 90000000 && lat >= -90000000 &&
                   lon <= 180000000 && lon >= -180000000) {
            // Upstream sends microdegrees (1e6); Meck stores 1e7
            prefs->position_lat_e7 = lat * 10;
            prefs->position_lon_e7 = lon * 10;
            if (prefs->position_mode == 0) prefs->position_mode = 1;  // manual
        } else {
            writeErrFrame(ERR_CODE_ILLEGAL_ARG);
            return;
        }
        _mesh->savePrefs();
        writeOKFrame();
        return;
    }

    // ---- CMD_SET_RADIO_PARAMS (11) ---- change radio frequency/bw/sf/cr
    if (cmd == CMD_SET_RADIO_PARAMS && len >= 11) {
        int i = 1;
        uint32_t freq_khz, bw_hz;
        memcpy(&freq_khz, &_cmd[i], 4); i += 4;
        memcpy(&bw_hz, &_cmd[i], 4); i += 4;
        uint8_t sf = _cmd[i++];
        uint8_t cr = _cmd[i++];
        uint8_t repeat = 0;  // default - false
        if ((int)len > i) {
            repeat = _cmd[i++];   // FIRMWARE_VER_CODE 9+ (client repeat toggle)
        }

        // meck_radio_bw_allowed: on the LR2021 only its own bandwidth steps
        // pass, and bw_khz becomes the exact step; SX1262 unchanged.
        float bw_khz = (float)bw_hz / 1000.0f;
        if (repeat && !isValidClientRepeatFreq(freq_khz)) {
            writeErrFrame(ERR_CODE_ILLEGAL_ARG);
        } else if (freq_khz >= 150000 && freq_khz <= 2500000 &&
            sf >= 5 && sf <= 12 && cr >= 5 && cr <= 8 &&
            bw_hz >= 7000 && bw_hz <= (uint32_t)(meck_radio_max_bw_khz() * 1000.0f) &&
            meck_radio_bw_allowed(bw_khz, &bw_khz)) {
            prefs->freq = (float)freq_khz / 1000.0f;
            prefs->bw   = bw_khz;
            prefs->sf    = sf;
            prefs->cr    = cr;
            prefs->client_repeat = repeat ? 1 : 0;   // takes effect at once
            // LR2021 on 2.4 GHz: lower a stored TX above the band's limit
            // (as Settings does on the device) so the radio never gets a
            // power it refuses. TX is stored unsigned; compare it signed.
            int8_t max_tx = meck_radio_max_tx_dbm(prefs->freq);
            if ((int8_t)prefs->tx_power_dbm > max_tx) {
                prefs->tx_power_dbm = (uint8_t)max_tx;
                printf("Companion: TX lowered to %d dBm for this band\n", (int)max_tx);
            }
            _mesh->savePrefs();
            printf("Companion: SET_RADIO_PARAMS freq=%.3f bw=%.1f sf=%d cr=%d repeat=%d (radio: reboot to apply)\n",
                   prefs->freq, prefs->bw, (int)sf, (int)cr, (int)prefs->client_repeat);
            writeOKFrame();
        } else {
            writeErrFrame(ERR_CODE_ILLEGAL_ARG);
        }
        return;
    }

    // ---- CMD_SET_RADIO_TX_POWER (12) ----
    if (cmd == CMD_SET_RADIO_TX_POWER && len >= 2) {
        int8_t power = (int8_t)_cmd[1];
        // Upper limit follows the radio and band: 22, or 12 on the LR2021
        // at 2.4 GHz (the radio refuses more there).
        if (power >= -9 && power <= meck_radio_max_tx_dbm(prefs->freq)) {
            prefs->tx_power_dbm = power;
            _mesh->savePrefs();
            printf("Companion: SET_RADIO_TX_POWER %d dBm (reboot to apply)\n", (int)power);
            writeOKFrame();
        } else {
            writeErrFrame(ERR_CODE_ILLEGAL_ARG);
        }
        return;
    }

    // ---- CMD_ADD_UPDATE_CONTACT (9) ----
    if (cmd == CMD_ADD_UPDATE_CONTACT && len >= 1 + PUB_KEY_SIZE + 3) {
        uint8_t* pub_key = &_cmd[1];
        ContactInfo* existing = _mesh->lookupContactByPubKey(pub_key, PUB_KEY_SIZE);
        uint32_t last_mod = _mesh->getRTCClock()->getCurrentTime();  // fallback value if not present in the frame
        if (existing) {
            // Update in place, including the path the app set
            updateContactFromFrame(*existing, last_mod, _cmd, (int)len);
            existing->lastmod = last_mod;
            _mesh->scheduleLazyContactSave();
            writeOKFrame();
        } else {
            // Add new contact
            ContactInfo ci;
            memset(&ci, 0, sizeof(ci));
            updateContactFromFrame(ci, last_mod, _cmd, (int)len);
            ci.lastmod = last_mod;
            ci.sync_since = 0;
            if (_mesh->addContact(ci)) {
                _mesh->scheduleLazyContactSave();
                writeOKFrame();
            } else {
                writeErrFrame(ERR_CODE_TABLE_FULL);
            }
        }
        return;
    }

    // ---- CMD_EXPORT_PRIVATE_KEY (23) ----
    if (cmd == CMD_EXPORT_PRIVATE_KEY) {
        mesh::LocalIdentity id = _mesh->getIdentity();
        _out[0] = RESP_CODE_PRIVATE_KEY;
        id.writeTo(&_out[1], 64);
        _serial->writeFrame(_out, 65);
        return;
    }

    // ---- CMD_IMPORT_PRIVATE_KEY (24) ----
    if (cmd == CMD_IMPORT_PRIVATE_KEY && len >= 65) {
        if (!mesh::LocalIdentity::validatePrivateKey(&_cmd[1])) {
            writeErrFrame(ERR_CODE_ILLEGAL_ARG);
        } else {
            mesh::LocalIdentity identity;
            identity.readFrom(&_cmd[1], 64);
            if (_mesh->getDataStore()->saveIdentity(identity)) {
                writeOKFrame();
                printf("Companion: private key imported (reboot to apply)\n");
            } else {
                writeErrFrame(ERR_CODE_NOT_FOUND);
            }
        }
        return;
    }

    // ---- CMD_GET_CONTACT_BY_KEY (30) ----
    if (cmd == CMD_GET_CONTACT_BY_KEY && len >= 1 + PUB_KEY_SIZE) {
        ContactInfo* c = _mesh->lookupContactByPubKey(&_cmd[1], PUB_KEY_SIZE);
        if (c) {
            writeContactRespFrame(RESP_CODE_CONTACT, *c);
        } else {
            writeErrFrame(ERR_CODE_NOT_FOUND);
        }
        return;
    }

    // ---- CMD_REMOVE_CONTACT (15) ----
    if (cmd == CMD_REMOVE_CONTACT && len >= 1 + PUB_KEY_SIZE) {
        ContactInfo* r = _mesh->lookupContactByPubKey(&_cmd[1], PUB_KEY_SIZE);
        if (r && _mesh->removeContact(*r)) {
            _mesh->scheduleLazyContactSave();
            writeOKFrame();
        } else {
            writeErrFrame(ERR_CODE_NOT_FOUND);
        }
        return;
    }

    // ---- CMD_SHARE_CONTACT (16) ---- re-send a contact's last advert
    // zero-hop, from the copy kept on SD (P4DataStore::putAdvertBlob).
    // As upstream: NOT_FOUND if not a contact, TABLE_FULL if it can't be
    // sent (no stored advert, e.g. no SD card, or no free packet).
    if (cmd == CMD_SHARE_CONTACT && len >= 1 + PUB_KEY_SIZE) {
        ContactInfo* r = _mesh->lookupContactByPubKey(&_cmd[1], PUB_KEY_SIZE);
        if (r) {
            if (_mesh->shareContactZeroHop(*r)) {
                writeOKFrame();
            } else {
                writeErrFrame(ERR_CODE_TABLE_FULL);   // unable to send
            }
        } else {
            writeErrFrame(ERR_CODE_NOT_FOUND);
        }
        return;
    }

    // ---- CMD_GET_CUSTOM_VARS (40) ----
    if (cmd == CMD_GET_CUSTOM_VARS) {
        _out[0] = RESP_CODE_CUSTOM_VARS;
        char *dp = (char *)&_out[1];
        char *end = (char *)&_out[MAX_FRAME_SIZE - 1];

        // gps
        dp += snprintf(dp, end - dp, "gps:%d", meck_gps_is_enabled() ? 1 : 0);

        // gps_interval (we don't have a configurable interval; report 0)
        dp += snprintf(dp, end - dp, ",gps_interval:0");

        // latitude / longitude (from prefs, converted from e7 to degrees)
        if (prefs->position_lat_e7 != 0 || prefs->position_lon_e7 != 0) {
            dp += snprintf(dp, end - dp, ",latitude:%.7f",
                           (double)prefs->position_lat_e7 / 1e7);
            dp += snprintf(dp, end - dp, ",longitude:%.7f",
                           (double)prefs->position_lon_e7 / 1e7);
        } else {
            dp += snprintf(dp, end - dp, ",latitude:0,longitude:0");
        }

        _serial->writeFrame(_out, dp - (char *)_out);
        return;
    }

    // ---- CMD_SET_CUSTOM_VAR (41) ----
    if (cmd == CMD_SET_CUSTOM_VAR && len >= 4) {
        _cmd[len] = 0;  // null-terminate
        char *sp = (char *)&_cmd[1];
        char *np = strchr(sp, ':');
        if (!np) {
            writeErrFrame(ERR_CODE_ILLEGAL_ARG);
            return;
        }
        *np++ = 0;  // split name:value

        bool success = false;
        if (strcmp(sp, "gps") == 0) {
            bool on = (np[0] == '1');
            meck_gps_set_enabled(on);
            prefs->gps_enabled = on ? 1 : 2;
            _mesh->savePrefs();
            success = true;
        } else if (strcmp(sp, "latitude") == 0) {
            double lat = atof(np);
            prefs->position_lat_e7 = (int32_t)(lat * 1e7);
            if (prefs->position_mode == 0 && prefs->position_lat_e7 != 0)
                prefs->position_mode = 1;
            _mesh->savePrefs();
            success = true;
        } else if (strcmp(sp, "longitude") == 0) {
            double lon = atof(np);
            prefs->position_lon_e7 = (int32_t)(lon * 1e7);
            if (prefs->position_mode == 0 && prefs->position_lon_e7 != 0)
                prefs->position_mode = 1;
            _mesh->savePrefs();
            success = true;
        } else if (strcmp(sp, "gps_interval") == 0) {
            // Acknowledged but not used (P4 GPS is on/off only)
            success = true;
        }

        if (success)
            writeOKFrame();
        else
            writeErrFrame(ERR_CODE_ILLEGAL_ARG);
        return;
    }

    // ---- CMD_GET_ADVERT_PATH (42) ----
    if (cmd == CMD_GET_ADVERT_PATH && len >= 2 + PUB_KEY_SIZE) {
        const uint8_t* pub_key = &_cmd[2];  // byte 1 = reserved
        const P4AdvertPath* found = _mesh->lookupAdvertPath(pub_key);
        if (found) {
            int i = 0;
            _out[i++] = RESP_CODE_ADVERT_PATH;
            memcpy(&_out[i], &found->recv_timestamp, 4); i += 4;
            _out[i++] = found->path_len;
            i += mesh::Packet::writePath(&_out[i], found->path, found->path_len);
            _serial->writeFrame(_out, i);
        } else {
            writeErrFrame(ERR_CODE_NOT_FOUND);
        }
        return;
    }

    // ---- CMD_GET_AUTOADD_CONFIG (59) ----
    if (cmd == CMD_GET_AUTOADD_CONFIG) {
        _out[0] = RESP_CODE_AUTOADD_CONFIG;
        _out[1] = prefs->autoadd_config;
        _serial->writeFrame(_out, 2);
        return;
    }

    // ---- CMD_GET_DEFAULT_FLOOD_SCOPE (64) ----
    if (cmd == CMD_GET_DEFAULT_FLOOD_SCOPE) {
        _out[0] = RESP_CODE_DEFAULT_FLOOD_SCOPE;
        if (prefs->default_scope_name[0] != '\0') {
            memcpy(&_out[1], prefs->default_scope_name, 31);
            memcpy(&_out[1 + 31], prefs->default_scope_key, 16);
            _serial->writeFrame(_out, 1 + 31 + 16);
        } else {
            _serial->writeFrame(_out, 1);  // no scope = just resp code
        }
        return;
    }

    // ---- CMD_SET_DEFAULT_FLOOD_SCOPE (63) ----
    if (cmd == CMD_SET_DEFAULT_FLOOD_SCOPE && len >= 1) {
        if (len >= 1 + 31 + 16) {
            // Name + key provided
            int n = strlen((char*)&_cmd[1]);
            if (n > 0 && n < 31) {
                strcpy(prefs->default_scope_name, (char*)&_cmd[1]);
                memcpy(prefs->default_scope_key, &_cmd[1 + 31], 16);
                _mesh->savePrefs();
                writeOKFrame();
            } else {
                writeErrFrame(ERR_CODE_ILLEGAL_ARG);
            }
        } else {
            // Clear scope
            memset(prefs->default_scope_name, 0, sizeof(prefs->default_scope_name));
            memset(prefs->default_scope_key, 0, sizeof(prefs->default_scope_key));
            _mesh->savePrefs();
            writeOKFrame();
        }
        return;
    }

    // ---- CMD_SET_PATH_HASH_MODE (61) ----
    if (cmd == CMD_SET_PATH_HASH_MODE && _cmd[1] == 0 && len >= 3) {
        if (_cmd[2] >= 3) {
            writeErrFrame(ERR_CODE_ILLEGAL_ARG);
        } else {
            prefs->path_hash_mode = _cmd[2];
            _mesh->savePrefs();
            writeOKFrame();
        }
        return;
    }

    // ---- CMD_SET_FLOOD_SCOPE_KEY (54) ---- v8+ flood scope
    // Acknowledge so the app doesn't retry. Full scope wiring is a
    // future task (needs send_scope member in MeckMesh).
    if (cmd == CMD_SET_FLOOD_SCOPE_KEY && len >= 2) {
        writeOKFrame();
        return;
    }

    // ---- CMD_SET_CHANNEL (32) ---- import/update a channel
    if (cmd == CMD_SET_CHANNEL && len >= 2 + 32 + 16) {
        uint8_t channel_idx = _cmd[1];
        // Capture the slot's current occupant before the overwrite: message
        // history is keyed by channel identity, so if the app puts a
        // different channel in this slot, the ring must be reloaded for the
        // new occupant instead of inheriting the previous occupant's
        // messages.
        ChannelDetails prev;
        bool had_prev = _mesh->getChannel(channel_idx, prev) && prev.name[0] != '\0';
        ChannelDetails channel;
        strzcpy(channel.name, (char*)&_cmd[2], 32);
        memset(channel.channel.secret, 0, sizeof(channel.channel.secret));
        memcpy(channel.channel.secret, &_cmd[2 + 32], 16);  // 128-bit key
        if (_mesh->setChannel(channel_idx, channel)) {
            _mesh->saveChannels();
            if (!had_prev ||
                memcmp(prev.channel.secret, channel.channel.secret,
                       sizeof(channel.channel.secret)) != 0) {
                _mesh->reloadChannelSlot(channel_idx);
            }
            writeOKFrame();
        } else {
            writeErrFrame(ERR_CODE_NOT_FOUND);
        }
        return;
    }

    // ---- CMD_SET_OTHER_PARAMS (38) ----
    if (cmd == CMD_SET_OTHER_PARAMS && len >= 2) {
        prefs->manual_add_contacts = _cmd[1];
        if (_cmd[1] == 0) {
            // Auto mode: shouldAutoAddContactType() consults the per-type
            // bits, so they must all be on (0x1E = chat|repeater|room|sensor)
            // or nothing gets added. Mirrors the on-screen Auto All fix in
            // MeckUI.cpp. The overwrite bit (0x01) is left as it was.
            prefs->autoadd_config |= 0x1E;
        } else {
            // Manual mode: clear the per-type bits so this is the same state
            // as the on-screen Manual Only. shouldAutoAddContactType() treats
            // manual-bit-plus-type-bits as the UI's Custom mode, so bits left
            // over from a previous auto selection would keep auto-adding.
            prefs->autoadd_config &= ~0x1E;
        }
        if (len >= 4) {
            // byte[3] = advert_loc_policy: 0=don't share, 1+=share
            uint8_t loc_policy = _cmd[3];
            if (loc_policy == 0) {
                prefs->position_mode = 0;  // off
            } else if (prefs->position_mode == 0) {
                prefs->position_mode = 1;  // manual (enable sharing)
            }
        }
        if (len >= 5) {
            prefs->multi_acks = _cmd[4];
        }
        _mesh->savePrefs();
        writeOKFrame();
        return;
    }

    // ---- CMD_SEND_LOGIN (26) ---- repeater admin login
    // App sends pub_key followed by password. We route to the mesh's
    // sendLogin via a Meck wrapper that mirrors the on-device admin
    // path (force-flood for the login itself, restore out_path after,
    // reset sync_since for room contacts) but uses companion-side
    // pending state so it doesn't disturb the device admin session.
    if (cmd == CMD_SEND_LOGIN && len >= 1 + PUB_KEY_SIZE) {
        uint8_t* pub_key = &_cmd[1];
        char* password = (char*)&_cmd[1 + PUB_KEY_SIZE];
        _cmd[len] = 0;  // ensure null-terminator on password
        uint32_t est_timeout = 0;
        uint8_t is_flood = 0;
        if (_mesh->companionLoginToRepeater(pub_key, password, est_timeout, is_flood)) {
            int i = 0;
            _out[i++] = RESP_CODE_SENT;
            _out[i++] = is_flood;
            memcpy(&_out[i], pub_key, 4); i += 4;  // pending_login marker for the app
            memcpy(&_out[i], &est_timeout, 4); i += 4;
            _serial->writeFrame(_out, i);
        } else {
            writeErrFrame(ERR_CODE_NOT_FOUND);
        }
        return;
    }

    // ---- CMD_SEND_STATUS_REQ (27) ---- repeater status query
    // App sends pub_key. Routes to sendRequest(REQ_TYPE_GET_STATUS) via
    // a Meck wrapper that records the response tag in companion-side
    // pending state.
    if (cmd == CMD_SEND_STATUS_REQ && len >= 1 + PUB_KEY_SIZE) {
        uint8_t* pub_key = &_cmd[1];
        uint32_t tag = 0, est_timeout = 0;
        uint8_t is_flood = 0;
        if (_mesh->companionSendStatusRequest(pub_key, tag, est_timeout, is_flood)) {
            int i = 0;
            _out[i++] = RESP_CODE_SENT;
            _out[i++] = is_flood;
            memcpy(&_out[i], &tag, 4); i += 4;
            memcpy(&_out[i], &est_timeout, 4); i += 4;
            _serial->writeFrame(_out, i);
        } else {
            writeErrFrame(ERR_CODE_NOT_FOUND);
        }
        return;
    }

    // ---- CMD_HAS_CONNECTION (28) ---- query whether a keep-alive session
    // is currently active for the given contact pub_key.
    if (cmd == CMD_HAS_CONNECTION && len >= 1 + PUB_KEY_SIZE) {
        if (_mesh->companionHasConnection(&_cmd[1])) {
            writeOKFrame();
        } else {
            writeErrFrame(ERR_CODE_NOT_FOUND);
        }
        return;
    }

    // ---- CMD_LOGOUT (29) ---- end the keep-alive session for a contact
    if (cmd == CMD_LOGOUT && len >= 1 + PUB_KEY_SIZE) {
        _mesh->companionLogout(&_cmd[1]);
        writeOKFrame();
        return;
    }

    // ---- CMD_SEND_BINARY_REQ (50) ---- generic binary request to a
    // repeater (GET_NEIGHBOURS, ACL queries, owner info, etc). App
    // packs the request type and parameters into req_data; we forward
    // verbatim to sendRequest and record the response tag in companion
    // pending state. Responses match by tag (4 bytes), not pub_key.
    if (cmd == CMD_SEND_BINARY_REQ && len >= 2 + PUB_KEY_SIZE) {
        uint8_t* pub_key = &_cmd[1];
        uint8_t* req_data = &_cmd[1 + PUB_KEY_SIZE];
        int req_data_len = (int)len - (1 + PUB_KEY_SIZE);
        uint32_t tag = 0, est_timeout = 0;
        uint8_t is_flood = 0;
        if (_mesh->companionSendBinaryReq(pub_key, req_data, req_data_len,
                                          tag, est_timeout, is_flood)) {
            int i = 0;
            _out[i++] = RESP_CODE_SENT;
            _out[i++] = is_flood;
            memcpy(&_out[i], &tag, 4); i += 4;
            memcpy(&_out[i], &est_timeout, 4); i += 4;
            _serial->writeFrame(_out, i);
        } else {
            writeErrFrame(ERR_CODE_NOT_FOUND);
        }
        return;
    }

    // ---- CMD_GET_ALLOWED_REPEAT_FREQ (60) ---- client repeat frequency list
    if (cmd == CMD_GET_ALLOWED_REPEAT_FREQ) {
        int i = 0;
        _out[i++] = RESP_ALLOWED_REPEAT_FREQ;
        for (size_t k = 0; k < sizeof(repeat_freq_ranges) / sizeof(repeat_freq_ranges[0]) &&
                           i + 8 <= MAX_FRAME_SIZE; k++) {
            memcpy(&_out[i], &repeat_freq_ranges[k].lower_freq, 4); i += 4;
            memcpy(&_out[i], &repeat_freq_ranges[k].upper_freq, 4); i += 4;
        }
        _serial->writeFrame(_out, i);
        return;
    }

    // ---- CMD_SEND_CONTROL_DATA (55) ---- control packet from the app,
    // sent zero-hop (upstream: only subtypes with the top bit set).
    if (cmd == CMD_SEND_CONTROL_DATA && len >= 2 && (_cmd[1] & 0x80) != 0) {
        if (_mesh->companionSendControlData(&_cmd[1], (int)len - 1)) {
            writeOKFrame();
        } else {
            writeErrFrame(ERR_CODE_TABLE_FULL);
        }
        return;
    }

    // ---- CMD_SEND_ANON_REQ (57) ---- request to any node, contact or not
    // (FIRMWARE_VER_CODE 13+). Reply comes back as PUSH_CODE_BINARY_RESPONSE.
    if (cmd == CMD_SEND_ANON_REQ && len > 1 + PUB_KEY_SIZE) {
        uint8_t* pub_key = &_cmd[1];
        uint8_t* data = &_cmd[1 + PUB_KEY_SIZE];
        uint32_t tag = 0, est_timeout = 0;
        int result = _mesh->companionSendAnonReq(pub_key, data, (int)len - (1 + PUB_KEY_SIZE),
                                                 tag, est_timeout);
        if (result == MSG_SEND_FAILED) {
            writeErrFrame(ERR_CODE_TABLE_FULL);
        } else {
            int i = 0;
            _out[i++] = RESP_CODE_SENT;
            _out[i++] = (result == MSG_SEND_SENT_FLOOD) ? 1 : 0;
            memcpy(&_out[i], &tag, 4); i += 4;
            memcpy(&_out[i], &est_timeout, 4); i += 4;
            _serial->writeFrame(_out, i);
        }
        return;
    }

    // ---- CMD_SET_TUNING_PARAMS (21) / CMD_GET_TUNING_PARAMS (43) ----
    // Receive delay base and airtime factor, each x1000, as upstream. Both
    // are live: MeckMesh's calcRxDelay and getAirtimeBudgetFactor read them
    // from prefs. Same settings as the CLI's rxdelay and af.
    if (cmd == CMD_SET_TUNING_PARAMS) {
        int i = 1;
        uint32_t rx, af;
        memcpy(&rx, &_cmd[i], 4); i += 4;
        memcpy(&af, &_cmd[i], 4); i += 4;
        prefs->rx_delay_base  = ((float)rx) / 1000.0f;
        prefs->airtime_factor = ((float)af) / 1000.0f;
        _mesh->savePrefs();
        writeOKFrame();
        return;
    }
    if (cmd == CMD_GET_TUNING_PARAMS) {
        uint32_t rx = prefs->rx_delay_base * 1000, af = prefs->airtime_factor * 1000;
        int i = 0;
        _out[i++] = RESP_CODE_TUNING_PARAMS;
        memcpy(&_out[i], &rx, 4); i += 4;
        memcpy(&_out[i], &af, 4); i += 4;
        _serial->writeFrame(_out, i);
        return;
    }

    // ---- CMD_RUN_CLI_COMMAND (66) ---- the app's Command Line (v14+).
    // Same handler as Settings > Experimental Features > Companion CLI.
    if (cmd == CMD_RUN_CLI_COMMAND && len >= 3) {
        char* text = (char*)&_cmd[1];
        _cmd[len] = 0;  // ensure NUL
        char reply[MECK_CLI_REPLY_SIZE];
        reply[0] = 0;
        if (!meck_cli_handle(text, reply)) {
            strcat(reply, "Unknown command");   // reply may hold the prefix reflected from 'text'
        }
        int rlen = (int)strlen(reply);
        if (rlen > MAX_FRAME_SIZE - 1) rlen = MAX_FRAME_SIZE - 1;
        _out[0] = RESP_CODE_CLI_REPLY;
        memcpy(&_out[1], reply, rlen);
        _serial->writeFrame(_out, 1 + rlen);
        return;
    }

    // =======================================================================
    // App commands ported from Meck (and CMD_SEND_RAW_PACKET from Meck Watch)
    // =======================================================================

    // ---- CMD_SEND_TRACE_PATH (36) ---- [tag:4][auth:4][flags][hop hashes].
    // The result comes back as PUSH_CODE_TRACE_DATA.
    if (cmd == CMD_SEND_TRACE_PATH && len > 10 && len - 10 < MAX_PACKET_PAYLOAD - 5) {
        uint8_t path_len = (uint8_t)(len - 10);
        uint8_t flags = _cmd[9];
        uint8_t path_sz = flags & 0x03;  // hash size as a shift
        if ((path_len >> path_sz) > MAX_PATH_SIZE || (path_len % (1 << path_sz)) != 0) {  // must be a multiple of the hash size
            writeErrFrame(ERR_CODE_ILLEGAL_ARG);
        } else {
            uint32_t tag, auth, est_timeout = 0;
            memcpy(&tag, &_cmd[1], 4);
            memcpy(&auth, &_cmd[5], 4);
            if (_mesh->companionSendTrace(tag, auth, flags, &_cmd[10], path_len, est_timeout)) {
                _out[0] = RESP_CODE_SENT;
                _out[1] = 0;
                memcpy(&_out[2], &tag, 4);
                memcpy(&_out[6], &est_timeout, 4);
                _serial->writeFrame(_out, 10);
            } else {
                writeErrFrame(ERR_CODE_TABLE_FULL);
            }
        }
        return;
    }

    // ---- CMD_SEND_PATH_DISCOVERY_REQ (52) ---- [0][pub_key]. The result
    // comes back as PUSH_CODE_PATH_DISCOVERY_RESPONSE.
    if (cmd == CMD_SEND_PATH_DISCOVERY_REQ && _cmd[1] == 0 && len >= 2 + PUB_KEY_SIZE) {
        ContactInfo* recipient = _mesh->lookupContactByPubKey(&_cmd[2], PUB_KEY_SIZE);
        if (recipient) {
            uint32_t tag = 0, est_timeout = 0;
            int result = _mesh->companionSendPathDiscovery(*recipient, tag, est_timeout);
            if (result == MSG_SEND_FAILED) {
                writeErrFrame(ERR_CODE_TABLE_FULL);
            } else {
                _out[0] = RESP_CODE_SENT;
                _out[1] = (result == MSG_SEND_SENT_FLOOD) ? 1 : 0;
                memcpy(&_out[2], &tag, 4);
                memcpy(&_out[6], &est_timeout, 4);
                _serial->writeFrame(_out, 10);
            }
        } else {
            writeErrFrame(ERR_CODE_NOT_FOUND);  // contact not found
        }
        return;
    }

    // ---- CMD_SEND_TELEMETRY_REQ (39) ---- [3 reserved][pub_key]: ask a
    // contact. The reply comes back as PUSH_CODE_TELEMETRY_RESPONSE.
    if (cmd == CMD_SEND_TELEMETRY_REQ && len >= 4 + PUB_KEY_SIZE) {
        ContactInfo* recipient = _mesh->lookupContactByPubKey(&_cmd[4], PUB_KEY_SIZE);
        if (recipient) {
            uint32_t tag = 0, est_timeout = 0;
            int result = _mesh->companionSendTelemetryReq(*recipient, tag, est_timeout);
            if (result == MSG_SEND_FAILED) {
                writeErrFrame(ERR_CODE_TABLE_FULL);
            } else {
                _out[0] = RESP_CODE_SENT;
                _out[1] = (result == MSG_SEND_SENT_FLOOD) ? 1 : 0;
                memcpy(&_out[2], &tag, 4);
                memcpy(&_out[6], &est_timeout, 4);
                _serial->writeFrame(_out, 10);
            }
        } else {
            writeErrFrame(ERR_CODE_NOT_FOUND);  // contact not found
        }
        return;
    }
    // ...with no pub_key: the P4's own telemetry. The battery voltage, as a
    // CayenneLPP voltage record on the 'self' channel.
    if (cmd == CMD_SEND_TELEMETRY_REQ && len == 4) {
        uint16_t centivolts = meck_battery_get_mv() / 10;   // LPP voltage: 0.01 V units, big-endian
        int i = 0;
        _out[i++] = PUSH_CODE_TELEMETRY_RESPONSE;
        _out[i++] = 0;  // reserved
        memcpy(&_out[i], _mesh->getIdentity().pub_key, 6); i += 6;  // pub_key_prefix
        _out[i++] = 1;      // TELEM_CHANNEL_SELF
        _out[i++] = 116;    // LPP_VOLTAGE
        _out[i++] = (uint8_t)(centivolts >> 8);
        _out[i++] = (uint8_t)(centivolts & 0xFF);
        _serial->writeFrame(_out, i);
        return;
    }

    // ---- CMD_SET_AUTOADD_CONFIG (58) ---- which contact types are auto-added
    if (cmd == CMD_SET_AUTOADD_CONFIG && len >= 2) {
        prefs->autoadd_config = _cmd[1];
        _mesh->savePrefs();
        writeOKFrame();
        return;
    }

    // ---- CMD_EXPORT_CONTACT (17) ---- a contact, or the P4 itself (no
    // pub_key), as a shareable advert packet.
    if (cmd == CMD_EXPORT_CONTACT) {
        if (len < 1 + PUB_KEY_SIZE) {
            // export SELF, with position when position sharing is on
            mesh::Packet* pkt;
            if (prefs->position_mode != 0 &&
                (prefs->position_lat_e7 != 0 || prefs->position_lon_e7 != 0)) {
                pkt = _mesh->createSelfAdvert(prefs->node_name,
                                              (double)prefs->position_lat_e7 / 1e7,
                                              (double)prefs->position_lon_e7 / 1e7);
            } else {
                pkt = _mesh->createSelfAdvert(prefs->node_name);
            }
            if (pkt) {
                pkt->header |= ROUTE_TYPE_FLOOD;  // would normally be sent in this mode
                _out[0] = RESP_CODE_EXPORT_CONTACT;
                uint8_t out_len = pkt->writeTo(&_out[1]);
                _mesh->releasePacket(pkt);  // undo the obtainNewPacket()
                _serial->writeFrame(_out, out_len + 1);
            } else {
                writeErrFrame(ERR_CODE_TABLE_FULL);
            }
        } else {
            ContactInfo* recipient = _mesh->lookupContactByPubKey(&_cmd[1], PUB_KEY_SIZE);
            uint8_t out_len = recipient ? _mesh->exportContact(*recipient, &_out[1]) : 0;
            if (out_len > 0) {
                _out[0] = RESP_CODE_EXPORT_CONTACT;
                _serial->writeFrame(_out, out_len + 1);
            } else {
                writeErrFrame(ERR_CODE_NOT_FOUND);
            }
        }
        return;
    }

    // ---- CMD_IMPORT_CONTACT (18) ---- add a contact from a shared advert packet
    if (cmd == CMD_IMPORT_CONTACT && len > 2 + 32 + 64) {
        if (_mesh->importContact(&_cmd[1], (uint8_t)(len - 1))) {
            _mesh->scheduleLazyContactSave();
            writeOKFrame();
        } else {
            writeErrFrame(ERR_CODE_ILLEGAL_ARG);
        }
        return;
    }

    // ---- CMD_SEND_RAW_DATA (25) ---- [path_len][path][payload]: a custom
    // packet sent direct along the given path.
    if (cmd == CMD_SEND_RAW_DATA && len >= 6) {
        int i = 1;
        uint8_t path_len = _cmd[i++];
        int path_bytes = (path_len & 63) * ((path_len >> 6) + 1);
        if (path_len != OUT_PATH_UNKNOWN && mesh::Packet::isValidPathLen(path_len)
            && i + path_bytes + 4 <= (int)len) {  // minimum 4 byte payload
            uint8_t* path = &_cmd[i];
            i += path_bytes;
            mesh::Packet* pkt = _mesh->createRawData(&_cmd[i], len - i);
            if (pkt) {
                _mesh->sendDirect(pkt, path, path_len);
                writeOKFrame();
            } else {
                writeErrFrame(ERR_CODE_TABLE_FULL);
            }
        } else {
            writeErrFrame(ERR_CODE_UNSUPPORTED_CMD);  // flood, not supported (yet)
        }
        return;
    }

    // ---- CMD_SEND_CHANNEL_DATA (62) ---- [channel][path_len][path][type:2][payload]:
    // a data packet (not text) on a channel; path_len 0xFF = flood.
    if (cmd == CMD_SEND_CHANNEL_DATA) {
        if (len < 4) {
            writeErrFrame(ERR_CODE_ILLEGAL_ARG);
            return;
        }
        int i = 1;
        uint8_t channel_idx = _cmd[i++];
        uint8_t path_len = _cmd[i++];

        // validate path len, allowing 0xFF for flood
        if (!mesh::Packet::isValidPathLen(path_len) && path_len != OUT_PATH_UNKNOWN) {
            writeErrFrame(ERR_CODE_ILLEGAL_ARG);
            return;
        }

        // parse provided path if not flood
        uint8_t path[MAX_PATH_SIZE];
        if (path_len != OUT_PATH_UNKNOWN) {
            i += mesh::Packet::writePath(path, &_cmd[i], path_len);
        }

        uint16_t data_type = ((uint16_t)_cmd[i]) | (((uint16_t)_cmd[i + 1]) << 8);
        i += 2;
        const uint8_t* payload = &_cmd[i];
        int payload_len = (len > (size_t)i) ? (int)(len - i) : 0;

        ChannelDetails channel;
        if (!_mesh->getChannel(channel_idx, channel)) {
            writeErrFrame(ERR_CODE_NOT_FOUND);  // bad channel_idx
        } else if (data_type == DATA_TYPE_RESERVED) {
            writeErrFrame(ERR_CODE_ILLEGAL_ARG);
        } else if (payload_len > MAX_GROUP_DATA_LENGTH) {
            writeErrFrame(ERR_CODE_ILLEGAL_ARG);
        } else if (_mesh->sendGroupData(channel.channel, path, path_len, data_type, payload, payload_len)) {
            writeOKFrame();
        } else {
            writeErrFrame(ERR_CODE_TABLE_FULL);
        }
        return;
    }

    // ---- CMD_SIGN_START (33) / CMD_SIGN_DATA (34) / CMD_SIGN_FINISH (35) ----
    // Sign data supplied by the app with the P4's own key.
    if (cmd == CMD_SIGN_START) {
        _out[0] = RESP_CODE_SIGN_START;
        _out[1] = 0;  // reserved
        uint32_t max_len = MAX_SIGN_DATA_LEN;
        memcpy(&_out[2], &max_len, 4);
        _serial->writeFrame(_out, 6);

        if (_sign_data) {
            free(_sign_data);
        }
        _sign_data = (uint8_t*)malloc(MAX_SIGN_DATA_LEN);
        _sign_data_len = 0;
        return;
    }
    if (cmd == CMD_SIGN_DATA && len > 1) {
        if (_sign_data == NULL || _sign_data_len + (len - 1) > MAX_SIGN_DATA_LEN) {
            writeErrFrame(_sign_data == NULL ? ERR_CODE_BAD_STATE : ERR_CODE_TABLE_FULL);  // error: too long
        } else {
            memcpy(&_sign_data[_sign_data_len], &_cmd[1], len - 1);
            _sign_data_len += (len - 1);
            writeOKFrame();
        }
        return;
    }
    if (cmd == CMD_SIGN_FINISH) {
        if (_sign_data) {
            _mesh->getIdentity().sign(&_out[1], _sign_data, _sign_data_len);

            free(_sign_data);  // don't need the data now
            _sign_data = NULL;

            _out[0] = RESP_CODE_SIGNATURE;
            _serial->writeFrame(_out, 1 + SIGNATURE_SIZE);
        } else {
            writeErrFrame(ERR_CODE_BAD_STATE);
        }
        return;
    }

    // ---- CMD_SEND_RAW_PACKET (65) ---- [priority][complete packet]: transmit
    // a packet supplied by the app, as Meck Watch.
    if (cmd == CMD_SEND_RAW_PACKET && len >= 4) {
        mesh::Packet* pkt = _mesh->obtainNewPacket();
        if (pkt) {
            uint8_t priority = _cmd[1];
            if (_mesh->tryParsePacket(pkt, &_cmd[2], (int)len - 2)) {
                _mesh->sendPacket(pkt, priority, 0);
                writeOKFrame();
            } else {
                _mesh->releasePacket(pkt);
                writeErrFrame(ERR_CODE_ILLEGAL_ARG);
            }
        } else {
            writeErrFrame(ERR_CODE_TABLE_FULL);
        }
        return;
    }

#if MECK_BLE_GARMIN
    // =======================================================================
    // Garmin watch extension, as Meck's MECK_WATCH_EXT
    // =======================================================================

    // ---- CMD_WATCH_HELLO (0x70) ---- the watch declares itself
    if (cmd == CMD_WATCH_HELLO) {
        _watch_client = true;
        int i = 0;
        _out[i++] = RESP_CODE_WATCH_INFO;
        _out[i++] = WATCH_EXT_VER;
        _out[i++] = CANNED_MSG_SLOTS;
        _out[i++] = prefs->path_hash_mode + 1;   // bytes per hop in hop lists
        _out[i++] = Meck::SCOPE_COUNT;
        _serial->writeFrame(_out, i);
        return;
    }

    // ---- CMD_WATCH_GET_CANNED (0x71) ---- one of the P4's canned messages
    if (cmd == CMD_WATCH_GET_CANNED && len >= 2) {
        uint8_t slot = _cmd[1];
        if (slot >= CANNED_MSG_SLOTS) {
            writeErrFrame(ERR_CODE_ILLEGAL_ARG);
        } else {
            int i = 0;
            _out[i++] = RESP_CODE_WATCH_CANNED;
            _out[i++] = slot;
            int tlen = (int)strnlen(prefs->canned_msgs[slot], CANNED_MSG_LEN);
            if (i + tlen > MAX_FRAME_SIZE) tlen = MAX_FRAME_SIZE - i;
            memcpy(&_out[i], prefs->canned_msgs[slot], tlen); i += tlen;
            _serial->writeFrame(_out, i);
        }
        return;
    }

    // ---- CMD_WATCH_GET_SCOPE (0x73) ---- region name for the scope byte of
    // a watch channel message frame (the P4's scope marker).
    if (cmd == CMD_WATCH_GET_SCOPE && len >= 2) {
        uint8_t idx = _cmd[1];
        char name[40];
        int i = 0;
        _out[i++] = RESP_CODE_WATCH_SCOPE;
        _out[i++] = idx;
        if (idx != 0xFF && _mesh->getScopeLabel(idx, name, sizeof(name))) {
            int tlen = (int)strlen(name);
            memcpy(&_out[i], name, tlen); i += tlen;
        }
        _serial->writeFrame(_out, i);
        return;
    }

    // ---- CMD_WATCH_DISCOVER_START (0x74) ---- the P4's own discovery scan
    if (cmd == CMD_WATCH_DISCOVER_START) {
        _mesh->startDiscovery();
        writeOKFrame();
        return;
    }

    // ---- CMD_WATCH_DISCOVER_GET (0x75) ---- the n-th node the scan found
    if (cmd == CMD_WATCH_DISCOVER_GET && len >= 2) {
        uint8_t n = _cmd[1];
        int count = _mesh->getDiscoveredCount();
        int i = 0;
        _out[i++] = RESP_CODE_WATCH_DISCOVER;
        _out[i++] = n;
        _out[i++] = _mesh->isDiscoveryActive() ? 1 : 0;
        _out[i++] = (uint8_t)count;
        if (n < count) {
            const DiscoveredNode& d = _mesh->getDiscovered(n);
            _out[i++] = d.adv_type;
            _out[i++] = (uint8_t)d.snr_x4;
            _out[i++] = d.path_len;
            _out[i++] = d.already_in_contacts ? 1 : 0;
            memcpy(&_out[i], d.pub_key, 2); i += 2;
            int tlen = (int)strnlen(d.name, sizeof(d.name));
            if (i + tlen > MAX_FRAME_SIZE) tlen = MAX_FRAME_SIZE - i;
            memcpy(&_out[i], d.name, tlen); i += tlen;
        }
        _serial->writeFrame(_out, i);
        return;
    }

    // ---- CMD_WATCH_GET_SENT_TRACK (0x72) ---- "Heard by" for a recent
    // channel send: n = 0 is the most recent, n = 1 the one before.
    if (cmd == CMD_WATCH_GET_SENT_TRACK && len >= 2) {
        uint8_t n = _cmd[1];
        int i = 0;
        _out[i++] = RESP_CODE_WATCH_SENT_TRACK;
        _out[i++] = n;
        const P4SentTrack* t = _mesh->getSentTrack(n);
        _out[i++] = t ? 1 : 0;   // 0 = no such slot, or expired
        if (t) {
            memcpy(&_out[i], t->fingerprint, P4_SENT_FINGERPRINT_SIZE); i += P4_SENT_FINGERPRINT_SIZE;
            _out[i++] = t->repeat_count;
            _out[i++] = t->echo_bph;
            _out[i++] = t->echo_count;
            int hlen = t->echo_bph * t->echo_count;
            memcpy(&_out[i], t->echo_hash, hlen); i += hlen;
            memcpy(&_out[i], t->echo_snr, t->echo_count); i += t->echo_count;
        }
        _serial->writeFrame(_out, i);
        return;
    }

    // ---- CMD_WATCH_GET_FAV (0x76) ---- the n-th contact with the favourite
    // bit (flags bit 0, upstream's isFav()), in contact order, with the
    // number of favourites. Walks the contacts with the standard iterator, as
    // CMD_GET_CONTACTS does, without touching that command's own iterator.
    if (cmd == CMD_WATCH_GET_FAV && len >= 2) {
        uint8_t n = _cmd[1];
        int count = 0;
        bool found = false;
        ContactInfo hit;
        ContactInfo c;
        ContactsIterator it = _mesh->startContactsIterator();
        while (it.hasNext(_mesh, c)) {
            if (c.flags & 0x01) {
                if (count == n) { hit = c; found = true; }
                count++;
            }
        }
        if (count > 255) count = 255;
        int i = 0;
        _out[i++] = RESP_CODE_WATCH_FAV;
        _out[i++] = n;
        _out[i++] = (uint8_t)count;
        if (found) {
            _out[i++] = hit.type;
            memcpy(&_out[i], &hit.last_advert_timestamp, 4); i += 4;
            memcpy(&_out[i], hit.id.pub_key, PUB_KEY_SIZE); i += PUB_KEY_SIZE;
            int tlen = (int)strnlen(hit.name, sizeof(hit.name));
            if (i + tlen > MAX_FRAME_SIZE) tlen = MAX_FRAME_SIZE - i;
            memcpy(&_out[i], hit.name, tlen); i += tlen;
        }
        _serial->writeFrame(_out, i);
        return;
    }

    // ---- CMD_WATCH_RESOLVE (0x77) ---- contact name for a public-key
    // prefix (the watch's neighbour and discover lists show names).
    if (cmd == CMD_WATCH_RESOLVE && len >= 3) {
        uint8_t plen = _cmd[1];
        if (plen < 1 || plen > PUB_KEY_SIZE || len < (size_t)(2 + plen)) {
            writeErrFrame(ERR_CODE_ILLEGAL_ARG);
        } else {
            ContactInfo* found = _mesh->lookupContactByPubKey(&_cmd[2], plen);
            int i = 0;
            _out[i++] = RESP_CODE_WATCH_RESOLVE;
            _out[i++] = plen;
            memcpy(&_out[i], &_cmd[2], plen); i += plen;
            if (found != NULL && found->name[0] != '\0') {
                int tlen = (int)strnlen(found->name, sizeof(found->name));
                if (i + tlen > MAX_FRAME_SIZE) tlen = MAX_FRAME_SIZE - i;
                memcpy(&_out[i], found->name, tlen); i += tlen;
            }
            _serial->writeFrame(_out, i);
        }
        return;
    }
#endif

    // ---- Unhandled command ----
    printf("Companion: unhandled cmd 0x%02X len=%d\n", cmd, (int)len);
    writeErrFrame(ERR_CODE_UNSUPPORTED_CMD);
}

// ===========================================================================
// Offline queue -- stores notification frames until the app pops them
// via CMD_SYNC_NEXT_MESSAGE.
// ===========================================================================

void MeckCompanion::addToOfflineQueue(const uint8_t frame[], int len) {
    if (len <= 0 || len > MAX_FRAME_SIZE) return;
    if (!_offline_queue) return;
    if (_offline_queue_len >= OFFLINE_QUEUE_SIZE) {
        // Queue full -- drop the oldest frame (advance the head)
        _offline_head = (_offline_head + 1) % OFFLINE_QUEUE_SIZE;
        _offline_queue_len--;
    }
    int slot = (_offline_head + _offline_queue_len) % OFFLINE_QUEUE_SIZE;
    _offline_queue[slot].len = (uint8_t)len;
    memcpy(_offline_queue[slot].buf, frame, len);
    _offline_queue_len++;
}

int MeckCompanion::getFromOfflineQueue(uint8_t frame[]) {
    if (_offline_queue_len <= 0 || !_offline_queue) return 0;
    int len = _offline_queue[_offline_head].len;
    memcpy(frame, _offline_queue[_offline_head].buf, len);
    _offline_head = (_offline_head + 1) % OFFLINE_QUEUE_SIZE;
    _offline_queue_len--;
    return len;
}

void MeckCompanion::pushMsgWaiting() {
    if (!_serial || !_serial->isConnected()) return;
    uint8_t frame[1] = { PUSH_CODE_MSG_WAITING };
    _serial->writeFrame(frame, 1);
}

// ===========================================================================
// Push notifications -- called from MeckMesh callbacks
// ===========================================================================

void MeckCompanion::pushChannelMessage(uint8_t ch_idx, uint8_t path_len,
                                        uint32_t timestamp, int8_t snr_x4,
                                        uint8_t scope_marker,
                                        const uint8_t* path, uint8_t path_bytes,
                                        const char* text) {
    int i = 0;
#if MECK_BLE_GARMIN
    if (_watch_client) {
        // Watch frame (as Meck's RESP_MECK_CHANNEL_MSG): carries the region
        // and the hop hashes, which the standard frame omits. The path byte
        // is the number of hop bytes that follow (0xFF = not a flood), which
        // is how the watch reads it (hops = bytes / bytes-per-hop).
        // Until the frame is collected (CMD_SYNC_NEXT_MESSAGE) the txt_type
        // slot holds the packet's encoded path_len, so the frame can be
        // rebuilt as a standard one for a non-watch client.
        uint8_t plen = (path_len != 0xFF && path != NULL) ? path_bytes : 0;
        _out[i++] = RESP_CODE_WATCH_CHANNEL_MSG;
        _out[i++] = (uint8_t)snr_x4;
        _out[i++] = (scope_marker == 0) ? 0xFF : scope_marker;   // 0xFF = no region
        _out[i++] = (path_len != 0xFF) ? plen : 0xFF;
        _out[i++] = ch_idx;
        _out[i++] = path_len;   // replaced with TXT_TYPE_PLAIN on collection
        memcpy(&_out[i], &timestamp, 4); i += 4;
        memcpy(&_out[i], path, plen); i += plen;
    } else {
#endif
    if (_app_target_ver >= 3) {
        _out[i++] = RESP_CODE_CHANNEL_MSG_RECV_V3;
        _out[i++] = (uint8_t)snr_x4;
        _out[i++] = 0;  // reserved1
        _out[i++] = 0;  // reserved2
    } else {
        _out[i++] = RESP_CODE_CHANNEL_MSG_RECV;
    }
    _out[i++] = ch_idx;
    _out[i++] = path_len;
    _out[i++] = 0;  // TXT_TYPE_PLAIN
    memcpy(&_out[i], &timestamp, 4); i += 4;
#if MECK_BLE_GARMIN
    }
#endif
    int tlen = text ? (int)strlen(text) : 0;
    if (i + tlen > MAX_FRAME_SIZE) tlen = MAX_FRAME_SIZE - i;
    if (tlen > 0) memcpy(&_out[i], text, tlen);
    i += tlen;
    addToOfflineQueue(_out, i);
    pushMsgWaiting();
}

void MeckCompanion::pushContactMessage(const uint8_t* pub_key_prefix,
                                        uint8_t path_len, uint8_t txt_type,
                                        uint32_t timestamp, int8_t snr_x4,
                                        const uint8_t* extra, int extra_len,
                                        const char* text) {
    int i = 0;
    if (_app_target_ver >= 3) {
        _out[i++] = RESP_CODE_CONTACT_MSG_RECV_V3;
        _out[i++] = (uint8_t)snr_x4;
        _out[i++] = 0;
        _out[i++] = 0;
    } else {
        _out[i++] = RESP_CODE_CONTACT_MSG_RECV;
    }
    memcpy(&_out[i], pub_key_prefix, 6); i += 6;
    _out[i++] = path_len;
    _out[i++] = txt_type;
    memcpy(&_out[i], &timestamp, 4); i += 4;
    if (extra && extra_len > 0) {
        memcpy(&_out[i], extra, extra_len);
        i += extra_len;
    }
    int tlen = text ? (int)strlen(text) : 0;
    if (i + tlen > MAX_FRAME_SIZE) tlen = MAX_FRAME_SIZE - i;
    if (tlen > 0) memcpy(&_out[i], text, tlen);
    i += tlen;
    addToOfflineQueue(_out, i);
    pushMsgWaiting();
}

void MeckCompanion::pushSendConfirmed(const uint8_t* ack_hash, uint32_t trip_time) {
    if (!_serial || !_serial->isConnected()) return;
    _out[0] = PUSH_CODE_SEND_CONFIRMED;
    memcpy(&_out[1], ack_hash, 4);
    memcpy(&_out[5], &trip_time, 4);
    _serial->writeFrame(_out, 9);
}

void MeckCompanion::pushRxLog(int8_t snr_x4, int8_t rssi,
                              const uint8_t* raw, int len) {
    if (!_serial || !_serial->isConnected()) return;
#if MECK_BLE_GARMIN
    if (_watch_client) return;   // the watch does not want the raw packet log
#endif
    if (len <= 0 || len + 3 > MAX_FRAME_SIZE) return;
    int i = 0;
    _out[i++] = PUSH_CODE_LOG_RX_DATA;
    _out[i++] = (uint8_t)snr_x4;
    _out[i++] = (uint8_t)rssi;
    memcpy(&_out[i], raw, len); i += len;
    _serial->writeFrame(_out, i);
}

void MeckCompanion::pushNewAdvert(const ContactInfo& contact) {
    if (!_serial || !_serial->isConnected()) return;
    writeContactRespFrame(PUSH_CODE_NEW_ADVERT, contact);
}

void MeckCompanion::pushAdvert(const uint8_t* pub_key) {
    if (!_serial || !_serial->isConnected()) return;
    _out[0] = PUSH_CODE_ADVERT;
    memcpy(&_out[1], pub_key, PUB_KEY_SIZE);
    _serial->writeFrame(_out, 1 + PUB_KEY_SIZE);
}

void MeckCompanion::pushPathUpdated(const uint8_t* pub_key) {
    if (!_serial || !_serial->isConnected()) return;
    _out[0] = PUSH_CODE_PATH_UPDATED;
    memcpy(&_out[1], pub_key, PUB_KEY_SIZE);
    _serial->writeFrame(_out, 1 + PUB_KEY_SIZE);
}

// ---------------------------------------------------------------------------
// Repeater admin pushes
// ---------------------------------------------------------------------------
//
// Frame layouts mirror upstream MyMesh::onContactResponse() byte-for-byte so
// the app parses them with no protocol-version negotiation needed.

void MeckCompanion::pushLoginSuccess(const uint8_t* pub_key, uint32_t server_clock,
                                     uint8_t is_admin, uint8_t acl_permissions,
                                     uint8_t fw_ver_level) {
    if (!_serial || !_serial->isConnected()) return;
    int i = 0;
    _out[i++] = PUSH_CODE_LOGIN_SUCCESS;
    _out[i++] = is_admin;                       // legacy is_admin / permissions byte
    memcpy(&_out[i], pub_key, 6); i += 6;       // pub_key_prefix
    memcpy(&_out[i], &server_clock, 4); i += 4; // server timestamp (response tag)
    _out[i++] = acl_permissions;                // v7 ACL permissions byte
    _out[i++] = fw_ver_level;                   // firmware version level
    _serial->writeFrame(_out, i);
}

void MeckCompanion::pushLoginFail(const uint8_t* pub_key) {
    if (!_serial || !_serial->isConnected()) return;
    int i = 0;
    _out[i++] = PUSH_CODE_LOGIN_FAIL;
    _out[i++] = 0;                              // reserved
    memcpy(&_out[i], pub_key, 6); i += 6;
    _serial->writeFrame(_out, i);
}

void MeckCompanion::pushStatusResponse(const uint8_t* pub_key,
                                       const uint8_t* payload, uint8_t payload_len) {
    if (!_serial || !_serial->isConnected()) return;
    int i = 0;
    _out[i++] = PUSH_CODE_STATUS_RESPONSE;
    _out[i++] = 0;                              // reserved
    memcpy(&_out[i], pub_key, 6); i += 6;
    if (payload_len > 0 && i + (int)payload_len <= MAX_FRAME_SIZE) {
        memcpy(&_out[i], payload, payload_len);
        i += payload_len;
    }
    _serial->writeFrame(_out, i);
}

void MeckCompanion::pushCliReply(const uint8_t* pub_key, uint8_t path_len,
                                 uint32_t timestamp, int8_t snr_x4,
                                 const char* text) {
    // CLI replies travel to the app via the same offline-queue path used
    // for ordinary contact messages, tagged with TXT_TYPE_CLI_DATA so the
    // app routes them to its repeater admin view.
    pushContactMessage(pub_key, path_len, TXT_TYPE_CLI_DATA,
                       timestamp, snr_x4, nullptr, 0, text);
}

void MeckCompanion::pushBinaryResponse(uint32_t tag,
                                       const uint8_t* payload, uint8_t payload_len) {
    if (!_serial || !_serial->isConnected()) return;
    int i = 0;
    _out[i++] = PUSH_CODE_BINARY_RESPONSE;
    _out[i++] = 0;                          // reserved
    memcpy(&_out[i], &tag, 4); i += 4;      // app matches reply by tag
    if (payload_len > 0 && i + (int)payload_len <= MAX_FRAME_SIZE) {
        memcpy(&_out[i], payload, payload_len);
        i += payload_len;
    }
    _serial->writeFrame(_out, i);
}
void MeckCompanion::pushControlData(int8_t snr_x4, int8_t rssi, uint8_t path_len,
                                    const uint8_t* payload, int payload_len) {
    if (!_serial || !_serial->isConnected()) return;   // upstream: dropped while app offline
    if (payload_len < 0 || payload_len + 4 > MAX_FRAME_SIZE) return;
    int i = 0;
    _out[i++] = PUSH_CODE_CONTROL_DATA;
    _out[i++] = (uint8_t)snr_x4;
    _out[i++] = (uint8_t)rssi;
    _out[i++] = path_len;
    memcpy(&_out[i], payload, payload_len); i += payload_len;
    _serial->writeFrame(_out, i);
}

// ---------------------------------------------------------------------------
// Pushes for the app commands ported from Meck. Frame layouts as Meck's
// MyMesh (onTraceRecv, onContactPathRecv, onContactResponse, onRawDataRecv,
// onChannelDataRecv, onContactOverwrite, onContactsFull).
// ---------------------------------------------------------------------------

void MeckCompanion::pushTraceData(uint8_t path_len, uint8_t flags, uint32_t tag,
                                  uint32_t auth_code, const uint8_t* path_hashes,
                                  const uint8_t* path_snrs, int8_t final_snr_x4) {
    if (!_serial || !_serial->isConnected()) return;
    uint8_t path_sz = flags & 0x03;  // hash size as a shift
    if (12 + path_len + (path_len >> path_sz) + 1 > (int)sizeof(_out)) return;  // path too long
    int i = 0;
    _out[i++] = PUSH_CODE_TRACE_DATA;
    _out[i++] = 0;  // reserved
    _out[i++] = path_len;
    _out[i++] = flags;
    memcpy(&_out[i], &tag, 4); i += 4;
    memcpy(&_out[i], &auth_code, 4); i += 4;
    memcpy(&_out[i], path_hashes, path_len); i += path_len;
    memcpy(&_out[i], path_snrs, path_len >> path_sz); i += path_len >> path_sz;
    _out[i++] = (uint8_t)final_snr_x4;  // extra/final SNR (to this node)
    _serial->writeFrame(_out, i);
}

void MeckCompanion::pushPathDiscovery(const uint8_t* pub_key,
                                      const uint8_t* out_path, uint8_t out_path_len,
                                      const uint8_t* in_path, uint8_t in_path_len) {
    if (!_serial || !_serial->isConnected()) return;
    int i = 0;
    _out[i++] = PUSH_CODE_PATH_DISCOVERY_RESPONSE;
    _out[i++] = 0;  // reserved
    memcpy(&_out[i], pub_key, 6); i += 6;  // pub_key_prefix
    _out[i++] = out_path_len;
    i += mesh::Packet::writePath(&_out[i], out_path, out_path_len);
    _out[i++] = in_path_len;
    i += mesh::Packet::writePath(&_out[i], in_path, in_path_len);
    _serial->writeFrame(_out, i);
}

void MeckCompanion::pushTelemetryResponse(const uint8_t* pub_key,
                                          const uint8_t* payload, uint8_t payload_len) {
    if (!_serial || !_serial->isConnected()) return;
    int i = 0;
    _out[i++] = PUSH_CODE_TELEMETRY_RESPONSE;
    _out[i++] = 0;  // reserved
    memcpy(&_out[i], pub_key, 6); i += 6;  // pub_key_prefix
    if (i + (int)payload_len > MAX_FRAME_SIZE) return;
    memcpy(&_out[i], payload, payload_len); i += payload_len;
    _serial->writeFrame(_out, i);
}

void MeckCompanion::pushRawData(int8_t snr_x4, int8_t rssi,
                                const uint8_t* payload, int payload_len) {
    if (!_serial || !_serial->isConnected()) return;
    if (payload_len < 0 || payload_len + 4 > MAX_FRAME_SIZE) return;
    int i = 0;
    _out[i++] = PUSH_CODE_RAW_DATA;
    _out[i++] = (uint8_t)snr_x4;
    _out[i++] = (uint8_t)rssi;
    _out[i++] = 0xFF;  // reserved (possibly path_len in future)
    memcpy(&_out[i], payload, payload_len); i += payload_len;
    _serial->writeFrame(_out, i);
}

void MeckCompanion::pushChannelData(int8_t snr_x4, uint8_t ch_idx, uint8_t path_len,
                                    uint16_t data_type, const uint8_t* data, int data_len) {
    if (data_len < 0 || data_len > MAX_CHANNEL_DATA_LENGTH) return;  // exceeds frame limit
    int i = 0;
    _out[i++] = RESP_CODE_CHANNEL_DATA_RECV;
    _out[i++] = (uint8_t)snr_x4;
    _out[i++] = 0;  // reserved1
    _out[i++] = 0;  // reserved2
    _out[i++] = ch_idx;
    _out[i++] = path_len;
    _out[i++] = (uint8_t)(data_type & 0xFF);
    _out[i++] = (uint8_t)(data_type >> 8);
    _out[i++] = (uint8_t)data_len;
    if (data_len > 0) {
        memcpy(&_out[i], data, data_len); i += data_len;
    }
    addToOfflineQueue(_out, i);
    pushMsgWaiting();
}

void MeckCompanion::pushContactDeleted(const uint8_t* pub_key) {
    if (!_serial || !_serial->isConnected()) return;
    _out[0] = PUSH_CODE_CONTACT_DELETED;
    memcpy(&_out[1], pub_key, PUB_KEY_SIZE);
    _serial->writeFrame(_out, 1 + PUB_KEY_SIZE);
}

void MeckCompanion::pushContactsFull() {
    if (!_serial || !_serial->isConnected()) return;
    _out[0] = PUSH_CODE_CONTACTS_FULL;
    _serial->writeFrame(_out, 1);
}
