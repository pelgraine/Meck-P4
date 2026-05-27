/*
 * MeckCompanion.cpp -- MeshCore companion protocol handler for T-Display P4
 *
 * Ported from upstream MyMesh::handleCmdFrame() / checkSerialInterface().
 * Implements the subset of the companion protocol needed for the phone app
 * to connect, sync contacts/channels, and exchange messages.
 */

#include "MeckCompanion.h"
#include "meck.h"
#include <cstdio>
#include <cstring>
#include <helpers/TxtDataHelpers.h>

// strncpy with guaranteed NUL termination (replaces strzcpy)
static void strzcpy(char* dest, const char* src, size_t n) {
    strncpy(dest, src, n - 1);
    dest[n - 1] = '\0';
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
#define CMD_ADD_UPDATE_CONTACT        9
#define CMD_SYNC_NEXT_MESSAGE         10
#define CMD_SET_RADIO_PARAMS          11
#define CMD_SET_RADIO_TX_POWER        12
#define CMD_RESET_PATH                13
#define CMD_REMOVE_CONTACT            15
#define CMD_REBOOT                    19
#define CMD_GET_BATT_AND_STORAGE      20
#define CMD_DEVICE_QEURY              22
#define CMD_GET_CHANNEL               31
#define CMD_SET_CHANNEL               32
#define CMD_GET_AUTOADD_CONFIG        59
#define CMD_SET_PATH_HASH_MODE        61
#define CMD_GET_DEFAULT_FLOOD_SCOPE   64

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
#define RESP_CODE_CHANNEL_INFO        18
#define RESP_CODE_AUTOADD_CONFIG      25
#define RESP_CODE_DEFAULT_FLOOD_SCOPE 28

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
#define PUSH_CODE_NEW_ADVERT          0x8A

// ---- Response codes for received messages (queued in offline queue) ----
#define RESP_CODE_CONTACT_MSG_RECV    7
#define RESP_CODE_CHANNEL_MSG_RECV    8
#define RESP_CODE_CONTACT_MSG_RECV_V3 16
#define RESP_CODE_CHANNEL_MSG_RECV_V3 17

// ---- Additional commands ----
#define CMD_SET_FLOOD_SCOPE_KEY       54

// ---- Firmware identity ----
// Must match MeckMesh.h / upstream FIRMWARE_VER_CODE so the app
// negotiates the right protocol level.
#ifndef FIRMWARE_VER_CODE
#define FIRMWARE_VER_CODE 12
#endif
#ifndef FIRMWARE_BUILD_DATE
#define FIRMWARE_BUILD_DATE "27 May 2026"
#endif
#ifndef FIRMWARE_VERSION
#define FIRMWARE_VERSION "v1.8-p4"
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
        _out[i++] = 0;  // client_repeat (v9+)
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
            _serial->writeFrame(_out, out_len);
        } else {
            _out[0] = RESP_CODE_NO_MORE_MESSAGES;
            _serial->writeFrame(_out, 1);
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
        if (recipient && (txt_type == TXT_TYPE_PLAIN || txt_type == TXT_TYPE_CLI_DATA)) {
            char* text = (char*)&_cmd[i];
            _cmd[len] = 0;  // ensure NUL
            uint32_t est_timeout = 0;
            uint32_t expected_ack = 0;
            int result;
            if (txt_type == TXT_TYPE_CLI_DATA) {
                msg_timestamp = _mesh->getRTCClock()->getCurrentTimeUnique();
                result = _mesh->sendCommandData(*recipient, msg_timestamp, attempt, text, est_timeout);
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
            if (ok && _mesh->sendGroupMessage(msg_timestamp, channel.channel,
                                               _mesh->getNodeName(), text, len - i)) {
                writeOKFrame();
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

    // ---- CMD_ADD_UPDATE_CONTACT (9) ----
    if (cmd == CMD_ADD_UPDATE_CONTACT && len >= 1 + PUB_KEY_SIZE + 3) {
        uint8_t* pub_key = &_cmd[1];
        ContactInfo* existing = _mesh->lookupContactByPubKey(pub_key, PUB_KEY_SIZE);
        if (existing) {
            // Update in place
            int j = 1 + PUB_KEY_SIZE;
            existing->type = _cmd[j++];
            existing->flags = _cmd[j++];
            // Skip out_path_len and out_path from app (device manages paths)
            existing->lastmod = _mesh->getRTCClock()->getCurrentTime();
            _mesh->scheduleLazyContactSave();
            writeOKFrame();
        } else {
            // Add new contact (minimal parse)
            ContactInfo ci;
            memset(&ci, 0, sizeof(ci));
            memcpy(ci.id.pub_key, pub_key, PUB_KEY_SIZE);
            int j = 1 + PUB_KEY_SIZE;
            ci.type = _cmd[j++];
            ci.flags = _cmd[j++];
            ci.out_path_len = OUT_PATH_UNKNOWN;
            // Skip path bytes, read name at offset 1+32+3+MAX_PATH_SIZE
            j++;  // skip out_path_len byte from frame
            j += MAX_PATH_SIZE;
            if (j + 32 <= (int)len) {
                memcpy(ci.name, &_cmd[j], 32);
                ci.name[31] = '\0';
            }
            ci.lastmod = _mesh->getRTCClock()->getCurrentTime();
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

    // ---- CMD_GET_AUTOADD_CONFIG (59) ----
    if (cmd == CMD_GET_AUTOADD_CONFIG) {
        _out[0] = RESP_CODE_AUTOADD_CONFIG;
        _out[1] = prefs->autoadd_config;
        _serial->writeFrame(_out, 2);
        return;
    }

    // ---- CMD_GET_DEFAULT_FLOOD_SCOPE (64) ----
    if (cmd == CMD_GET_DEFAULT_FLOOD_SCOPE) {
        int i = 0;
        _out[i++] = RESP_CODE_DEFAULT_FLOOD_SCOPE;
        strzcpy((char*)&_out[i], prefs->default_scope_name, 31);
        i += 31;
        _serial->writeFrame(_out, i);
        return;
    }

    // ---- CMD_SET_FLOOD_SCOPE_KEY (54) ---- v8+ flood scope
    // Acknowledge so the app doesn't retry. Full scope wiring is a
    // future task (needs send_scope member in MeckMesh).
    if (cmd == CMD_SET_FLOOD_SCOPE_KEY && len >= 2) {
        writeOKFrame();
        return;
    }

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
    if (_offline_queue_len >= OFFLINE_QUEUE_SIZE) {
        // Queue full -- drop oldest
        _offline_queue_len--;
        for (int i = 0; i < _offline_queue_len; i++) {
            _offline_queue[i] = _offline_queue[i + 1];
        }
    }
    _offline_queue[_offline_queue_len].len = (uint8_t)len;
    memcpy(_offline_queue[_offline_queue_len].buf, frame, len);
    _offline_queue_len++;
}

int MeckCompanion::getFromOfflineQueue(uint8_t frame[]) {
    if (_offline_queue_len <= 0) return 0;
    int len = _offline_queue[0].len;
    memcpy(frame, _offline_queue[0].buf, len);
    _offline_queue_len--;
    for (int i = 0; i < _offline_queue_len; i++) {
        _offline_queue[i] = _offline_queue[i + 1];
    }
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
                                        const char* text) {
    int i = 0;
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