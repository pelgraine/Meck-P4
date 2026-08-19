/*
 * MeckCompanion.h -- MeshCore companion protocol handler for T-Display P4
 *
 * Implements the frame-level companion protocol that the MeshCore phone app
 * speaks over the BLE or WiFi serial interface. Ported from upstream MyMesh's
 * checkSerialInterface() / handleCmdFrame().
 *
 * Usage: create one instance, call begin() with pointers to the Meck mesh
 * and a BaseSerialInterface, then call check() every meck_task iteration.
 * Push methods are called from MeckMesh callbacks to notify the app of
 * incoming messages, acks, and contact updates.
 */

#pragma once

#include "MeckMesh.h"
#include "esp_heap_caps.h"
#include <helpers/BaseSerialInterface.h>
#include <helpers/ContactInfo.h>
#include <helpers/ChannelDetails.h>

class MeckCompanion {
public:
    MeckCompanion() : _mesh(nullptr), _serial(nullptr),
                      _iter_started(false), _iter_filter_since(0),
                      _most_recent_lastmod(0), _app_target_ver(0),
                      _offline_queue(nullptr), _offline_head(0), _offline_queue_len(0) {}

    void begin(Meck* mesh, BaseSerialInterface* serial) {
        _mesh = mesh;
        _serial = serial;
        // Offline queue lives in PSRAM (1500 x 173 bytes); allocated once.
        if (!_offline_queue) {
            _offline_queue = (Frame*)heap_caps_malloc(sizeof(Frame) * OFFLINE_QUEUE_SIZE,
                                                      MALLOC_CAP_SPIRAM);
            if (!_offline_queue) {
                printf("MeckCompanion: offline queue PSRAM alloc failed (%u bytes)\n",
                       (unsigned)(sizeof(Frame) * OFFLINE_QUEUE_SIZE));
            }
            _offline_head = 0;
            _offline_queue_len = 0;
        }
    }

    // Call every meck_task loop iteration.
    void check();

    // ---- Push notifications (called from MeckMesh callbacks) ----

    // Channel message received -- queues the frame and tickles the app
    void pushChannelMessage(uint8_t ch_idx, uint8_t path_len,
                            uint32_t timestamp, int8_t snr_x4,
                            const char* text);

    // DM / signed message received
    void pushContactMessage(const uint8_t* pub_key_prefix,
                            uint8_t path_len, uint8_t txt_type,
                            uint32_t timestamp, int8_t snr_x4,
                            const uint8_t* extra, int extra_len,
                            const char* text);

    // ACK received for a sent message
    void pushSendConfirmed(const uint8_t* ack_hash, uint32_t trip_time);
    // New contact discovered
    void pushNewAdvert(const ContactInfo& contact);

    // Existing contact updated (advert seen)
    void pushAdvert(const uint8_t* pub_key);

    // Self-echo heard (path updated)
    void pushPathUpdated(const uint8_t* pub_key);

    // Raw RX packet stream (SNR, RSSI, full frame) for app repeat analysis
    void pushRxLog(int8_t snr_x4, int8_t rssi, const uint8_t* raw, int len);

    // Repeater admin -- login response (success path)
    void pushLoginSuccess(const uint8_t* pub_key, uint32_t server_clock,
                          uint8_t is_admin, uint8_t acl_permissions,
                          uint8_t fw_ver_level);

    // Repeater admin -- login response (failure path)
    void pushLoginFail(const uint8_t* pub_key);

    // Repeater admin -- status response (RepeaterStats payload)
    void pushStatusResponse(const uint8_t* pub_key,
                            const uint8_t* payload, uint8_t payload_len);

    // Repeater admin -- CLI text reply (queued as a contact message with
    // txt_type=TXT_TYPE_CLI_DATA so the app's existing sync flow picks it up)
    void pushCliReply(const uint8_t* pub_key, uint8_t path_len,
                      uint32_t timestamp, int8_t snr_x4, const char* text);

    // Repeater admin -- binary request response (e.g. GET_NEIGHBOURS, ACL).
    // Tag matched on the app side, not pub_key, so the frame carries the
    // 4-byte response tag followed by the request-type-specific payload.
    void pushBinaryResponse(uint32_t tag,
                            const uint8_t* payload, uint8_t payload_len);

private:
    void handleCmdFrame(size_t len);

    // Response helpers
    void writeOKFrame();
    void writeErrFrame(uint8_t err_code);
    void writeContactRespFrame(uint8_t code, const ContactInfo& contact);

    void addToOfflineQueue(const uint8_t frame[], int len);
    int  getFromOfflineQueue(uint8_t frame[]);
    void pushMsgWaiting();

    Meck* _mesh;
    BaseSerialInterface* _serial;

    // Contact iteration state
    ContactsIterator _iter;
    bool _iter_started;
    uint32_t _iter_filter_since;
    uint32_t _most_recent_lastmod;

    // Protocol version the connected app understands
    uint8_t _app_target_ver;

    // Offline queue: messages (channel and DM) queued for the app while no
    // companion is connected, drained by CMD_SYNC_NEXT_MESSAGE. A ring of
    // OFFLINE_QUEUE_SIZE frames in PSRAM (allocated in begin()); when full
    // the oldest frame is dropped. 1500 frames is roughly a full day's
    // traffic on a busy mesh (upstream's static default is 16, sized for
    // boards without PSRAM).
    static constexpr int OFFLINE_QUEUE_SIZE = 1500;
    struct Frame {
        uint8_t len;
        uint8_t buf[MAX_FRAME_SIZE];
    };
    Frame* _offline_queue;      // PSRAM, OFFLINE_QUEUE_SIZE entries
    int _offline_head;          // index of the oldest queued frame
    int _offline_queue_len;

    // Frame buffers
    uint8_t _cmd[MAX_FRAME_SIZE + 1];
    uint8_t _out[MAX_FRAME_SIZE + 1];
};