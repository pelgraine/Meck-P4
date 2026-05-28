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
#include <helpers/BaseSerialInterface.h>
#include <helpers/ContactInfo.h>
#include <helpers/ChannelDetails.h>

class MeckCompanion {
public:
    MeckCompanion() : _mesh(nullptr), _serial(nullptr),
                      _iter_started(false), _iter_filter_since(0),
                      _most_recent_lastmod(0), _app_target_ver(0),
                      _offline_queue_len(0) {}

    void begin(Meck* mesh, BaseSerialInterface* serial) {
        _mesh = mesh;
        _serial = serial;
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

    // Offline queue
    static constexpr int OFFLINE_QUEUE_SIZE = 32;
    struct Frame {
        uint8_t len;
        uint8_t buf[MAX_FRAME_SIZE];
    };
    Frame _offline_queue[OFFLINE_QUEUE_SIZE];
    int _offline_queue_len;

    // Frame buffers
    uint8_t _cmd[MAX_FRAME_SIZE + 1];
    uint8_t _out[MAX_FRAME_SIZE + 1];
};