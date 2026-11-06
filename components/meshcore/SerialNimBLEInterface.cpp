/*
 * SerialNimBLEInterface.cpp -- MeshCore Bluetooth companion transport on the P4
 *
 * See SerialNimBLEInterface.h for the design and threading model.
 */

#include "SerialNimBLEInterface.h"

#if MECK_BLE_ENABLED

#include <string.h>
#include "esp_heap_caps.h"
#include "freertos/idf_additions.h"   // xQueueCreateWithCaps

#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/ble_gap.h"
#include "host/ble_gatt.h"
#include "host/ble_sm.h"
#include "host/ble_store.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

// ESP-IDF's NimBLE has no header declaring this; its examples declare it too.
extern "C" void ble_store_config_init(void);

#define BLE_LOG(fmt, ...) printf("BLE: " fmt "\n", ##__VA_ARGS__)

#define ADVERT_RESTART_DELAY_MS  1000   // as Meck's SerialBLEInterface
#define BLE_WRITE_MIN_INTERVAL   7      // ms between notifications, as Meck

// Nordic UART service and characteristics, little-endian byte order
// 6E400001-B5A3-F393-E0A9-E50E24DCCA9E (service), ...02 (RX), ...03 (TX)
static const ble_uuid128_t NUS_SVC_UUID = BLE_UUID128_INIT(
    0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0,
    0x93, 0xf3, 0xa3, 0xb5, 0x01, 0x00, 0x40, 0x6e);
static const ble_uuid128_t NUS_RX_UUID = BLE_UUID128_INIT(
    0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0,
    0x93, 0xf3, 0xa3, 0xb5, 0x02, 0x00, 0x40, 0x6e);
static const ble_uuid128_t NUS_TX_UUID = BLE_UUID128_INIT(
    0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0,
    0x93, 0xf3, 0xa3, 0xb5, 0x03, 0x00, 0x40, 0x6e);

static SerialNimBLEInterface* s_ble = nullptr;   // the one instance
static uint16_t s_tx_val_handle = 0;              // filled by ble_gatts_add_svcs()

// GATT tables, filled in startNimBLE() (field-by-field, so C++ needs no
// designated initialisers). NimBLE keeps pointers to these: they must live
// for the whole session.
static struct ble_gatt_chr_def s_nus_chrs[3];
static struct ble_gatt_svc_def s_nus_svcs[2];

// ---------------------------------------------------------------------------
// C callbacks (NimBLE host task) forwarding to the instance
// ---------------------------------------------------------------------------

static int nus_access_cb(uint16_t conn_handle, uint16_t attr_handle,
                         struct ble_gatt_access_ctxt* ctxt, void* arg)
{
    if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
        uint8_t buf[MAX_FRAME_SIZE];
        uint16_t len = OS_MBUF_PKTLEN(ctxt->om);
        if (len > MAX_FRAME_SIZE) {
            BLE_LOG("write too big (%u bytes), dropped", (unsigned)len);
            return 0;
        }
        uint16_t copied = 0;
        if (ble_hs_mbuf_to_flat(ctxt->om, buf, sizeof(buf), &copied) != 0) {
            return BLE_ATT_ERR_UNLIKELY;
        }
        if (s_ble) s_ble->onRxWrite(buf, copied);
        return 0;
    }
    if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
        return 0;   // TX is read for completeness only; frames go by notification
    }
    return BLE_ATT_ERR_UNLIKELY;
}

static int gap_event_cb(struct ble_gap_event* event, void* arg)
{
    return s_ble ? s_ble->onGapEvent(event) : 0;
}

static void on_sync_cb(void)       { if (s_ble) s_ble->onSync(); }
static void on_reset_cb(int r)     { if (s_ble) s_ble->onReset(r); }

static void host_task(void* param)
{
    nimble_port_run();              // returns only if nimble_port_stop() is called
    nimble_port_freertos_deinit();
}

// Copy src into dst (cap bytes incl. terminator) without splitting a UTF-8
// character, so a node name with emoji is never cut mid-character.
static void copy_utf8_trunc(char* dst, size_t cap, const char* src)
{
    size_t n = strlen(src);
    if (n >= cap) {
        n = cap - 1;
        while (n > 0 && ((uint8_t)src[n] & 0xC0) == 0x80) n--;   // back to a lead byte
    }
    memcpy(dst, src, n);
    dst[n] = '\0';
}

// ---------------------------------------------------------------------------
// Instance
// ---------------------------------------------------------------------------

SerialNimBLEInterface::SerialNimBLEInterface()
    : _pin(0), _begun(false), _nimble_started(false), _enabled(false),
      _synced(false), _own_addr_type(0), _conn_handle(BLE_HS_CONN_HANDLE_NONE),
      _paired_conn(false), _tx_subscribed(false), _adv_restart_at(0),
      _recv_queue(nullptr), _send_queue(nullptr), _send_queue_len(0), _last_write(0)
{
    _dev_name[0] = '\0';
}

void SerialNimBLEInterface::begin(const char* dev_name, uint32_t pin)
{
    copy_utf8_trunc(_dev_name, sizeof(_dev_name), dev_name ? dev_name : "MeshCore");
    _pin = pin;
    s_ble = this;

    if (!_recv_queue) {
        _recv_queue = xQueueCreateWithCaps(FRAME_QUEUE_SIZE, sizeof(Frame), MALLOC_CAP_SPIRAM);
    }
    if (!_send_queue) {
        _send_queue = (Frame*)heap_caps_malloc(sizeof(Frame) * FRAME_QUEUE_SIZE, MALLOC_CAP_SPIRAM);
    }
    _send_queue_len = 0;
    _begun = (_recv_queue != nullptr && _send_queue != nullptr);
    if (!_begun) {
        BLE_LOG("could not allocate the frame buffers in PSRAM");
    }
    BLE_LOG("companion ready as '%s' (Bluetooth starts when enabled)", _dev_name);
}

bool SerialNimBLEInterface::startNimBLE()
{
    esp_err_t err = nimble_port_init();
    if (err != ESP_OK) {
        BLE_LOG("NimBLE did not start (%s)", esp_err_to_name(err));
        return false;
    }

    ble_hs_cfg.reset_cb = on_reset_cb;
    ble_hs_cfg.sync_cb = on_sync_cb;
    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;

    // Static PIN: shown on the device, typed on the phone (passkey entry),
    // Secure Connections with MITM protection, bonded. Matches Meck's
    // ESP_LE_AUTH_REQ_SC_MITM_BOND on the T-Deck.
    ble_hs_cfg.sm_io_cap = BLE_HS_IO_DISPLAY_ONLY;
    ble_hs_cfg.sm_bonding = 1;
    ble_hs_cfg.sm_mitm = 1;
    ble_hs_cfg.sm_sc = 1;
    ble_hs_cfg.sm_our_key_dist = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;
    ble_hs_cfg.sm_their_key_dist = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;

    memset(s_nus_chrs, 0, sizeof(s_nus_chrs));
    memset(s_nus_svcs, 0, sizeof(s_nus_svcs));
    // RX: the app writes frames here
    s_nus_chrs[0].uuid = &NUS_RX_UUID.u;
    s_nus_chrs[0].access_cb = nus_access_cb;
    s_nus_chrs[0].flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_ENC | BLE_GATT_CHR_F_WRITE_AUTHEN;
    // TX: frames to the app go out as notifications
    s_nus_chrs[1].uuid = &NUS_TX_UUID.u;
    s_nus_chrs[1].access_cb = nus_access_cb;
    s_nus_chrs[1].flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY |
                          BLE_GATT_CHR_F_READ_ENC | BLE_GATT_CHR_F_READ_AUTHEN;
    s_nus_chrs[1].val_handle = &s_tx_val_handle;
    // s_nus_chrs[2] stays zeroed: end of list
    s_nus_svcs[0].type = BLE_GATT_SVC_TYPE_PRIMARY;
    s_nus_svcs[0].uuid = &NUS_SVC_UUID.u;
    s_nus_svcs[0].characteristics = s_nus_chrs;
    // s_nus_svcs[1] stays zeroed: end of list

    ble_svc_gap_init();
    ble_svc_gatt_init();
    int rc = ble_gatts_count_cfg(s_nus_svcs);
    if (rc == 0) rc = ble_gatts_add_svcs(s_nus_svcs);
    if (rc != 0) {
        BLE_LOG("adding the MeshCore service failed (rc=%d)", rc);
        return false;
    }
    rc = ble_svc_gap_device_name_set(_dev_name);
    if (rc != 0) {
        BLE_LOG("setting the device name failed (rc=%d)", rc);
    }

    ble_store_config_init();        // bonds (kept in NVS when NVS persist is on)
    nimble_port_freertos_init(host_task);
    BLE_LOG("NimBLE started, waiting for it to sync with the C6");
    return true;
}

void SerialNimBLEInterface::startAdvertising()
{
    if (!_synced || !_enabled) return;
    if (_conn_handle != BLE_HS_CONN_HANDLE_NONE) return;
    if (ble_gap_adv_active()) return;

    // Advert: flags + the MeshCore (Nordic UART) service UUID.
    struct ble_hs_adv_fields fields;
    memset(&fields, 0, sizeof(fields));
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.uuids128 = &NUS_SVC_UUID;
    fields.num_uuids128 = 1;
    fields.uuids128_is_complete = 1;
    int rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        BLE_LOG("setting the advert data failed (rc=%d)", rc);
        return;
    }

    // Scan response: the device name (shortened if it does not fit).
    struct ble_hs_adv_fields rsp;
    memset(&rsp, 0, sizeof(rsp));
    char name[30];
    copy_utf8_trunc(name, sizeof(name), _dev_name);   // 29 bytes max in a scan response
    rsp.name = (uint8_t*)name;
    rsp.name_len = (uint8_t)strlen(name);
    rsp.name_is_complete = (strcmp(name, _dev_name) == 0) ? 1 : 0;
    rc = ble_gap_adv_rsp_set_fields(&rsp);
    if (rc != 0) {
        BLE_LOG("setting the scan response failed (rc=%d)", rc);
        return;
    }

    struct ble_gap_adv_params adv;
    memset(&adv, 0, sizeof(adv));
    adv.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv.disc_mode = BLE_GAP_DISC_MODE_GEN;
    rc = ble_gap_adv_start(_own_addr_type, NULL, BLE_HS_FOREVER, &adv, gap_event_cb, NULL);
    if (rc != 0 && rc != BLE_HS_EALREADY) {
        BLE_LOG("advertising failed to start (rc=%d)", rc);
        return;
    }
    BLE_LOG("advertising as '%s'", _dev_name);
}

void SerialNimBLEInterface::onSync()
{
    int rc = ble_hs_util_ensure_addr(0);
    if (rc == 0) rc = ble_hs_id_infer_auto(0, &_own_addr_type);
    if (rc != 0) {
        BLE_LOG("no Bluetooth address available (rc=%d)", rc);
        return;
    }
    uint8_t addr[6] = {0};
    ble_hs_id_copy_addr(_own_addr_type, addr, NULL);
    BLE_LOG("in sync with the C6, address %02x:%02x:%02x:%02x:%02x:%02x",
            addr[5], addr[4], addr[3], addr[2], addr[1], addr[0]);
    _synced = true;
    startAdvertising();
}

void SerialNimBLEInterface::onReset(int reason)
{
    BLE_LOG("host reset (reason %d)", reason);
    _synced = false;
}

void SerialNimBLEInterface::onRxWrite(const uint8_t* data, int len)
{
    if (!_recv_queue || len <= 0) return;
    Frame f;
    f.len = (uint8_t)len;
    memcpy(f.buf, data, len);
    if (xQueueSend(_recv_queue, &f, 0) != pdTRUE) {
        BLE_LOG("receive queue full, frame dropped");
    }
}

// Encryption is up: if it is also authenticated (PIN pairing, or a bonded
// phone reconnecting), frames may flow. Then ask for a fast connection
// interval for contact sync, as Meck does (15-20 ms, 4 s timeout).
void SerialNimBLEInterface::onLinkSecured(uint16_t conn_handle)
{
    struct ble_gap_conn_desc desc;
    if (ble_gap_conn_find(conn_handle, &desc) != 0) return;
    if (!desc.sec_state.encrypted || !desc.sec_state.authenticated) {
        BLE_LOG("link encrypted but not authenticated, frames stay blocked");
        return;
    }
    _paired_conn = true;
    BLE_LOG("paired link ready (bonded=%d)", (int)desc.sec_state.bonded);

    struct ble_gap_upd_params p;
    memset(&p, 0, sizeof(p));
    p.itvl_min = 12;                // 15 ms (units of 1.25 ms)
    p.itvl_max = 16;                // 20 ms
    p.latency = 0;
    p.supervision_timeout = 400;    // 4 s (units of 10 ms)
    p.min_ce_len = 0;
    p.max_ce_len = 0;
    ble_gap_update_params(conn_handle, &p);
}

int SerialNimBLEInterface::onGapEvent(void* ev)
{
    struct ble_gap_event* event = (struct ble_gap_event*)ev;

    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status == 0) {
            _conn_handle = event->connect.conn_handle;
            _paired_conn = false;
            _tx_subscribed = false;
            _adv_restart_at = 0;
            BLE_LOG("phone connected");
        } else {
            BLE_LOG("connection attempt failed (status %d)", event->connect.status);
            _conn_handle = BLE_HS_CONN_HANDLE_NONE;
            _adv_restart_at = millis() + ADVERT_RESTART_DELAY_MS;
        }
        return 0;

    case BLE_GAP_EVENT_DISCONNECT:
        BLE_LOG("phone disconnected (reason %d)", event->disconnect.reason);
        _conn_handle = BLE_HS_CONN_HANDLE_NONE;
        _paired_conn = false;
        _tx_subscribed = false;
        if (_enabled) _adv_restart_at = millis() + ADVERT_RESTART_DELAY_MS;
        return 0;

    case BLE_GAP_EVENT_ENC_CHANGE:
        if (event->enc_change.status == 0) {
            onLinkSecured(event->enc_change.conn_handle);
        } else {
            BLE_LOG("pairing or encryption failed (status %d)", event->enc_change.status);
        }
        return 0;

    case BLE_GAP_EVENT_PASSKEY_ACTION:
        if (event->passkey.params.action == BLE_SM_IOACT_DISP) {
            struct ble_sm_io pk;
            memset(&pk, 0, sizeof(pk));
            pk.action = BLE_SM_IOACT_DISP;
            pk.passkey = _pin;
            int rc = ble_sm_inject_io(event->passkey.conn_handle, &pk);
            BLE_LOG("pairing: phone asked for the PIN (shown on the home screen)%s",
                    rc == 0 ? "" : ", PIN hand-over failed");
        } else {
            BLE_LOG("pairing: unexpected passkey action %d", (int)event->passkey.params.action);
        }
        return 0;

    case BLE_GAP_EVENT_REPEAT_PAIRING: {
        // The phone has forgotten our bond but we still hold one: drop ours
        // and let pairing start again (standard NimBLE handling).
        struct ble_gap_conn_desc desc;
        if (ble_gap_conn_find(event->repeat_pairing.conn_handle, &desc) == 0) {
            ble_store_util_delete_peer(&desc.peer_id_addr);
        }
        BLE_LOG("pairing again with a phone that lost its bond");
        return BLE_GAP_REPEAT_PAIRING_RETRY;
    }

    case BLE_GAP_EVENT_SUBSCRIBE:
        if (event->subscribe.attr_handle == s_tx_val_handle) {
            _tx_subscribed = event->subscribe.cur_notify ? true : false;
        }
        return 0;

    case BLE_GAP_EVENT_MTU:
        BLE_LOG("MTU %u", (unsigned)event->mtu.value);
        return 0;

    case BLE_GAP_EVENT_ADV_COMPLETE:
        if (_enabled && _conn_handle == BLE_HS_CONN_HANDLE_NONE) {
            _adv_restart_at = millis() + ADVERT_RESTART_DELAY_MS;
        }
        return 0;

    default:
        return 0;
    }
}

// ---------------------------------------------------------------------------
// BaseSerialInterface (meck_task)
// ---------------------------------------------------------------------------

void SerialNimBLEInterface::enable()
{
    if (_enabled) return;
    if (!_begun) {
        BLE_LOG("enable: begin() has not run or its buffers failed");
        return;
    }
    if (!_nimble_started) {
        if (!startNimBLE()) return;
        _nimble_started = true;
    }
    xQueueReset(_recv_queue);
    _send_queue_len = 0;
    _enabled = true;
    _adv_restart_at = 0;
    startAdvertising();             // no-op until the host has synced; onSync() starts it
}

void SerialNimBLEInterface::disable()
{
    bool was = _enabled;
    _enabled = false;
    _adv_restart_at = 0;
    if (was && _nimble_started) {
        ble_gap_adv_stop();
        uint16_t ch = _conn_handle;
        if (ch != BLE_HS_CONN_HANDLE_NONE) {
            ble_gap_terminate(ch, BLE_ERR_REM_USER_CONN_TERM);
        }
    }
    _paired_conn = false;
    if (_recv_queue) xQueueReset(_recv_queue);
    _send_queue_len = 0;
    BLE_LOG("companion off");
}

bool SerialNimBLEInterface::isWriteBusy() const
{
    if (_send_queue_len >= FRAME_QUEUE_SIZE - 1) return true;    // keep a slot spare
    return millis() < _last_write + BLE_WRITE_MIN_INTERVAL;
}

size_t SerialNimBLEInterface::writeFrame(const uint8_t src[], size_t len)
{
    if (len == 0 || len > MAX_FRAME_SIZE) return 0;
    if (!_enabled || !_paired_conn || !_send_queue) return 0;
    if (_send_queue_len >= FRAME_QUEUE_SIZE) {
        BLE_LOG("send queue full, frame dropped");
        return 0;
    }
    _send_queue[_send_queue_len].len = (uint8_t)len;
    memcpy(_send_queue[_send_queue_len].buf, src, len);
    _send_queue_len++;
    return len;
}

size_t SerialNimBLEInterface::checkRecvFrame(uint8_t dest[])
{
    if (!_enabled) return 0;

    // Send the oldest queued frame, spaced like Meck. If NimBLE is out of
    // buffers the frame stays queued and is tried again next pass.
    if (_send_queue_len > 0 && _paired_conn && millis() >= _last_write + BLE_WRITE_MIN_INTERVAL) {
        uint16_t ch = _conn_handle;
        struct os_mbuf* om = ble_hs_mbuf_from_flat(_send_queue[0].buf, _send_queue[0].len);
        if (om && ch != BLE_HS_CONN_HANDLE_NONE) {
            int rc = ble_gatts_notify_custom(ch, s_tx_val_handle, om);   // consumes om
            if (rc == 0) {
                _last_write = millis();
                _send_queue_len--;
                if (_send_queue_len > 0) {
                    memmove(&_send_queue[0], &_send_queue[1], _send_queue_len * sizeof(Frame));
                }
            }
        } else if (om) {
            os_mbuf_free_chain(om);
        }
    }

    // Restart advertising after a disconnect (or a failed connection).
    uint32_t at = _adv_restart_at;
    if (at && millis() >= at) {
        _adv_restart_at = 0;
        startAdvertising();
    }

    // Hand the next received frame to MeckCompanion.
    Frame f;
    if (_recv_queue && xQueueReceive(_recv_queue, &f, 0) == pdTRUE) {
        if (!_paired_conn) return 0;      // never act on frames from an unpaired link
        memcpy(dest, f.buf, f.len);
        return f.len;
    }
    return 0;
}

#endif // MECK_BLE_ENABLED
