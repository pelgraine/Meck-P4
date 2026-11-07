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

#if MECK_BLE_GARMIN
// Small-MTU slicing for the watch: the same tags as Meck and the garmin
// branch. No real frame starts with one of these (command codes are below
// 0x80, push codes stop well short of 0xF0).
#define SLICE_TAG_FIRST   0xF0   // first slice of a multi-slice frame
#define SLICE_TAG_MIDDLE  0xF1
#define SLICE_TAG_LAST    0xF2
#define SLICE_TAG_SINGLE  0xF3   // whole frame fits in one slice
#define SLICE_TAG_MASK    0xFC   // (b & MASK) == SLICE_TAG_FIRST matches any tag
#define SPLIT_MAX_MTU     23     // slice outgoing frames when the peer MTU is at or below this

// Heart-rate sensor advertising, as Meck's MECK_BLE_SENSOR_ADVERT.
#define SENSOR_SVC_UUID        0x180D   // Heart Rate
#define SENSOR_CHR_HRM_UUID    0x2A37   // Heart Rate Measurement (notify)
#define SENSOR_CHR_LOC_UUID    0x2A38   // Body Sensor Location (read)
#define SENSOR_APPEARANCE      0x0341   // Heart Rate Belt
#define ADV_NAME_MAX_BYTES     18       // 31 - flags(3) - 16-bit svc(4) - appearance(4) - name header(2)

static const ble_uuid16_t HR_SVC_UUID = BLE_UUID16_INIT(SENSOR_SVC_UUID);
static const ble_uuid16_t HR_HRM_UUID = BLE_UUID16_INIT(SENSOR_CHR_HRM_UUID);
static const ble_uuid16_t HR_LOC_UUID = BLE_UUID16_INIT(SENSOR_CHR_LOC_UUID);

// NimBLE takes the access rules of every notification switch (CCCD) from one
// setting, applied when the host starts. Declared only in a private NimBLE
// header (ble_gatt_priv.h), so declared here.
extern "C" void ble_gatts_set_clt_cfg_perm_flags(uint8_t flags);
#endif

static SerialNimBLEInterface* s_ble = nullptr;   // the one instance
static uint16_t s_tx_val_handle = 0;              // filled by ble_gatts_add_svcs()

// GATT tables, filled in startNimBLE() (field-by-field, so C++ needs no
// designated initialisers). NimBLE keeps pointers to these: they must live
// for the whole session.
static struct ble_gatt_chr_def s_nus_chrs[3];
#if MECK_BLE_GARMIN
static struct ble_gatt_chr_def s_hr_chrs[3];
static struct ble_gatt_svc_def s_nus_svcs[3];    // UART, Heart Rate, end of list
#else
static struct ble_gatt_svc_def s_nus_svcs[2];    // UART, end of list
#endif

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

#if MECK_BLE_GARMIN
// Heart Rate service: Body Sensor Location reads "chest" (1). The measurement
// is notify-only and never sent.
static int hr_access_cb(uint16_t conn_handle, uint16_t attr_handle,
                        struct ble_gatt_access_ctxt* ctxt, void* arg)
{
    if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
        if (ble_uuid_cmp(ctxt->chr->uuid, &HR_LOC_UUID.u) == 0) {
            uint8_t loc_chest = 1;
            return os_mbuf_append(ctxt->om, &loc_chest, 1) == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
        }
        uint8_t hrm[2] = { 0x00, 0x00 };   // flags byte + 8-bit value 0
        return os_mbuf_append(ctxt->om, hrm, 2) == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
    }
    return BLE_ATT_ERR_UNLIKELY;
}
#endif

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
#if MECK_BLE_GARMIN
    _slice_tx_off = 0;
    _slice_rx_len = -1;
#endif
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

#if MECK_BLE_GARMIN
    // No PIN: the watch can only reach an encrypted link without a passkey
    // ("just works"), so pairing is Secure Connections, bonded, no MITM
    // (Meck's MECK_BLE_JUST_WORKS: SC_BOND instead of SC_MITM_BOND).
    ble_hs_cfg.sm_io_cap = BLE_HS_IO_NO_INPUT_OUTPUT;
    ble_hs_cfg.sm_bonding = 1;
    ble_hs_cfg.sm_mitm = 0;
    ble_hs_cfg.sm_sc = 1;
    // Every notification switch (CCCD) needs an encrypted link, so the
    // watch's sensor menu has to pair before it can subscribe to the heart-
    // rate measurement (on the Max that refusal is what starts its pairing),
    // and the watch app meets the same refusal on the UART TX switch as on
    // nRF52 nodes, which it already answers by pairing.
    ble_gatts_set_clt_cfg_perm_flags(BLE_ATT_F_READ | BLE_ATT_F_WRITE |
                                     BLE_ATT_F_READ_ENC | BLE_ATT_F_WRITE_ENC);
    const ble_gatt_chr_flags rx_sec = BLE_GATT_CHR_F_WRITE_ENC;
    const ble_gatt_chr_flags tx_sec = BLE_GATT_CHR_F_READ_ENC;
    BLE_LOG("Garmin watch support on: pairing without a PIN, heart-rate sensor advert");
#else
    // Static PIN: shown on the device, typed on the phone (passkey entry),
    // Secure Connections with MITM protection, bonded. Matches Meck's
    // ESP_LE_AUTH_REQ_SC_MITM_BOND on the T-Deck.
    ble_hs_cfg.sm_io_cap = BLE_HS_IO_DISPLAY_ONLY;
    ble_hs_cfg.sm_bonding = 1;
    ble_hs_cfg.sm_mitm = 1;
    ble_hs_cfg.sm_sc = 1;
    const ble_gatt_chr_flags rx_sec = BLE_GATT_CHR_F_WRITE_ENC | BLE_GATT_CHR_F_WRITE_AUTHEN;
    const ble_gatt_chr_flags tx_sec = BLE_GATT_CHR_F_READ_ENC | BLE_GATT_CHR_F_READ_AUTHEN;
#endif
    ble_hs_cfg.sm_our_key_dist = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;
    ble_hs_cfg.sm_their_key_dist = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;

    memset(s_nus_chrs, 0, sizeof(s_nus_chrs));
    memset(s_nus_svcs, 0, sizeof(s_nus_svcs));
    // RX: the app writes frames here
    s_nus_chrs[0].uuid = &NUS_RX_UUID.u;
    s_nus_chrs[0].access_cb = nus_access_cb;
    s_nus_chrs[0].flags = BLE_GATT_CHR_F_WRITE | rx_sec;
    // TX: frames to the app go out as notifications
    s_nus_chrs[1].uuid = &NUS_TX_UUID.u;
    s_nus_chrs[1].access_cb = nus_access_cb;
    s_nus_chrs[1].flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY | tx_sec;
    s_nus_chrs[1].val_handle = &s_tx_val_handle;
    // s_nus_chrs[2] stays zeroed: end of list
    s_nus_svcs[0].type = BLE_GATT_SVC_TYPE_PRIMARY;
    s_nus_svcs[0].uuid = &NUS_SVC_UUID.u;
    s_nus_svcs[0].characteristics = s_nus_chrs;
#if MECK_BLE_GARMIN
    // Heart Rate service: notify-only measurement plus body sensor location
    // (encrypted read), as Meck's MECK_BLE_SENSOR_ADVERT.
    memset(s_hr_chrs, 0, sizeof(s_hr_chrs));
    s_hr_chrs[0].uuid = &HR_HRM_UUID.u;
    s_hr_chrs[0].access_cb = hr_access_cb;
    s_hr_chrs[0].flags = BLE_GATT_CHR_F_NOTIFY;
    s_hr_chrs[1].uuid = &HR_LOC_UUID.u;
    s_hr_chrs[1].access_cb = hr_access_cb;
    s_hr_chrs[1].flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_READ_ENC;
    // s_hr_chrs[2] stays zeroed: end of list
    s_nus_svcs[1].type = BLE_GATT_SVC_TYPE_PRIMARY;
    s_nus_svcs[1].uuid = &HR_SVC_UUID.u;
    s_nus_svcs[1].characteristics = s_hr_chrs;
    // s_nus_svcs[2] stays zeroed: end of list
#else
    // s_nus_svcs[1] stays zeroed: end of list
#endif

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

#if MECK_BLE_GARMIN
    // Advert: flags, the Heart Rate service, appearance "Heart Rate Belt" and
    // the name (shortened to 18 bytes at a UTF-8 character boundary if it does
    // not fit), so the watch's Sensors & Accessories search lists the node.
    // Scan response: the MeshCore (Nordic UART) service UUID, so phone apps
    // that look for it still find the node. As Meck's MECK_BLE_SENSOR_ADVERT.
    struct ble_hs_adv_fields fields;
    memset(&fields, 0, sizeof(fields));
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.uuids16 = &HR_SVC_UUID;
    fields.num_uuids16 = 1;
    fields.uuids16_is_complete = 1;
    fields.appearance = SENSOR_APPEARANCE;
    fields.appearance_is_present = 1;
    char adv_name[ADV_NAME_MAX_BYTES + 1];
    copy_utf8_trunc(adv_name, sizeof(adv_name), _dev_name);
    fields.name = (uint8_t*)adv_name;
    fields.name_len = (uint8_t)strlen(adv_name);
    fields.name_is_complete = (strcmp(adv_name, _dev_name) == 0) ? 1 : 0;
    int rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        BLE_LOG("setting the advert data failed (rc=%d)", rc);
        return;
    }

    struct ble_hs_adv_fields rsp;
    memset(&rsp, 0, sizeof(rsp));
    rsp.uuids128 = &NUS_SVC_UUID;
    rsp.num_uuids128 = 1;
    rsp.uuids128_is_complete = 1;
    rc = ble_gap_adv_rsp_set_fields(&rsp);
#else
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
#endif
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
#if MECK_BLE_GARMIN
    // A sliced write from a small-MTU peer (the watch): collect the slices
    // and queue the frame once its last slice is in. Same rules as Meck and
    // the garmin branch.
    if ((data[0] & SLICE_TAG_MASK) == SLICE_TAG_FIRST) {   // any SLICE_TAG_*
        uint8_t tag = data[0];
        if (tag == SLICE_TAG_FIRST || tag == SLICE_TAG_SINGLE) _slice_rx_len = 0;
        if (_slice_rx_len < 0) return;               // middle/last with no start: drop
        if (len < 2) return;                         // no length byte
        int n = data[1];
        if (n > len - 2) n = len - 2;                // never read past the write
        if (_slice_rx_len + n > MAX_FRAME_SIZE) { _slice_rx_len = -1; return; }   // oversize: drop
        memcpy(&_slice_rx_buf[_slice_rx_len], data + 2, n);
        _slice_rx_len += n;
        if (tag == SLICE_TAG_LAST || tag == SLICE_TAG_SINGLE) {
            Frame sf;
            sf.len = (uint8_t)_slice_rx_len;
            memcpy(sf.buf, _slice_rx_buf, _slice_rx_len);
            if (xQueueSend(_recv_queue, &sf, 0) != pdTRUE) {
                BLE_LOG("receive queue full, sliced frame dropped");
            }
            _slice_rx_len = -1;
        }
        return;
    }
#endif
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
// Garmin build: encryption alone is enough (no PIN), and a peer still at the
// 23-byte MTU is taken to be the watch and gets the relaxed interval; any
// larger MTU is a phone and gets the fast one (also asked for from the MTU
// event if the phone raises its MTU later), as Meck's MECK_BLE_SLOW_INTERVAL.
void SerialNimBLEInterface::onLinkSecured(uint16_t conn_handle)
{
    struct ble_gap_conn_desc desc;
    if (ble_gap_conn_find(conn_handle, &desc) != 0) return;
#if MECK_BLE_GARMIN
    if (!desc.sec_state.encrypted) {
        BLE_LOG("link not encrypted, frames stay blocked");
        return;
    }
#else
    if (!desc.sec_state.encrypted || !desc.sec_state.authenticated) {
        BLE_LOG("link encrypted but not authenticated, frames stay blocked");
        return;
    }
#endif
    _paired_conn = true;
    BLE_LOG("paired link ready (bonded=%d)", (int)desc.sec_state.bonded);

#if MECK_BLE_GARMIN
    requestInterval(conn_handle, ble_att_mtu(conn_handle) > SPLIT_MAX_MTU);
#else
    requestInterval(conn_handle, true);
#endif
}

// Fast: 15-20 ms, no latency, 4 s timeout (contact sync with a phone).
// Relaxed (Garmin build, the watch): 100-200 ms, latency 2, 6 s timeout.
void SerialNimBLEInterface::requestInterval(uint16_t conn_handle, bool fast)
{
    struct ble_gap_upd_params p;
    memset(&p, 0, sizeof(p));
    if (fast) {
        p.itvl_min = 12;                // 15 ms (units of 1.25 ms)
        p.itvl_max = 16;                // 20 ms
        p.latency = 0;
        p.supervision_timeout = 400;    // 4 s (units of 10 ms)
    } else {
        p.itvl_min = 80;                // 100 ms
        p.itvl_max = 160;               // 200 ms
        p.latency = 2;                  // the peer may skip 2 intervals when idle
        p.supervision_timeout = 600;    // 6 s
    }
    p.min_ce_len = 0;
    p.max_ce_len = 0;
    ble_gap_update_params(conn_handle, &p);
#if MECK_BLE_GARMIN
    BLE_LOG("asked for the %s connection interval (MTU %u)",
            fast ? "fast" : "relaxed", (unsigned)ble_att_mtu(conn_handle));
#endif
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
#if MECK_BLE_GARMIN
            _slice_rx_len = -1;
#endif
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
#if MECK_BLE_GARMIN
        // A phone that raises its MTU after pairing moves to the fast interval.
        if (_paired_conn && event->mtu.conn_handle == _conn_handle &&
            event->mtu.value > SPLIT_MAX_MTU) {
            requestInterval(event->mtu.conn_handle, true);
        }
#endif
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
#if MECK_BLE_GARMIN
    _slice_tx_off = 0;
#endif
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
#if MECK_BLE_GARMIN
    _slice_tx_off = 0;
#endif
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
#if MECK_BLE_GARMIN
    if (!_paired_conn) _slice_tx_off = 0;   // a new link starts each frame from its first byte
#endif

    // Send the oldest queued frame, spaced like Meck. If NimBLE is out of
    // buffers the frame stays queued and is tried again next pass.
    if (_send_queue_len > 0 && _paired_conn && millis() >= _last_write + BLE_WRITE_MIN_INTERVAL) {
        uint16_t ch = _conn_handle;
        const uint8_t* out = _send_queue[0].buf;
        int out_len = _send_queue[0].len;
        bool frame_done = true;             // this notification finishes the frame
#if MECK_BLE_GARMIN
        // Small-MTU peer (the watch): send the next slice of the frame at the
        // head of the queue, one slice per pass. The frame leaves the queue
        // only once its last slice has gone.
        uint8_t chunk[2 + MAX_FRAME_SIZE];
        int slice_n = -1;                   // -1 = sending the whole frame
        uint16_t mtu = ble_att_mtu(ch);
        // A frame already part-sent as slices is finished as slices, even if
        // the MTU has gone up since.
        if ((mtu != 0 && mtu <= SPLIT_MAX_MTU) || _slice_tx_off != 0) {
            if (mtu == 0 || mtu > SPLIT_MAX_MTU) mtu = SPLIT_MAX_MTU;
            int room = (int)mtu - 3 - 2;    // ATT payload minus tag and length bytes
            if (room < 1) room = 1;
            int total = _send_queue[0].len;
            int n = total - _slice_tx_off;
            if (n > room) n = room;
            bool first = (_slice_tx_off == 0);
            frame_done = (_slice_tx_off + n >= total);
            chunk[0] = (first && frame_done) ? SLICE_TAG_SINGLE
                     : first      ? SLICE_TAG_FIRST
                     : frame_done ? SLICE_TAG_LAST : SLICE_TAG_MIDDLE;
            chunk[1] = (uint8_t)n;
            memcpy(&chunk[2], &_send_queue[0].buf[_slice_tx_off], n);
            out = chunk;
            out_len = n + 2;
            slice_n = n;
        }
#endif
        struct os_mbuf* om = ble_hs_mbuf_from_flat(out, out_len);
        if (om && ch != BLE_HS_CONN_HANDLE_NONE) {
            int rc = ble_gatts_notify_custom(ch, s_tx_val_handle, om);   // consumes om
            if (rc == 0) {
                _last_write = millis();
#if MECK_BLE_GARMIN
                if (slice_n >= 0) {
                    _slice_tx_off = frame_done ? 0 : (uint16_t)(_slice_tx_off + slice_n);
                }
#endif
                if (frame_done) {
                    _send_queue_len--;
                    if (_send_queue_len > 0) {
                        memmove(&_send_queue[0], &_send_queue[1], _send_queue_len * sizeof(Frame));
                    }
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
