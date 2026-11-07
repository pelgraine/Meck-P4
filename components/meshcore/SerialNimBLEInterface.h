/*
 * SerialNimBLEInterface.h -- MeshCore Bluetooth companion transport on the P4
 *
 * For boards whose C6 runs ESP-Hosted: the P4 runs ESP-IDF's NimBLE host and
 * the C6 is its Bluetooth radio, through ESP-Hosted's HCI link over SDIO.
 * Built only when MECK_BLE_ENABLED is 1 (meck.h: the ESP-Hosted C6 build with
 * NimBLE switched on); otherwise this class compiles to nothing.
 *
 * Follows Meck's ESP32 SerialBLEInterface (Arduino BLE, T-Deck):
 *   - the Nordic UART service the MeshCore app looks for
 *     (6E400001-B5A3-F393-E0A9-E50E24DCCA9E), RX 6E400002 (write) and
 *     TX 6E400003 (read + notify), both needing an authenticated link;
 *   - a static 6-digit PIN (passkey shown on the device, typed on the phone),
 *     Secure Connections with MITM protection and bonding;
 *   - one MeshCore frame per write or notification, at most MAX_FRAME_SIZE;
 *   - writes spaced at least 7 ms apart, advertising restarted 1 s after a
 *     disconnect, fast connection interval requested once paired.
 *
 * Threading: NimBLE's host task runs the GAP and GATT callbacks. Received
 * frames go into a FreeRTOS queue; everything else (sending, advertising
 * restarts, enable/disable) runs on meck_task through the BaseSerialInterface
 * calls, as with the WiFi companion. Frame buffers are in PSRAM.
 *
 * NimBLE is started on the first enable() and stays up until the next
 * reboot; disable() stops advertising and drops any connection.
 *
 * MECK_BLE_GARMIN (meck.h, menuconfig "Meck: Garmin watch support") makes the
 * companion usable from a Garmin watch running the Meck Connect IQ app, as
 * the garmin branch of MeshCore (BLE_GARMIN) and Meck's meck_max_ble_garmin
 * build do:
 *   - pairing without a PIN: the watch can only reach an encrypted link with
 *     no passkey, so the characteristics need encryption only. Phones also
 *     pair without a PIN while this is on.
 *   - small-MTU slicing: the watch never raises the ATT MTU above 23, so
 *     frames to it travel as [tag][len][payload] slices of up to 18 bytes,
 *     and sliced writes from it are put back together (tags F0 to F3).
 *   - heart-rate sensor advertising: the watch's Connect IQ app never starts
 *     pairing with a node it does not know, but its Sensors & Accessories
 *     menu does. The advert shows the Heart Rate service, the UART service
 *     goes in the scan response, and every notification switch (CCCD) needs
 *     an encrypted link, so that menu pairs before it can subscribe. No
 *     heart-rate data is sent.
 *   - a relaxed connection interval for a peer that keeps the 23-byte MTU
 *     (the watch) and the fast one for any larger-MTU peer (a phone).
 *   - no PHY or packet-length change requests (none are made in any build;
 *     on the T-Deck Max the 2M PHY request crashed the controller with the
 *     watch).
 */

#pragma once

#include "meck.h"
#include <helpers/BaseSerialInterface.h>

#if MECK_BLE_ENABLED

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

class SerialNimBLEInterface : public BaseSerialInterface {
public:
    SerialNimBLEInterface();

    // Device name as advertised (the caller adds the "MeshCore-" prefix) and
    // the 6-digit pairing PIN. Allocates the PSRAM frame buffers. Does not
    // start Bluetooth; the first enable() does.
    void begin(const char* dev_name, uint32_t pin);
    void setPin(uint32_t pin) { _pin = pin; }

    // BaseSerialInterface
    void enable() override;
    void disable() override;
    bool isEnabled() const override { return _enabled; }
    bool isConnected() const override { return _paired_conn; }
    bool isWriteBusy() const override;
    size_t writeFrame(const uint8_t src[], size_t len) override;
    size_t checkRecvFrame(uint8_t dest[]) override;

    // Called from the NimBLE host task (static C callbacks forward here).
    void onSync();
    void onReset(int reason);
    int  onGapEvent(void* event);                 // struct ble_gap_event*
    void onRxWrite(const uint8_t* data, int len);

private:
    struct Frame {
        uint8_t len;
        uint8_t buf[MAX_FRAME_SIZE];
    };
    static constexpr int FRAME_QUEUE_SIZE = 16;

    bool startNimBLE();
    void startAdvertising();
    void onLinkSecured(uint16_t conn_handle);
    void requestInterval(uint16_t conn_handle, bool fast);

    char     _dev_name[32];          // fits NimBLE's 31-byte GAP name limit
    uint32_t _pin;

    bool     _begun;                 // begin() has run (buffers allocated)
    bool     _nimble_started;        // nimble_port_init() + host task done
    volatile bool _enabled;
    volatile bool _synced;           // host and C6 in sync, address known
    uint8_t  _own_addr_type;

    volatile uint16_t _conn_handle;  // BLE_HS_CONN_HANDLE_NONE when idle
    volatile bool _paired_conn;      // connected AND encrypted + authenticated
    volatile bool _tx_subscribed;    // phone has notifications on for TX
    volatile uint32_t _adv_restart_at;   // millis() when to restart advertising, 0 = none

    QueueHandle_t _recv_queue;       // host task -> meck_task, storage in PSRAM
    Frame*   _send_queue;            // meck_task only, PSRAM
    int      _send_queue_len;
    uint32_t _last_write;

#if MECK_BLE_GARMIN
    uint16_t _slice_tx_off;          // meck_task: bytes of _send_queue[0] already sent as slices
    uint8_t  _slice_rx_buf[MAX_FRAME_SIZE];   // host task: reassembly of sliced writes
    int      _slice_rx_len;          // host task: bytes collected (-1 = not mid-frame)
#endif
};

#endif // MECK_BLE_ENABLED
