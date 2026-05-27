/*
 * SerialC6BLEInterface.cpp -- BLE companion transport via ESP32-C6 AT over SDIO
 *
 * See SerialC6BLEInterface.h for design overview.
 */

#include "SerialC6BLEInterface.h"
#include "esp_at.h"            // Cpp_Bus_Driver::Esp_At
#include <cstdio>
#include <cstring>
#include <cstdlib>

#define C6BLE_DEBUG 1
#if C6BLE_DEBUG
  #define C6BLE_LOG(fmt, ...) printf("C6BLE: " fmt "\n", ##__VA_ARGS__)
#else
  #define C6BLE_LOG(...) do {} while(0)
#endif

// ---------------------------------------------------------------------------
// Construction
// ---------------------------------------------------------------------------

SerialC6BLEInterface::SerialC6BLEInterface()
    : _at(nullptr), _begun(false), _enabled(false), _connected(false),
      _adv_restart_time(0), _last_write_ms(0),
      _rx_len(0), _recv_queue_len(0), _send_queue_len(0),
      _got_ok(false), _got_error(false), _got_prompt(false),
      _ble_pin(0), _probe_mode(false)
{
    memset(_dev_name, 0, sizeof(_dev_name));
    memset(_rx_buf, 0, sizeof(_rx_buf));
}

// ---------------------------------------------------------------------------
// begin()
// ---------------------------------------------------------------------------

void SerialC6BLEInterface::begin(Cpp_Bus_Driver::Esp_At* at, const char* dev_name) {
    _at = at;
    strncpy(_dev_name, dev_name, sizeof(_dev_name) - 1);
    _dev_name[sizeof(_dev_name) - 1] = '\0';
    _begun = true;
    C6BLE_LOG("begin: name='%s'", _dev_name);
}

// ---------------------------------------------------------------------------
// AT command helpers
// ---------------------------------------------------------------------------

bool SerialC6BLEInterface::pollSDIO() {
    if (!_at) return false;

    uint32_t flag = _at->get_irq_flag();
    if (!_at->assert_rx_new_packet_flag(flag)) return false;
    _at->clear_irq_flag(flag);

    std::vector<uint8_t> buf;
    if (!_at->receive_packet(buf)) return false;
    if (buf.empty()) return false;

    // Append to line buffer, clamping to avoid overflow
    int space = RX_BUF_SIZE - _rx_len - 1;  // leave room for NUL
    int copy = (int)buf.size() < space ? (int)buf.size() : space;
    if (copy > 0) {
        memcpy(&_rx_buf[_rx_len], buf.data(), copy);
        _rx_len += copy;
        _rx_buf[_rx_len] = '\0';
    }
    return true;
}

void SerialC6BLEInterface::drainLines() {
    // Extract complete lines (terminated by \n) from _rx_buf and dispatch.
    while (true) {
        char* nl = (char*)memchr(_rx_buf, '\n', _rx_len);
        if (!nl) break;

        int line_len = (int)(nl - _rx_buf);
        // Strip trailing \r if present
        int end = line_len;
        if (end > 0 && _rx_buf[end - 1] == '\r') end--;

        _rx_buf[end] = '\0';
        if (end > 0) {
            processLine(_rx_buf, end);
        }

        // Shift remainder forward
        int consumed = line_len + 1;  // include the \n
        int remaining = _rx_len - consumed;
        if (remaining > 0) {
            memmove(_rx_buf, _rx_buf + consumed, remaining);
        }
        _rx_len = remaining;
        _rx_buf[_rx_len] = '\0';
    }

    // Check for '>' prompt (no trailing newline in data-mode prompts)
    if (_rx_len > 0) {
        for (int i = 0; i < _rx_len; i++) {
            if (_rx_buf[i] == '>') {
                _got_prompt = true;
                // Remove the '>' and anything before it
                int after = _rx_len - (i + 1);
                if (after > 0) {
                    memmove(_rx_buf, _rx_buf + i + 1, after);
                }
                _rx_len = after;
                _rx_buf[_rx_len] = '\0';
                break;
            }
        }
    }
}

void SerialC6BLEInterface::processLine(const char* line, int len) {
    // AT response codes
    if (strcmp(line, "OK") == 0) {
        if (_probe_mode) printf("C6_PROBE: << OK\n");
        _got_ok = true;
        return;
    }
    if (strcmp(line, "ERROR") == 0 || strncmp(line, "ERR CODE:", 9) == 0) {
        if (_probe_mode) printf("C6_PROBE: << %s\n", line);
        _got_error = true;
        C6BLE_LOG("AT error: %s", line);
        return;
    }

    // Unsolicited BLE events
    if (strncmp(line, "+BLECONN:", 9) == 0) {
        _connected = true;
        _adv_restart_time = 0;
        C6BLE_LOG("connected: %s", line);
        return;
    }
    if (strncmp(line, "+BLEDISCONN:", 12) == 0) {
        _connected = false;
        clearBuffers();
        if (_enabled) {
            _adv_restart_time = millis() + ADV_RESTART_DELAY_MS;
        }
        C6BLE_LOG("disconnected: %s", line);
        return;
    }

    // +WRITE:<conn>,<srv>,<char>,<desc>,<len>,<data>
    // We match srv=SPP_SRV_INDEX, char=SPP_RX_CHAR (C304).
    // The <data> after the last comma is <len> raw bytes.
    if (strncmp(line, "+WRITE:", 7) == 0) {
        // Parse: +WRITE:conn,srv,char,desc,len,data...
        const char* p = line + 7;
        int conn = 0, srv = 0, chr = 0, desc = 0, data_len = 0;

        // Parse comma-separated integer fields (5 fields before data)
        conn = strtol(p, (char**)&p, 10); if (*p == ',') p++;
        srv  = strtol(p, (char**)&p, 10); if (*p == ',') p++;
        chr  = strtol(p, (char**)&p, 10); if (*p == ',') p++;
        // desc may be empty (,,)
        if (*p == ',') { p++; } else { desc = strtol(p, (char**)&p, 10); if (*p == ',') p++; }
        data_len = strtol(p, (char**)&p, 10); if (*p == ',') p++;

        (void)conn; (void)desc;

        if (srv == SPP_SRV_INDEX && chr == SPP_RX_CHAR && data_len > 0) {
            if (data_len > MAX_FRAME_SIZE) {
                C6BLE_LOG("+WRITE: frame too big (%d)", data_len);
            } else if (_recv_queue_len >= FRAME_QUEUE_SIZE) {
                C6BLE_LOG("+WRITE: recv queue full");
            } else {
                // p now points to the raw data bytes within the line.
                // The line was NUL-terminated by drainLines, and the data
                // occupies the remaining bytes.  Copy exactly data_len bytes.
                int available = len - (int)(p - line);
                int copy = data_len < available ? data_len : available;
                Frame& f = _recv_queue[_recv_queue_len];
                f.len = (uint8_t)copy;
                memcpy(f.buf, p, copy);
                _recv_queue_len++;
                C6BLE_LOG("+WRITE: frame %d bytes", copy);
            }
        }
        return;
    }

    // Log anything else for debugging
    if (len > 0 && line[0] != '\0') {
        if (_probe_mode) {
            printf("C6_PROBE: << %s\n", line);
        }
        C6BLE_LOG("[C6] %s", line);
    }
}

bool SerialC6BLEInterface::atCmd(const char* cmd, int timeout_ms) {
    if (!_at) return false;

    _got_ok = false;
    _got_error = false;

    // Send the command
    char buf[256];
    int n = snprintf(buf, sizeof(buf), "%s\r\n", cmd);
    _at->send_packet(buf, n);

    // Wait for OK or ERROR
    unsigned long deadline = millis() + timeout_ms;
    while (!_got_ok && !_got_error && (long)(millis() - deadline) < 0) {
        pollSDIO();
        drainLines();
        vTaskDelay(pdMS_TO_TICKS(5));
    }

    if (!_got_ok && !_got_error) {
        C6BLE_LOG("atCmd timeout: %s", cmd);
    }
    return _got_ok;
}

bool SerialC6BLEInterface::atCmdWithData(const char* cmd,
                                          const uint8_t* data, size_t data_len,
                                          int timeout_ms) {
    if (!_at) return false;

    _got_ok = false;
    _got_error = false;
    _got_prompt = false;

    // Send the command
    char buf[256];
    int n = snprintf(buf, sizeof(buf), "%s\r\n", cmd);
    _at->send_packet(buf, n);

    // Wait for '>' prompt
    unsigned long deadline = millis() + timeout_ms;
    while (!_got_prompt && !_got_error && (long)(millis() - deadline) < 0) {
        pollSDIO();
        drainLines();
        vTaskDelay(pdMS_TO_TICKS(2));
    }

    if (!_got_prompt) {
        C6BLE_LOG("atCmdWithData: no prompt for: %s", cmd);
        return false;
    }

    // Send the raw data bytes
    _at->send_packet((const char*)data, data_len);

    // Wait for OK
    while (!_got_ok && !_got_error && (long)(millis() - deadline) < 0) {
        pollSDIO();
        drainLines();
        vTaskDelay(pdMS_TO_TICKS(2));
    }

    if (!_got_ok && !_got_error) {
        C6BLE_LOG("atCmdWithData timeout after data: %s", cmd);
    }
    return _got_ok;
}

// ---------------------------------------------------------------------------
// BLE advertising helpers
// ---------------------------------------------------------------------------

void SerialC6BLEInterface::buildScanRspHex(char* hex_out, size_t hex_out_sz) {
    // Build scan response data containing the complete local name.
    // AD structure: [len] [type=0x09 Complete Local Name] [name bytes]
    int name_len = strlen(_dev_name);
    if (name_len > 28) name_len = 28;  // max AD field

    uint8_t raw[32];
    int i = 0;
    raw[i++] = (uint8_t)(name_len + 1);  // AD length (type byte + name)
    raw[i++] = 0x09;                      // AD type: Complete Local Name
    memcpy(&raw[i], _dev_name, name_len);
    i += name_len;

    // Hex-encode for AT+BLESCANRSPDATA
    hex_out[0] = '"';
    int pos = 1;
    for (int j = 0; j < i && pos + 2 < (int)hex_out_sz - 2; j++) {
        pos += snprintf(&hex_out[pos], hex_out_sz - pos, "%02X", raw[j]);
    }
    hex_out[pos++] = '"';
    hex_out[pos] = '\0';
}

bool SerialC6BLEInterface::startAdvertising() {
    // Init BLE as server (peripheral)
    if (!atCmd("AT+BLEINIT=2")) return false;

    // BLE security: static 6-digit PIN pairing.
    // auth_req=1 (bonding), iocap=1 (display only), key_size=16,
    // init_key=3 (LTK+IRK), resp_key=3 (LTK+IRK), auth_option=0.
    // The phone app reads the PIN from CMD_DEVICE_QEURY and initiates
    // pairing with it.
    if (_ble_pin != 0) {
        atCmd("AT+BLESECPARAM=1,1,16,3,3");
        char pin_cmd[32];
        snprintf(pin_cmd, sizeof(pin_cmd), "AT+BLESETKEY=%lu",
                 (unsigned long)_ble_pin);
        atCmd(pin_cmd);
    }

    // Set device name
    char cmd[64];
    snprintf(cmd, sizeof(cmd), "AT+BLENAME=\"%s\"", _dev_name);
    if (!atCmd(cmd)) return false;

    // Advertising parameters:
    //   min_interval=50 (×0.625ms = 31.25ms)
    //   max_interval=100 (×0.625ms = 62.5ms)
    //   type=0 (ADV_IND, connectable undirected)
    //   own_addr_type=0 (public)
    //   channel_map=7 (all three channels)
    //   filter_policy=0 (allow any)
    if (!atCmd("AT+BLEADVPARAM=50,100,0,0,7,0")) return false;

    // Advertising data: flags only (LE General Discoverable + BR/EDR Not Supported)
    if (!atCmd("AT+BLEADVDATA=\"020106\"")) return false;

    // Scan response: complete local name
    char srp_hex[80];
    buildScanRspHex(srp_hex, sizeof(srp_hex));
    snprintf(cmd, sizeof(cmd), "AT+BLESCANRSPDATA=%s", srp_hex);
    if (!atCmd(cmd)) return false;

    // Start advertising
    if (!atCmd("AT+BLEADVSTART")) return false;

    C6BLE_LOG("advertising as '%s'", _dev_name);
    return true;
}

// ---------------------------------------------------------------------------
// BaseSerialInterface: enable / disable
// ---------------------------------------------------------------------------

void SerialC6BLEInterface::enable() {
    if (_enabled) return;
    if (!_begun || !_at) return;

    clearBuffers();
    _connected = false;
    _adv_restart_time = 0;
    _rx_len = 0;

    if (startAdvertising()) {
        _enabled = true;
        C6BLE_LOG("enabled");
    } else {
        C6BLE_LOG("enable FAILED");
    }
}

void SerialC6BLEInterface::disable() {
    if (!_enabled) return;
    _enabled = false;

    atCmd("AT+BLEADVSTOP", 500);
    if (_connected) {
        atCmd("AT+BLEDISCONN=0", 500);
    }
    atCmd("AT+BLEINIT=0", 500);

    _connected = false;
    _adv_restart_time = 0;
    clearBuffers();
    C6BLE_LOG("disabled");
}

// ---------------------------------------------------------------------------
// probeOTA -- diagnostic: check C6 firmware version and OTA capability
// ---------------------------------------------------------------------------

void SerialC6BLEInterface::probeOTA() {
    if (!_begun || !_at) {
        printf("C6_PROBE: not ready (begun=%d, at=%p)\n", _begun, _at);
        return;
    }

    _probe_mode = true;
    _rx_len = 0;  // clear any stale data

    printf("\n========== C6 OTA PROBE ==========\n");

    // 1. Basic connectivity
    printf("C6_PROBE: >> AT\n");
    bool ok = atCmd("AT", 2000);
    printf("C6_PROBE: AT -> %s\n\n", ok ? "OK" : "FAIL");

    // 2. Firmware version
    printf("C6_PROBE: >> AT+GMR\n");
    ok = atCmd("AT+GMR", 3000);
    printf("C6_PROBE: AT+GMR -> %s\n\n", ok ? "OK" : "FAIL");

    // 3. WiFi mode support
    printf("C6_PROBE: >> AT+CWMODE?\n");
    ok = atCmd("AT+CWMODE?", 2000);
    printf("C6_PROBE: AT+CWMODE? -> %s\n\n", ok ? "OK" : "FAIL");

    // 4. Check USEROTA (custom URL OTA)
    printf("C6_PROBE: >> AT+USEROTA?\n");
    ok = atCmd("AT+USEROTA?", 2000);
    printf("C6_PROBE: AT+USEROTA? -> %s\n\n", ok ? "OK (supported)" : "FAIL (not supported)");

    // 5. Check CIUPDATE (Espressif cloud OTA)
    printf("C6_PROBE: >> AT+CIUPDATE?\n");
    ok = atCmd("AT+CIUPDATE?", 2000);
    printf("C6_PROBE: AT+CIUPDATE? -> %s\n\n", ok ? "OK (supported)" : "FAIL (not supported)");

    // 6. Check flash partitions
    printf("C6_PROBE: >> AT+SYSFLASH?\n");
    ok = atCmd("AT+SYSFLASH?", 2000);
    printf("C6_PROBE: AT+SYSFLASH? -> %s\n\n", ok ? "OK" : "FAIL");

    // 7. Check available heap (useful for OTA feasibility)
    printf("C6_PROBE: >> AT+SYSRAM?\n");
    ok = atCmd("AT+SYSRAM?", 2000);
    printf("C6_PROBE: AT+SYSRAM? -> %s\n\n", ok ? "OK" : "FAIL");

    // 8. Check web server (local firmware upload via browser)
    printf("C6_PROBE: >> AT+WEBSERVER=?\n");
    ok = atCmd("AT+WEBSERVER=?", 2000);
    printf("C6_PROBE: AT+WEBSERVER=? -> %s\n\n", ok ? "OK (supported)" : "FAIL (not supported)");

    // 9. Check HTTP client (fetch firmware from local network)
    printf("C6_PROBE: >> AT+HTTPCLIENT=?\n");
    ok = atCmd("AT+HTTPCLIENT=?", 2000);
    printf("C6_PROBE: AT+HTTPCLIENT=? -> %s\n\n", ok ? "OK (supported)" : "FAIL (not supported)");

    // 10. Check if OTA partitions are accessible via SYSFLASH
    printf("C6_PROBE: >> AT+SYSFLASH=0,\"ota_0\",0,16\n");
    ok = atCmd("AT+SYSFLASH=0,\"ota_0\",0,16", 3000);
    printf("C6_PROBE: read ota_0 -> %s\n\n", ok ? "OK (accessible!)" : "FAIL");

    printf("C6_PROBE: >> AT+SYSFLASH=0,\"ota_1\",0,16\n");
    ok = atCmd("AT+SYSFLASH=0,\"ota_1\",0,16", 3000);
    printf("C6_PROBE: read ota_1 -> %s\n\n", ok ? "OK (accessible!)" : "FAIL");

    // 11. Check otadata partition (OTA boot selector)
    printf("C6_PROBE: >> AT+SYSFLASH=0,\"otadata\",0,16\n");
    ok = atCmd("AT+SYSFLASH=0,\"otadata\",0,16", 3000);
    printf("C6_PROBE: read otadata -> %s\n\n", ok ? "OK (accessible!)" : "FAIL");

    // 12. Check CIUPDATE parameter format (can we pass a URL?)
    printf("C6_PROBE: >> AT+CIUPDATE=?\n");
    ok = atCmd("AT+CIUPDATE=?", 2000);
    printf("C6_PROBE: AT+CIUPDATE=? -> %s\n\n", ok ? "OK" : "FAIL");

    // 13. List all available AT commands
    printf("C6_PROBE: >> AT+CMD?\n");
    ok = atCmd("AT+CMD?", 5000);
    printf("C6_PROBE: AT+CMD? -> %s\n\n", ok ? "OK" : "FAIL");

    printf("========== END C6 OTA PROBE ==========\n\n");

    _probe_mode = false;
}

// ---------------------------------------------------------------------------
// BaseSerialInterface: writeFrame / isWriteBusy
// ---------------------------------------------------------------------------

bool SerialC6BLEInterface::isWriteBusy() const {
    return (long)(millis() - _last_write_ms) < WRITE_MIN_INTERVAL_MS;
}

size_t SerialC6BLEInterface::writeFrame(const uint8_t src[], size_t len) {
    if (len == 0 || len > MAX_FRAME_SIZE) return 0;
    if (!_connected) return 0;

    if (_send_queue_len >= FRAME_QUEUE_SIZE) {
        C6BLE_LOG("writeFrame: send queue full");
        return 0;
    }

    Frame& f = _send_queue[_send_queue_len];
    f.len = (uint8_t)len;
    memcpy(f.buf, src, len);
    _send_queue_len++;
    return len;
}

// ---------------------------------------------------------------------------
// BaseSerialInterface: checkRecvFrame  (the main workhorse)
// ---------------------------------------------------------------------------

size_t SerialC6BLEInterface::checkRecvFrame(uint8_t dest[]) {
    if (!_at || !_enabled) return 0;

    // 1. Poll SDIO for incoming C6 data and process events
    //    (multiple polls per call to keep the pipe drained)
    for (int i = 0; i < 4; i++) {
        if (!pollSDIO()) break;
    }
    drainLines();

    // 2. Send one queued frame to the phone via NOTIFY (if connected and not busy)
    if (_connected && _send_queue_len > 0 &&
        (long)(millis() - _last_write_ms) >= WRITE_MIN_INTERVAL_MS) {

        Frame& f = _send_queue[0];

        char cmd[64];
        snprintf(cmd, sizeof(cmd), "AT+BLEGATTSNTFY=0,%d,%d,%d",
                 SPP_SRV_INDEX, SPP_TX_CHAR, (int)f.len);

        if (atCmdWithData(cmd, f.buf, f.len, 500)) {
            _last_write_ms = millis();
        } else {
            C6BLE_LOG("NOTIFY failed, len=%d", (int)f.len);
        }

        // Remove top of queue regardless (don't retry failed frames)
        _send_queue_len--;
        for (int i = 0; i < _send_queue_len; i++) {
            _send_queue[i] = _send_queue[i + 1];
        }
    }

    // 3. Return one received frame to the caller (if any)
    if (_recv_queue_len > 0) {
        size_t len = _recv_queue[0].len;
        memcpy(dest, _recv_queue[0].buf, len);

        _recv_queue_len--;
        for (int i = 0; i < _recv_queue_len; i++) {
            _recv_queue[i] = _recv_queue[i + 1];
        }
        return len;
    }

    // 4. Handle advertising restart after disconnect
    if (_adv_restart_time && (long)(millis() - _adv_restart_time) >= 0) {
        _adv_restart_time = 0;
        if (!_connected) {
            C6BLE_LOG("restarting advertising after disconnect");
            startAdvertising();
        }
    }

    return 0;
}