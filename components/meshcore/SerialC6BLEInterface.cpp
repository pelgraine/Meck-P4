/*
 * SerialC6BLEInterface.cpp -- BLE HID keyboard host via ESP32-C6 AT over SDIO
 *
 * See SerialC6BLEInterface.h for the design overview and threading model.
 *
 * What is solid here and what needs on-hardware validation
 * --------------------------------------------------------
 * Solid (verified against the ESP-AT v4.1 C6 slave binary in firmware/ and
 * mirrors the working WiFi/old-BLE poller):
 *   - SDIO poll / line parser / AT command plumbing.
 *   - AT+BLEINIT=1 client init, AT+BLESCAN, AT+BLECONN, AT+BLEGATTCPRIMSRV /
 *     AT+BLEGATTCCHAR discovery, AT+BLEGATTCWR, +NOTIFY decoding.
 *   - HID boot-protocol report decode and key-down edge detection.
 *
 * Needs validation on a real keyboard (commented inline where they live):
 *   1. The pairing/encryption handshake. Keyboards vary between Just Works
 *      and passkey-entry. The flow below uses IO-cap = NoInputNoOutput and
 *      initiates encryption after connect; passkey-entry keyboards will need
 *      the +BLEKEYREQ / AT+BLEKEYREPLY path added.
 *   2. The exact srv/char/desc indices ESP-AT returns. We match by UUID
 *      rather than hard-coding indices; the CCCD-vs-value distinction in
 *      AT+BLEGATTCWR (desc field present or absent) is the fiddly part.
 *
 * Nothing here was run; it is a first implementation to validate.
 */

#include "SerialC6BLEInterface.h"
#include "esp_at.h"            // Cpp_Bus_Driver::Esp_At
#include <Arduino.h>          // millis() (arduino_cpp_bus_driver layer; arduino_compat.h
                              // documents that this layer provides it). The old companion
                              // got it transitively via BaseSerialInterface.h, which this
                              // central no longer inherits.
#include <cstdio>
#include <cstring>
#include <cstdlib>

#define C6KBD_DEBUG 1
#if C6KBD_DEBUG
  #define C6KBD_LOG(fmt, ...) printf("C6KBD: " fmt "\n", ##__VA_ARGS__)
#else
  #define C6KBD_LOG(...) do {} while(0)
#endif

// ---------------------------------------------------------------------------
// Construction
// ---------------------------------------------------------------------------

C6BleKeyboard::C6BleKeyboard()
    : _at(nullptr), _begun(false), _enabled(false), _state(State::OFF),
      _conn_index(0), _want_connect(false), _step_done(false), _conn_addr_type(0),
      _hid_srv_index(-1), _proto_char_index(-1),
      _input_char_index(-1), _input_cccd_index(-1),
      _report_input_count(0), _last_char_is_report(false), _last_report_char_idx(-1),
      _state_deadline(0), _rescan_at(0),
      _rx_len(0), _got_ok(false), _got_error(false), _got_prompt(false),
      _key_head(0), _key_tail(0), _scan_count(0)
{
    _target_addr[0] = '\0';
    _paired_addr[0] = '\0';
    _connect_addr[0] = '\0';
    memset(_prev_keys, 0, sizeof(_prev_keys));
    memset(_rx_buf, 0, sizeof(_rx_buf));
    memset(_scan, 0, sizeof(_scan));
}

void C6BleKeyboard::begin(Cpp_Bus_Driver::Esp_At* at) {
    _at = at;
    _begun = true;
    C6KBD_LOG("begin");
}

// True only for a well-formed BLE MAC string "xx:xx:xx:xx:xx:xx". Guards
// against garbage/uninitialised prefs (e.g. an old NVS blob whose bytes land
// in the appended kbd_addr field): a junk target must never reach AT+BLECONN.
static bool is_hex_digit(char c) {
    return (c >= '0' && c <= '9') || (c >= 'a' && c <= 'f') || (c >= 'A' && c <= 'F');
}
static bool is_valid_ble_addr(const char* a) {
    if (!a || strlen(a) != 17) return false;
    for (int i = 0; i < 17; i++) {
        if (i % 3 == 2) { if (a[i] != ':') return false; }
        else if (!is_hex_digit(a[i]))      return false;
    }
    return true;
}

void C6BleKeyboard::setTargetAddr(const char* addr) {
    // Reject anything that isn't a real MAC; an invalid target becomes "none"
    // so enable() sits IDLE and waits for the picker rather than trying to
    // reconnect to a device that was never paired.
    if (!is_valid_ble_addr(addr)) { _target_addr[0] = '\0'; return; }
    strncpy(_target_addr, addr, sizeof(_target_addr) - 1);
    _target_addr[sizeof(_target_addr) - 1] = '\0';
}

// ---------------------------------------------------------------------------
// SDIO + AT command plumbing
// ---------------------------------------------------------------------------

bool C6BleKeyboard::pollSDIO() {
    if (!_at) return false;
    uint32_t flag = _at->get_irq_flag();
    if (!_at->assert_rx_new_packet_flag(flag)) return false;
    _at->clear_irq_flag(flag);

    std::vector<uint8_t> buf;
    if (!_at->receive_packet(buf)) return false;
    if (buf.empty()) return false;

    int space = RX_BUF_SIZE - _rx_len - 1;
    int copy = (int)buf.size() < space ? (int)buf.size() : space;
    if (copy > 0) {
        memcpy(&_rx_buf[_rx_len], buf.data(), copy);
        _rx_len += copy;
        _rx_buf[_rx_len] = '\0';
    }
    return true;
}

void C6BleKeyboard::drainLines() {
    while (true) {
        char* nl = (char*)memchr(_rx_buf, '\n', _rx_len);
        if (!nl) break;
        int line_len = (int)(nl - _rx_buf);
        int end = line_len;
        if (end > 0 && _rx_buf[end - 1] == '\r') end--;
        _rx_buf[end] = '\0';
        if (end > 0) processLine(_rx_buf, end);
        int consumed = line_len + 1;
        int remaining = _rx_len - consumed;
        if (remaining > 0) memmove(_rx_buf, _rx_buf + consumed, remaining);
        _rx_len = remaining;
        _rx_buf[_rx_len] = '\0';
    }
    if (_rx_len > 0) {
        for (int i = 0; i < _rx_len; i++) {
            if (_rx_buf[i] == '>') {
                _got_prompt = true;
                int after = _rx_len - (i + 1);
                if (after > 0) memmove(_rx_buf, _rx_buf + i + 1, after);
                _rx_len = after;
                _rx_buf[_rx_len] = '\0';
                break;
            }
        }
    }
}

bool C6BleKeyboard::atCmd(const char* cmd, int timeout_ms) {
    if (!_at) return false;
    _got_ok = false; _got_error = false;
    char buf[160];
    int n = snprintf(buf, sizeof(buf), "%s\r\n", cmd);
    _at->send_packet(buf, n);
    unsigned long deadline = millis() + timeout_ms;
    while (!_got_ok && !_got_error && (long)(millis() - deadline) < 0) {
        pollSDIO(); drainLines(); vTaskDelay(pdMS_TO_TICKS(5));
    }
    if (!_got_ok && !_got_error) C6KBD_LOG("atCmd timeout: %s", cmd);
    return _got_ok;
}

bool C6BleKeyboard::gattcWrite(int srv_index, int char_index, int desc_index,
                               const uint8_t* data, size_t len, int timeout_ms) {
    if (!_at) return false;
    _got_ok = false; _got_error = false; _got_prompt = false;
    char cmd[96];
    if (desc_index >= 0) {
        snprintf(cmd, sizeof(cmd), "AT+BLEGATTCWR=%d,%d,%d,%d,%d\r\n",
                 _conn_index, srv_index, char_index, desc_index, (int)len);
    } else {
        snprintf(cmd, sizeof(cmd), "AT+BLEGATTCWR=%d,%d,%d,,%d\r\n",
                 _conn_index, srv_index, char_index, (int)len);
    }
    _at->send_packet(cmd, strlen(cmd));
    unsigned long deadline = millis() + timeout_ms;
    while (!_got_prompt && !_got_error && (long)(millis() - deadline) < 0) {
        pollSDIO(); drainLines(); vTaskDelay(pdMS_TO_TICKS(2));
    }
    if (!_got_prompt) { C6KBD_LOG("gattcWrite: no prompt"); return false; }
    _at->send_packet((const char*)data, len);
    while (!_got_ok && !_got_error && (long)(millis() - deadline) < 0) {
        pollSDIO(); drainLines(); vTaskDelay(pdMS_TO_TICKS(2));
    }
    return _got_ok;
}

// ---------------------------------------------------------------------------
// Scan helpers + browse
// ---------------------------------------------------------------------------

bool C6BleKeyboard::startScanRaw() { return atCmd("AT+BLESCAN=1"); }
bool C6BleKeyboard::stopScanRaw()  { return atCmd("AT+BLESCAN=0", 1000); }

void C6BleKeyboard::addScanDev(const char* addr, const char* name, int8_t rssi, uint8_t addr_type) {
    // Dedup by address; update rssi/name if seen again.
    for (int i = 0; i < _scan_count; i++) {
        if (strcasecmp(_scan[i].addr, addr) == 0) {
            _scan[i].rssi = rssi;
            _scan[i].addr_type = addr_type;
            if (name && name[0] && _scan[i].name[0] == '\0') {
                strncpy(_scan[i].name, name, sizeof(_scan[i].name) - 1);
            }
            return;
        }
    }
    if (_scan_count >= MAX_SCAN) return;
    int i = _scan_count;                       // write the entry fully...
    strncpy(_scan[i].addr, addr, sizeof(_scan[i].addr) - 1);
    _scan[i].addr[sizeof(_scan[i].addr) - 1] = '\0';
    _scan[i].name[0] = '\0';
    if (name) { strncpy(_scan[i].name, name, sizeof(_scan[i].name) - 1);
                _scan[i].name[sizeof(_scan[i].name) - 1] = '\0'; }
    _scan[i].rssi = rssi;
    _scan[i].addr_type = addr_type;
    _scan_count = i + 1;                        // ...then publish the count
}

bool C6BleKeyboard::getScan(int i, ScanDev* out) const {
    if (!out || i < 0 || i >= _scan_count) return false;
    *out = _scan[i];
    return true;
}

void C6BleKeyboard::startBrowse() {
    if (!_enabled) { C6KBD_LOG("startBrowse: not enabled"); return; }
    _scan_count = 0;
    _state = State::BROWSING;
    _state_deadline = 0;       // browse has no auto-timeout; UI stops it
    startScanRaw();
}

void C6BleKeyboard::stopBrowse() {
    if (_state != State::BROWSING) return;
    stopScanRaw();
    _state = (_paired_addr[0] && isConnected()) ? State::READY : State::IDLE;
}

void C6BleKeyboard::connectTo(const char* addr) {
    if (!is_valid_ble_addr(addr)) { C6KBD_LOG("connectTo: invalid addr"); return; }
    if (_state == State::BROWSING) stopScanRaw();
    // Use the address type the scan reported for this device (keyboards are
    // usually random); default to random if it isn't in the scan list.
    _conn_addr_type = 1;
    for (int i = 0; i < _scan_count; i++) {
        if (strcasecmp(_scan[i].addr, addr) == 0) { _conn_addr_type = _scan[i].addr_type; break; }
    }
    setTargetAddr(addr);
    connectToInternal(addr);
}

void C6BleKeyboard::connectToInternal(const char* addr) {
    strncpy(_paired_addr, addr, sizeof(_paired_addr) - 1);
    _paired_addr[sizeof(_paired_addr) - 1] = '\0';
    _state = State::CONNECTING;
    _state_deadline = millis() + STEP_TIMEOUT_MS;
    // AT+BLECONN=<conn_index>,<addr>,<addr_type>. Without the type the C6
    // assumes public; most BLE keyboards advertise a random address, so a
    // public-typed connect just times out (+BLECONN:0,-1).
    char cmd[56];
    snprintf(cmd, sizeof(cmd), "AT+BLECONN=0,\"%s\",%u", addr, (unsigned)_conn_addr_type);
    if (!atCmd(cmd, 6000)) {
        C6KBD_LOG("connect cmd rejected");
        resetLink();
    }
    // Success continues from the +BLECONN: unsolicited event.
}

// ---------------------------------------------------------------------------
// Unsolicited line handling + state machine
// ---------------------------------------------------------------------------

void C6BleKeyboard::processLine(const char* line, int len) {
    if (strcmp(line, "OK") == 0)    { _got_ok = true; return; }
    if (strcmp(line, "ERROR") == 0 || strncmp(line, "ERR CODE:", 9) == 0) {
        _got_error = true; C6KBD_LOG("AT error: %s", line); return;
    }
    // ESP-AT emits "busy p..." when a previous command is still processing.
    // Treat it as a soft failure so atCmd() returns immediately instead of
    // spinning out its full timeout (which blocks meck_task on the bus).
    if (strncmp(line, "busy", 4) == 0) { _got_error = true; C6KBD_LOG("C6 busy"); return; }

    if (strncmp(line, "+BLESCAN:", 9) == 0) { onScanResult(line); return; }

    if (strncmp(line, "+BLECONN:", 9) == 0) {
        const char* p = line + 9;
        _conn_index = strtol(p, (char**)&p, 10);
        if (*p == ',') p++;
        // Success form is +BLECONN:<idx>,"aa:bb:..."; the failure form is
        // +BLECONN:<idx>,-1 (no address). Anything that isn't a quoted address
        // means the connection did not establish.
        if (*p != '"') {
            C6KBD_LOG("connect failed: %s", line);
            resetLink();
            return;
        }
        C6KBD_LOG("connected: %s", line);
        // Link up; hand off to poll() to start security (HID requires an
        // encrypted link). poll() sends AT+BLEENC once on entering SECURING,
        // then the passkey arrives via +BLESECNTFYKEY (handled below).
        _state = State::SECURING;
        _step_done = false;
        _state_deadline = millis() + STEP_TIMEOUT_MS;
        return;
    }

    if (strncmp(line, "+BLESECREQ:", 11) == 0) {
        // Peripheral asked us to start security. If we haven't begun yet, drop
        // into SECURING so poll() issues AT+BLEENC; if already securing, ignore.
        if (_state == State::CONNECTING) { _state = State::SECURING; _step_done = false; }
        return;
    }

    if (strncmp(line, "+BLESECNTFYKEY:", 15) == 0) {
        // +BLESECNTFYKEY:<conn>,<6-digit passkey> -- the host (us) must show
        // this; the user types it on the keyboard and presses Enter. Extend the
        // SECURING deadline so the per-step watchdog does not reset the link
        // while the user is typing. (On-screen display is a follow-up; for now
        // the code is logged so it can be entered from the serial monitor.)
        const char* p = line + 15;
        (void)strtol(p, (char**)&p, 10);
        long key = (*p == ',') ? strtol(p + 1, nullptr, 10) : -1;
        C6KBD_LOG("PASSKEY: type %06ld then Enter on the keyboard", key);
        _state_deadline = millis() + PASSKEY_TIMEOUT_MS;
        return;
    }

    if (strncmp(line, "+BLESECKEYREQ:", 14) == 0) {
        // The stack is asking US to input a passkey (peer is DisplayOnly). A
        // keyboard has no display, so this is not the expected path with our
        // DisplayOnly capability; log it so we can see if a keyboard ever does.
        C6KBD_LOG("passkey input requested (unexpected for a keyboard): %s", line);
        return;
    }

    if (strncmp(line, "+BLESECNCREQ:", 13) == 0) {
        // Numeric-comparison request (both sides DisplayYesNo). Not expected
        // with DisplayOnly; log it rather than silently dropping into [C6].
        C6KBD_LOG("numeric-compare requested (unexpected): %s", line);
        return;
    }

    if (strncmp(line, "+BLEAUTHCMPL:", 13) == 0) {
        const char* p = line + 13;
        (void)strtol(p, (char**)&p, 10);
        int result = (*p == ',') ? strtol(p + 1, nullptr, 10) : -1;
        if (result == 0) {
            C6KBD_LOG("paired OK");
            _state = State::DISCOVERING;   // poll() runs discovery at top level
            _step_done = false;
            _state_deadline = millis() + STEP_TIMEOUT_MS;
        } else {
            C6KBD_LOG("pairing failed (%d)", result);
            resetLink();
        }
        return;
    }

    if (strncmp(line, "+BLEGATTCPRIMSRV:", 17) == 0) { onPrimSrv(line); return; }
    if (strncmp(line, "+BLEGATTCCHAR:",    14) == 0) { onChar(line);    return; }
    if (strncmp(line, "+NOTIFY:",          8)  == 0) { onNotify(line, len); return; }

    if (strncmp(line, "+BLEDISCONN:", 12) == 0) {
        C6KBD_LOG("disconnected: %s", line);
        resetLink();
        return;
    }

    C6KBD_LOG("[C6] %s", line);
}

void C6BleKeyboard::onScanResult(const char* line) {
    // Firmware format (verified against the C6 AT v4.1 binary):
    //   +BLESCAN:"aa:bb:cc:dd:ee:ff",<rssi>,<adv_data>,...
    // The address is QUOTED, so skip the surrounding quotes; reading up to the
    // first comma (the old behaviour) captured the opening quote and truncated
    // the MAC, which is why scanned addresses came out malformed.
    const char* p = line + 9;
    if (*p == '"') p++;                       // opening quote
    char addr[18] = {0};
    int i = 0;
    while (*p && *p != '"' && *p != ',' && i < 17) addr[i++] = *p++;
    addr[i] = '\0';
    if (*p == '"') p++;                        // closing quote
    if (*p == ',') p++;
    int rssi = strtol(p, (char**)&p, 10);

    // The HID service UUID 0x1812 / keyboard appearance 0x03C1 appears in the
    // adv or scan-response hex. Cheap substring scan over the rest of the line.
    const char* rest = p;
    bool is_hid = (strstr(rest, "1218") || strstr(rest, "1812") ||
                   strstr(rest, "C103") || strstr(rest, "03C1"));

    // Address type is the final comma-separated field of a +BLESCAN line
    // (0 = public, 1 = random). Parsed as the last field rather than a fixed
    // column so the intermediate adv/scan-rsp fields don't have to be counted.
    // BLE keyboards are almost always random, so default to 1 if it's missing.
    uint8_t addr_type = 1;
    const char* last_comma = strrchr(line, ',');
    if (last_comma && last_comma[1]) {
        int t = strtol(last_comma + 1, nullptr, 10);
        if (t == 0 || t == 1) addr_type = (uint8_t)t;
    }

    if (_state == State::BROWSING) {
        if (is_hid) addScanDev(addr, "", (int8_t)rssi, addr_type);   // name parsing TODO
        return;
    }

    // Reconnect path: only match the known target. Hand the actual connect to
    // poll() (stopping the scan + AT+BLECONN there) so no AT command runs from
    // inside the parser.
    if (_state == State::SCANNING && _target_addr[0] &&
        strcasecmp(addr, _target_addr) == 0) {
        strncpy(_connect_addr, addr, sizeof(_connect_addr) - 1);
        _connect_addr[sizeof(_connect_addr) - 1] = '\0';
        _conn_addr_type = addr_type;
        _want_connect = true;
    }
}

void C6BleKeyboard::beginDiscovery() {
    _hid_srv_index = _proto_char_index = _input_char_index = _input_cccd_index = -1;
    _report_input_count = 0;
    _last_char_is_report = false;
    _last_report_char_idx = -1;
    _state = State::DISCOVERING;
    _state_deadline = millis() + STEP_TIMEOUT_MS;

    char cmd[24];
    snprintf(cmd, sizeof(cmd), "AT+BLEGATTCPRIMSRV=%d", _conn_index);
    atCmd(cmd, 4000);
    if (_hid_srv_index < 0) { C6KBD_LOG("no HID service (0x1812)"); resetLink(); return; }

    snprintf(cmd, sizeof(cmd), "AT+BLEGATTCCHAR=%d,%d", _conn_index, _hid_srv_index);
    atCmd(cmd, 4000);
    finishSubscribe();
}

void C6BleKeyboard::onPrimSrv(const char* line) {
    // +BLEGATTCPRIMSRV:<conn>,<srv_index>,<srv_uuid>,<srv_type>
    const char* p = line + 17;
    (void)strtol(p, (char**)&p, 10); if (*p == ',') p++;
    int srv_index = strtol(p, (char**)&p, 10); if (*p == ',') p++;
    char uuid[16] = {0}; int j = 0;
    while (*p && *p != ',' && j < 15) uuid[j++] = *p++;
    uuid[j] = '\0';
    if (strstr(uuid, "1812")) { _hid_srv_index = srv_index;
                                C6KBD_LOG("HID service at srv_index %d", srv_index); }
}

void C6BleKeyboard::onChar(const char* line) {
    //  +BLEGATTCCHAR:"char",<conn>,<srv>,<char_index>,<uuid>,<prop>
    //  +BLEGATTCCHAR:"desc",<conn>,<srv>,<char_index>,<desc_index>,<uuid>
    const char* p = line + 14;
    bool is_char = (strncmp(p, "\"char\"", 6) == 0);
    bool is_desc = (strncmp(p, "\"desc\"", 6) == 0);
    p = strchr(p, ','); if (!p) return; p++;

    (void)strtol(p, (char**)&p, 10); if (*p == ',') p++;   // conn
    (void)strtol(p, (char**)&p, 10); if (*p == ',') p++;   // srv
    int char_index = strtol(p, (char**)&p, 10); if (*p == ',') p++;

    if (is_char) {
        char uuid[16] = {0}; int j = 0;
        while (*p && *p != ',' && j < 15) uuid[j++] = *p++;
        uuid[j] = '\0';
        C6KBD_LOG("  char idx=%d uuid=%s", char_index, uuid);
        bool is_report = (strstr(uuid, "2A4D") != nullptr);
        _last_char_is_report = is_report;
        if (is_report) _last_report_char_idx = char_index;
        if      (strstr(uuid, "2A4E")) _proto_char_index = char_index;          // Protocol Mode
        else if (strstr(uuid, "2A22")) _input_char_index = char_index;          // Boot kbd input
        else if (is_report && _input_char_index < 0)
                                        _input_char_index = char_index;          // Report (fallback)
    } else if (is_desc) {
        int desc_index = strtol(p, (char**)&p, 10); if (*p == ',') p++;
        char uuid[16] = {0}; int j = 0;
        while (*p && *p != ',' && j < 15) uuid[j++] = *p++;
        uuid[j] = '\0';
        C6KBD_LOG("    desc char_idx=%d desc_idx=%d uuid=%s", char_index, desc_index, uuid);
        if (strstr(uuid, "2902")) {
            // Collect the CCCD of every Report characteristic for the probe.
            if (_last_char_is_report && char_index == _last_report_char_idx &&
                _report_input_count < MAX_REPORTS) {
                _report_inputs[_report_input_count].char_idx = char_index;
                _report_inputs[_report_input_count].cccd_idx = desc_index;
                _report_input_count++;
            }
            if (char_index == _input_char_index) _input_cccd_index = desc_index;
        }
    }
}

void C6BleKeyboard::finishSubscribe() {
    _state = State::SUBSCRIBING;
    _state_deadline = millis() + STEP_TIMEOUT_MS;

    // Probe: drive Report Protocol (1), not Boot (0) — the MX Keys reports
    // through its Report (0x2A4D) characteristics, and the boot input stayed
    // silent. Subscribe the CCCD of every Report characteristic and let a real
    // keypress reveal (via the raw +NOTIFY log) which one carries the keys.
    if (_proto_char_index >= 0) {
        uint8_t mode = 0x01;   // 1 = Report Protocol
        gattcWrite(_hid_srv_index, _proto_char_index, -1, &mode, 1);
    }

    int subscribed = 0;
    uint8_t cccd[2] = { 0x01, 0x00 };
    for (int i = 0; i < _report_input_count; i++) {
        if (gattcWrite(_hid_srv_index, _report_inputs[i].char_idx,
                       _report_inputs[i].cccd_idx, cccd, 2)) {
            subscribed++;
            C6KBD_LOG("subscribed report char=%d cccd=%d",
                      _report_inputs[i].char_idx, _report_inputs[i].cccd_idx);
        } else {
            C6KBD_LOG("subscribe failed char=%d cccd=%d",
                      _report_inputs[i].char_idx, _report_inputs[i].cccd_idx);
        }
    }

    if (subscribed == 0) {
        C6KBD_LOG("no report inputs subscribed (count=%d)", _report_input_count);
        resetLink();
        return;
    }
    _state = State::READY;
    memset(_prev_keys, 0, sizeof(_prev_keys));
    C6KBD_LOG("keyboard READY (%s) — %d report input(s)", _paired_addr, subscribed);
}

void C6BleKeyboard::resetLink() {
    _hid_srv_index = _proto_char_index = _input_char_index = _input_cccd_index = -1;
    memset(_prev_keys, 0, sizeof(_prev_keys));
    _want_connect = false;
    _step_done = false;
    _rescan_at = 0;
    // Auto-reconnect (here and in enable()) is parked while pairing is being
    // validated: retrying a failed/absent connect just re-wedges the C6. Drop
    // to IDLE and let the user reconnect via the picker.
    _state = _enabled ? State::IDLE : State::OFF;
}

// ---------------------------------------------------------------------------
// HID boot-keyboard report decode
// ---------------------------------------------------------------------------

static char hid_usage_to_ascii(uint8_t usage, bool shift) {
    if (usage >= 0x04 && usage <= 0x1D) {           // a..z
        char c = 'a' + (usage - 0x04);
        return shift ? (char)(c - 32) : c;
    }
    static const char num[10]    = {'1','2','3','4','5','6','7','8','9','0'};
    static const char num_sh[10] = {'!','@','#','$','%','^','&','*','(',')'};
    if (usage >= 0x1E && usage <= 0x27) {           // number row
        int i = usage - 0x1E;
        return shift ? num_sh[i] : num[i];
    }
    switch (usage) {
        case 0x2C: return ' ';
        case 0x2D: return shift ? '_' : '-';
        case 0x2E: return shift ? '+' : '=';
        case 0x2F: return shift ? '{' : '[';
        case 0x30: return shift ? '}' : ']';
        case 0x31: return shift ? '|' : '\\';
        case 0x33: return shift ? ':' : ';';
        case 0x34: return shift ? '"' : '\'';
        case 0x35: return shift ? '~' : '`';
        case 0x36: return shift ? '<' : ',';
        case 0x37: return shift ? '>' : '.';
        case 0x38: return shift ? '?' : '/';
        default:   return 0;
    }
}

static uint32_t hid_usage_to_lvkey(uint8_t usage) {
    switch (usage) {
        case 0x28: return LV_KEY_ENTER;
        case 0x29: return LV_KEY_ESC;
        case 0x2A: return LV_KEY_BACKSPACE;
        case 0x2B: return LV_KEY_NEXT;       // Tab
        case 0x4F: return LV_KEY_RIGHT;
        case 0x50: return LV_KEY_LEFT;
        case 0x51: return LV_KEY_DOWN;
        case 0x52: return LV_KEY_UP;
        default:   return 0;
    }
}

void C6BleKeyboard::onNotify(const char* line, int len) {
    if (_state != State::READY) return;
    // +NOTIFY:<conn>,<srv>,<char>,<len>,<data...>
    const char* p = line + 8;
    strtol(p, (char**)&p, 10); if (*p == ',') p++;            // conn
    strtol(p, (char**)&p, 10); if (*p == ',') p++;            // srv
    long chr  = strtol(p, (char**)&p, 10); if (*p == ',') p++; // char index
    long dlen = strtol(p, (char**)&p, 10); if (*p == ',') p++; // declared payload length

    int payload_off = (int)(p - line);
    int avail = len - payload_off;
    if (avail < 0) avail = 0;

    // Hex-dump the report so the true byte layout is visible. drainLines()
    // splits on 0x0A, so a payload containing 0x0A/0x0D is truncated and
    // 'avail' will read short of 'declared' -- that mismatch flags the
    // line-splitting problem we still need to fix for binary payloads.
    char hex[3 * 24 + 1]; int hn = 0;
    int dump = (avail > 24) ? 24 : avail;
    for (int i = 0; i < dump && hn < (int)sizeof(hex) - 3; i++)
        hn += snprintf(hex + hn, sizeof(hex) - hn, "%02x ", (unsigned char)p[i]);
    hex[hn] = '\0';
    C6KBD_LOG("NOTIFY char=%ld declared=%ld avail=%d : %s", chr, dlen, avail, hex);

    decodeBootReport((const uint8_t*)p, avail);
}

void C6BleKeyboard::decodeBootReport(const uint8_t* rpt, int len) {
    if (len >= 9) { rpt++; len--; }   // skip a Report ID byte if present
    if (len < 8) return;

    uint8_t mods = rpt[0];
    bool shift = (mods & 0x22) != 0;          // L/R shift
    const uint8_t* keys = &rpt[2];

    for (int i = 0; i < 6; i++) {
        uint8_t u = keys[i];
        if (u == 0 || u == 0x01) continue;

        bool was_down = false;
        for (int j = 0; j < 6; j++) if (_prev_keys[j] == u) { was_down = true; break; }
        if (was_down) continue;

        uint32_t lvk = hid_usage_to_lvkey(u);
        if (lvk) { pushKey(lvk); continue; }
        char c = hid_usage_to_ascii(u, shift);
        if (c) pushKey((uint32_t)(unsigned char)c);
    }
    memcpy(_prev_keys, keys, 6);
}

// ---------------------------------------------------------------------------
// SPSC key ring
// ---------------------------------------------------------------------------

void C6BleKeyboard::pushKey(uint32_t k) {
    uint16_t head = _key_head;
    uint16_t next = (uint16_t)((head + 1) % KEY_FIFO);
    if (next == _key_tail) return;
    _key_ring[head] = k;
    _key_head = next;
}

uint32_t C6BleKeyboard::read_key() {
    uint16_t tail = _key_tail;
    if (tail == _key_head) return 0;
    uint32_t k = _key_ring[tail];
    _key_tail = (uint16_t)((tail + 1) % KEY_FIFO);
    return k;
}

// ---------------------------------------------------------------------------
// enable / disable / poll
// ---------------------------------------------------------------------------

void C6BleKeyboard::enable() {
    if (_enabled) return;
    if (!_begun || !_at) { C6KBD_LOG("enable: not ready"); return; }

    _rx_len = 0;
    _key_head = 0;
    _key_tail = 0;
    _scan_count = 0;
    _want_connect = false;
    _step_done = false;

    if (!atCmd("AT+BLEINIT=1")) { C6KBD_LOG("BLEINIT=1 failed"); return; }

    // AT+BLESECPARAM=<auth_req>,<iocap>,<key_size>,<init_key>,<resp_key>.
    // iocap=0 (DisplayOnly) so we show the passkey. auth_req=7 (SC+MITM+bond)
    // was rejected by this build, and the accepted set varies by build, so
    // probe every value, log which are accepted, and keep the strongest
    // MITM-capable one active. The keyboard needs an authenticated link to
    // notify, so an accepted SC/MITM value (7/6/3/2) is what we want.
    // Preference order, strongest first: 7,6,3,2 (MITM), then 5,4,1,0.
    static const uint8_t pref[] = {7, 6, 3, 2, 5, 4, 1, 0};
    int chosen = -1;
    for (int i = 0; i < 8; i++) {
        char cmd[40];
        snprintf(cmd, sizeof(cmd), "AT+BLESECPARAM=%u,0,16,3,3", (unsigned)pref[i]);
        bool ok = atCmd(cmd);
        C6KBD_LOG("auth_req=%u -> %s", (unsigned)pref[i], ok ? "OK" : "ERROR");
        if (ok && chosen < 0) chosen = pref[i];   // first (strongest) accepted
    }
    // The loop left the last-tried value set; re-assert the chosen one so it is
    // the active security config for the upcoming pairing.
    if (chosen >= 0) {
        char cmd[40];
        snprintf(cmd, sizeof(cmd), "AT+BLESECPARAM=%u,0,16,3,3", (unsigned)chosen);
        atCmd(cmd);
        C6KBD_LOG("using auth_req=%d (iocap=0)", chosen);
    } else {
        C6KBD_LOG("no auth_req accepted with iocap=0; check iocap/key params");
    }

    _enabled = true;

    // Auto-reconnect on enable is parked: a stored address that was never
    // actually paired (the picker saved it on tap, before pairing succeeded)
    // makes enable() fire AT+BLECONN at an absent device, which wedges the C6
    // and starves meck_task -> task-watchdog. Until pairing is validated
    // end-to-end and the address is only persisted on a confirmed pair, always
    // sit IDLE and connect only via the picker (connectTo). To restore later,
    // re-enable the connectToInternal(_target_addr) branch.
    _state = State::IDLE;
    C6KBD_LOG("enabled");
}

void C6BleKeyboard::disable() {
    if (!_enabled) return;
    _enabled = false;

    if (_state == State::BROWSING || _state == State::SCANNING) stopScanRaw();
    if (_state == State::CONNECTING || _state == State::SECURING ||
        _state == State::DISCOVERING || _state == State::SUBSCRIBING ||
        _state == State::READY) {
        char cmd[24];
        snprintf(cmd, sizeof(cmd), "AT+BLEDISCONN=%d", _conn_index);
        atCmd(cmd, 1000);
    }
    atCmd("AT+BLEINIT=0", 1000);

    _state = State::OFF;
    _rescan_at = 0;
    _scan_count = 0;
    _want_connect = false;
    _step_done = false;
    _key_head = 0;
    _key_tail = 0;
    C6KBD_LOG("disabled");
}

void C6BleKeyboard::poll() {
    if (!_at || !_enabled) return;

    for (int i = 0; i < 4; i++) if (!pollSDIO()) break;
    drainLines();   // parses events: only updates state + flags, never sends AT

    // ---- Connect/discover steps, issued here at the top level ----------------
    // drainLines() above may have set _want_connect or advanced _state. Acting
    // on those here (rather than inside processLine) keeps every AT command out
    // of the parser's call stack, so atCmd -> drainLines -> processLine can
    // never re-enter atCmd. Each branch fires once per step.
    if (_want_connect) {
        _want_connect = false;
        stopScanRaw();
        connectToInternal(_connect_addr);
    }
    if (_state == State::SECURING && !_step_done) {
        _step_done = true;
        char cmd[32];
        snprintf(cmd, sizeof(cmd), "AT+BLEENC=%d,3", _conn_index);
        atCmd(cmd, 4000);
    }
    if (_state == State::DISCOVERING && !_step_done) {
        _step_done = true;
        beginDiscovery();   // PRIMSRV + CHAR + subscribe; transitions to READY
    }

    if (_rescan_at && (long)(millis() - _rescan_at) >= 0) {
        _rescan_at = 0;
        if (_enabled && _state == State::SCANNING) startScanRaw();
    }

    // Per-step watchdog for the connect/discover chain (not for IDLE/BROWSING).
    bool in_chain = (_state == State::CONNECTING || _state == State::SECURING ||
                     _state == State::DISCOVERING || _state == State::SUBSCRIBING);
    if (in_chain && _state_deadline && (long)(millis() - _state_deadline) >= 0) {
        C6KBD_LOG("step timeout in state %d, resetting", (int)_state);
        resetLink();
    }
}