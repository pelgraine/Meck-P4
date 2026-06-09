/*
 * MeckWebFetch.h -- plain-HTTP page fetch over the ESP32-C6 (Stage 1)
 *
 * Stage 1 of the web reader port. Drives the C6 directly over its own AT
 * command + parser, completely separate from the companion transport in
 * SerialC6WiFiInterface. Given the C6 AT driver and a set of WiFi
 * credentials, it:
 *   1. brings WiFi up on the C6 (CWMODE + CWJAP) using the given credentials,
 *   2. opens a single plain-http (port 80) TCP client connection,
 *   3. sends an HTTP/1.0 GET,
 *   4. reads the raw response and prints it to the debug log,
 *   5. closes the connection and disconnects WiFi.
 *
 * It leaves WiFi off on exit. It does NOT re-enable any companion; the
 * caller is responsible for switching the C6 back to a companion transport
 * (and Stage 1's policy is to leave everything off for manual re-enable).
 *
 * https is intentionally not handled here -- that is a later stage.
 *
 * MUST be called from meck_task: it drives the shared C6 SDIO link, which is
 * not safe to touch from the LVGL/UI task.
 *
 * The C6 bus-driver header is included only in MeckWebFetch.cpp, never here,
 * so this header stays safe to include from meck_app.cpp.
 */

#pragma once

#include "MeckWebHtml.h"             // ParseResult / WebLink / WebForm

namespace Cpp_Bus_Driver { class Esp_At; }

class MeckWebFetch {
public:
    // Fetch a plain-HTTP page and run the reader-mode parser over the body,
    // writing the readable text into textOut and the extracted links/forms
    // into the caller's arrays. The ParseResult (text length, link/form
    // counts) is written to *out. Returns true if any response bytes were
    // received from the server (i.e. the fetch itself succeeded).
    bool fetch(Cpp_Bus_Driver::Esp_At* at,
               const char* ssid, const char* password, const char* url,
               char* textOut, int textCap,
               WebLink* links, int maxLinks,
               WebForm* forms, int maxForms,
               ParseResult* out);

private:
    // C6 AT plumbing (mirrors the subset used by SerialC6WiFiInterface,
    // trimmed for a single-connection client fetch).
    bool poll();                              // pull SDIO bytes into _rx
    void drain();                             // parse _rx byte-by-byte
    void onLine(const char* line, int len);   // handle a complete AT line
    bool cmd(const char* c, int timeout_ms);  // send AT cmd, wait OK/ERROR
    void teardown();                          // CIPCLOSE + CWQAP + free buffer

    Cpp_Bus_Driver::Esp_At* _at = nullptr;

    // Raw SDIO read buffer.
    char _rx[2048];
    int  _rx_len = 0;

    // Line accumulation buffer for text AT responses.
    char _line[512];
    int  _line_len = 0;

    // AT response flags.
    bool _got_ok = false;
    bool _got_error = false;
    bool _got_prompt = false;
    bool _closed = false;   // remote closed the TCP connection

    // Server IP from AT+CIPDOMAIN; empty if DNS resolution failed.
    char _resolved_ip[20] = {0};

    // Passive receive: set when the C6 notifies "+IPD,<id>,<len>" that data is
    // buffered and waiting to be pulled with CIPRECVDATA.
    bool _recv_pending = false;

    // Byte-level parser state for +IPD handling. In CIPMUX=1 mode the header
    // is "+IPD,<link_id>,<len>:"; the parser also tolerates the bare
    // "+IPD,<len>:" single-connection form.
    enum ParseState { PS_LINE, PS_IPD };
    ParseState _state = PS_LINE;
    int _ipd_remain = 0;

    // Captured response body (PSRAM). Stage 1 caps the capture; bytes past
    // the cap are consumed but not stored so the parser stays aligned.
    char* _resp = nullptr;
    int   _resp_len = 0;
    int   _resp_cap = 0;
};