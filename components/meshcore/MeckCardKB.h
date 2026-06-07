/*
 * MeckCardKB.h — M5Stack CardKB (I2C keyboard) driver for Meck on T-Display P4
 *
 * Adapted from the T5S3 CardKBKeyboard.h. On the P4 there is no Arduino Wire
 * layer, so the I2C transport goes through cpp_bus_driver's Software_Iic on the
 * CardKB pins (the 1x4 "P1" connector, SDA=48 SCL=47). Both hardware I2C
 * controllers are already taken by other peripherals, so a bit-banged software
 * bus on the spare extension pins is the right transport — the same approach the
 * keyboard variant uses for its IIC_3 devices.
 *
 * read_key() returns 0 when no key is pending, raw printable ASCII for normal
 * characters, or an LV_KEY_* code for the navigation/control keys, so the LVGL
 * consumer can switch on it directly (matches Tca8418_Map_Lvgl's convention).
 *
 * Detection probes 0x5F a few times at boot — the first probe on the freshly
 * configured bit-banged bus can miss. The bus runs at 10 kHz; the CardKB
 * mis-samples at 100 kHz. The 50 ms poll throttle and the three-error bus
 * re-init / back-off are carried over from the T5S3 driver.
 */

#pragma once

#if defined(MECK_CARDKB)

#include <memory>
#include <cstdio>
#include "cpp_bus_driver_library.h"
#include "lvgl.h"
#include "esp_timer.h"
#include "t_display_p4_config.h"   // CARDKB_SDA / CARDKB_SCL / CARDKB_IIC_ADDRESS

// CardKB raw special-key codes (from M5Stack documentation).
#define CARDKB_RAW_UP    0xB5
#define CARDKB_RAW_DOWN  0xB6
#define CARDKB_RAW_LEFT  0xB4
#define CARDKB_RAW_RIGHT 0xB7
#define CARDKB_RAW_TAB   0x09
#define CARDKB_RAW_ESC   0x1B
#define CARDKB_RAW_BS    0x08
#define CARDKB_RAW_ENTER 0x0D
#define CARDKB_RAW_DEL   0x7F

class MeckCardKB {
public:
    MeckCardKB() {}

    // Construct the software-I2C bus on the CardKB pins and probe for the
    // keyboard at 0x5F. Returns true if a CardKB answered.
    bool begin() {
        _bus = std::make_shared<Cpp_Bus_Driver::Software_Iic>(CARDKB_SDA, CARDKB_SCL);
        _bus->begin(10000, CARDKB_IIC_ADDRESS);   // 10 kHz: a real M5 CardKB returns
                                                   // 0x00 idle, but at 100 kHz the
                                                   // bit-banged bus mis-samples (0x80
                                                   // idle); the slower clock lets the
                                                   // line settle. Binds 0x5F.
        // The single probe right after begin() can NACK before the open-drain
        // lines settle and the CardKB answers — the boot bus scan only detects
        // it because it probes the bus dozens of times. Retry a few times (with
        // a short settle between) before concluding nothing is there.
        _detected = false;
        for (int attempt = 0; attempt < 8 && !_detected; attempt++) {
            _detected = _bus->probe(CARDKB_IIC_ADDRESS);
            if (!_detected) {
                uint64_t t0 = esp_timer_get_time();
                while ((esp_timer_get_time() - t0) < 20000ULL) { }   // ~20 ms
            }
        }
        if (_detected) {
            printf("[CardKB] detected at 0x%02X\n", CARDKB_IIC_ADDRESS);
        }
        return _detected;
    }

    bool is_detected() const { return _detected; }

    // Poll for a keypress. Returns 0 if none. Self-throttled to at most one
    // read per _poll_interval_ms; on repeated read failure the bus is
    // re-init'd and polling backs off to 500 ms until reads succeed again.
    uint32_t read_key() {
        if (!_detected || !_bus) return 0;

        uint32_t now = (uint32_t)(esp_timer_get_time() / 1000ULL);
        if ((now - _last_poll_ms) < _poll_interval_ms) return 0;
        _last_poll_ms = now;

        uint8_t raw = 0;
        if (!_bus->read(&raw, 1)) {
            if (++_error_count >= 3) {
                _bus->begin(10000, CARDKB_IIC_ADDRESS);    // recover bus state (10 kHz)
                _poll_interval_ms = 500;                    // back off
                _error_count = 0;
                printf("[CardKB] I2C error recovery, bus re-init\n");
            }
            return 0;
        }
        _error_count = 0;
        _poll_interval_ms = 50;

        if (raw == 0) return 0;

        switch (raw) {
            case CARDKB_RAW_UP:    return LV_KEY_PREV;   // focus previous (used in 4A)
            case CARDKB_RAW_DOWN:  return LV_KEY_NEXT;   // focus next     (used in 4A)
            case CARDKB_RAW_LEFT:  return LV_KEY_LEFT;
            case CARDKB_RAW_RIGHT: return LV_KEY_RIGHT;
            case CARDKB_RAW_ENTER: return LV_KEY_ENTER;
            case CARDKB_RAW_BS:    return LV_KEY_BACKSPACE;
            case CARDKB_RAW_DEL:   return LV_KEY_BACKSPACE;   // treat Del as Backspace
            case CARDKB_RAW_ESC:   return LV_KEY_ESC;
            case CARDKB_RAW_TAB:   return LV_KEY_NEXT;
            default:
                if (raw >= 0x20 && raw <= 0x7E) return (uint32_t)raw;   // printable ASCII
                return 0;                                               // ignore unknown
        }
    }

private:
    std::shared_ptr<Cpp_Bus_Driver::Software_Iic> _bus = nullptr;
    bool     _detected         = false;
    uint32_t _last_poll_ms     = 0;
    uint32_t _poll_interval_ms = 50;
    uint8_t  _error_count      = 0;
};

#endif // MECK_CARDKB