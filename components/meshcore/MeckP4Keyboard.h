/*
 * MeckP4Keyboard.h -- LilyGo T-Display-P4-Keyboard (K270) driver for Meck
 *
 * The expansion board carries an XL9555 I/O expander at 0x20 and a TCA8418
 * keypad controller at 0x34, both on a bit-banged I2C bus on the 1x4 "P2"
 * connector (SDA = GPIO46, SCL = GPIO45). Modelled on MeckCardKB.h and on the
 * proven main/keyboard_examples/tca8418 example.
 *
 * Two Software_Iic instances are created on the same pair of pins. That is not
 * redundancy: Software_Iic stores a single bound target address, and each
 * chip's begin() rebinds it, so two chips sharing one bus object would fight
 * over it. The example does the same thing for the same reason.
 *
 * Polled, not interrupt-driven. The example arms a falling-edge ISR on the
 * TCA8418 INT line (GPIO48); here get_multiple_touch_point() is called from an
 * LVGL timer instead, which returns false when the event FIFO is empty. That
 * avoids registering a second GPIO ISR service alongside the radio's, and
 * keeps the driver off GPIO47/48 entirely, so the LDO channel 4 power domain
 * those pins may need never comes into it.
 *
 * read_key() returns 0 when nothing is pending, raw printable ASCII for normal
 * characters, or an LV_KEY_* code for navigation keys -- the same convention
 * MeckCardKB::read_key() uses, so the consumer routing is identical.
 *
 * Key-up events are discarded: the TCA8418 reports both edges (press_flag 1
 * then 0) and emitting on both would double every keystroke.
 */

#pragma once

#if defined(CONFIG_BOARD_TYPE_T_DISPLAY_P4_KEYBOARD)

#include <memory>
#include <cstdio>
#include <cstddef>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "cpp_bus_driver_library.h"
#include "lvgl.h"
#include "t_display_p4_driver.h"           // Init_Ldo_Channel_Power
#include "t_display_p4_keyboard_config.h"   // pins, addresses, Tca8418_Map_Lvgl*

class MeckP4Keyboard {
public:
    MeckP4Keyboard() {}

    // Bring up the expander, reset the keypad controller through it, then
    // start the keypad scanner. Returns false if either chip fails to answer,
    // which is the "no keyboard attached" case and is not an error.
    bool begin() {
        // Power the I/O domain the keyboard bus sits in. Every keyboard example
        // that actually talks to the TCA8418 does this first -- tca8418,
        // screen_tca8418_lvgl_touch_draw and bq25896 all call it -- while Meck
        // itself only ever acquires channel 3. Leaving channel 4 unpowered
        // leaves the bit-banged bus marginal: reads fail intermittently, and a
        // failed read is indistinguishable from "no key pressed" (the vendor
        // driver's get_finger_count() returns -1 on failure and
        // get_multiple_touch_point() treats that the same as a count of 0), so
        // dropped reads surface as keypresses that need repeating.
        Init_Ldo_Channel_Power(4, 3300);

        // Keyboard backlight via the SY7200A's enable/dimming pin. Plain
        // high drives the LED string at full output, which measured about
        // +1 A of battery draw, so the pin is driven with LEDC PWM at 25%
        // duty instead (datasheet dimming range 20 kHz - 1 MHz; 20 kHz
        // keeps it above audible). Channel 1 is the one LilyGo's own
        // example reserves for KEYBOARD_BL; timer 1 avoids sharing
        // whatever timer the display backlight's channel-0 helper binds.
        // Starts off; the LilyGo key toggles it.
        ledc_timer_config_t bl_timer = {};
        bl_timer.speed_mode      = LEDC_LOW_SPEED_MODE;
        bl_timer.duty_resolution = LEDC_TIMER_10_BIT;
        bl_timer.timer_num       = kBlTimer;
        bl_timer.freq_hz         = 20000;
        bl_timer.clk_cfg         = LEDC_AUTO_CLK;
        if (ledc_timer_config(&bl_timer) != ESP_OK) {
            printf("[P4KBD] backlight LEDC timer config failed\n");
        }
        ledc_channel_config_t bl_ch = {};
        bl_ch.gpio_num   = KEYBOARD_BL;
        bl_ch.speed_mode = LEDC_LOW_SPEED_MODE;
        bl_ch.channel    = kBlChannel;
        bl_ch.timer_sel  = kBlTimer;
        bl_ch.duty       = 0;
        bl_ch.hpoint     = 0;
        if (ledc_channel_config(&bl_ch) != ESP_OK) {
            printf("[P4KBD] backlight LEDC channel config failed\n");
        }
        set_backlight(false);

        _xl_bus  = std::make_shared<Cpp_Bus_Driver::Software_Iic>(XL9555_SDA,  XL9555_SCL);
        _kbd_bus = std::make_shared<Cpp_Bus_Driver::Software_Iic>(TCA8418_SDA, TCA8418_SCL);

        _xl  = std::make_unique<Cpp_Bus_Driver::Xl95x5>(
                   _xl_bus,  XL9555_IIC_ADDRESS,  DEFAULT_CPP_BUS_DRIVER_VALUE);
        _kbd = std::make_unique<Cpp_Bus_Driver::Tca8418>(
                   _kbd_bus, TCA8418_IIC_ADDRESS, DEFAULT_CPP_BUS_DRIVER_VALUE);

        if (_xl->begin() == false) {
            printf("[P4KBD] no XL9555 at 0x%02X - keyboard not attached\n",
                   XL9555_IIC_ADDRESS);
            return false;
        }

        // The TCA8418 reset line hangs off expander IO6, not off a P4 GPIO, so
        // the chip driver's own _rst path cannot reach it (hence the
        // DEFAULT_CPP_BUS_DRIVER_VALUE above). Pulse it by hand before
        // starting the controller.
        _xl->pin_mode(XL9555_TCA8418_RST, Cpp_Bus_Driver::Xl95x5::Mode::OUTPUT);
        _xl->pin_write(XL9555_TCA8418_RST, Cpp_Bus_Driver::Xl95x5::Value::HIGH);
        vTaskDelay(pdMS_TO_TICKS(10));
        _xl->pin_write(XL9555_TCA8418_RST, Cpp_Bus_Driver::Xl95x5::Value::LOW);
        vTaskDelay(pdMS_TO_TICKS(10));
        _xl->pin_write(XL9555_TCA8418_RST, Cpp_Bus_Driver::Xl95x5::Value::HIGH);
        vTaskDelay(pdMS_TO_TICKS(10));

        if (_kbd->begin() == false) {
            printf("[P4KBD] no TCA8418 at 0x%02X\n", TCA8418_IIC_ADDRESS);
            return false;
        }

        _kbd->set_keypad_scan_window(0, 0, TCA8418_KEYPAD_SCAN_WIDTH,
                                           TCA8418_KEYPAD_SCAN_HEIGHT);
        _kbd->set_irq_pin_mode(Cpp_Bus_Driver::Tca8418::Irq_Mask::KEY_EVENTS);
        _kbd->clear_irq_flag(Cpp_Bus_Driver::Tca8418::Irq_Flag::KEY_EVENTS);

        _detected = true;
        printf("[P4KBD] keyboard detected (XL9555 0x%02X + TCA8418 0x%02X)\n",
               XL9555_IIC_ADDRESS, TCA8418_IIC_ADDRESS);
        return true;
    }

    bool detected() const { return _detected; }

    // One key per call, 0 when nothing pending. Events are drained from the
    // controller's FIFO into a local ring so a burst of fast typing is not
    // lost between polls.
    uint32_t read_key() {
        if (_detected == false) return 0;
        if (_head != _tail) return pop();
        drain();
        return (_head != _tail) ? pop() : 0;
    }

    // Set the backlight brightness as a percent of full drive (5..100),
    // mapped to LEDC duty. Stores the level for future toggles; if the
    // backlight is currently on, re-applies immediately so the Settings
    // slider is live. 25% == the previous fixed duty (~398 mA measured).
    void set_backlight_level_pct(uint8_t pct) {
        if (pct < 5)   pct = 5;
        if (pct > 100) pct = 100;
        _bl_duty = ((uint32_t)pct * 1023 + 50) / 100;
        if (_backlight) {
            ledc_set_duty(LEDC_LOW_SPEED_MODE, kBlChannel, _bl_duty);
            ledc_update_duty(LEDC_LOW_SPEED_MODE, kBlChannel);
        }
    }

    // ---- GBC raw joypad mode (MeckGBC) ------------------------------------
    // While on, key events update a held-key bitmask on BOTH edges instead
    // of feeding the character ring, so nothing leaks into the UI and the
    // emulator gets true press-and-hold input. read_key() still drains the
    // FIFO (the 30 ms poll keeps running); it just returns nothing. The
    // pending ring is cleared on enable so keystrokes queued before the
    // game do not pop into the UI afterwards, and the Shift/Fn one-shots
    // are cleared so no stale modifier survives the game.
    void set_raw_joypad(bool on) {
        _raw      = on;
        _raw_mask = 0;
        _raw_exit = false;
        _raw_mute = false;
        _head = _tail = 0;
        _shift = false;
        _fn    = false;
    }
    // Held-key bitmask. Bit layout matches Peanut-GB direct.joypad exactly
    // (a 0x01, b 0x02, select 0x04, start 0x08, right 0x10, left 0x20,
    // up 0x40, down 0x80); a set bit means held.
    uint8_t raw_joypad() const { return _raw_mask; }
    // Esc press-edge latch, consumed on read: the emulator's exit signal.
    bool raw_exit_pressed() {
        bool e = _raw_exit;
        _raw_exit = false;
        return e;
    }
    // Microphone (Record) key press-edge latch, consumed on read: the
    // emulator's mute toggle.
    bool raw_mute_pressed() {
        bool m = _raw_mute;
        _raw_mute = false;
        return m;
    }

private:
    // Custom codes assigned to the modifier keys by Tca8418_Map_Lvgl in
    // t_display_p4_keyboard_config.h.
    // Backlight PWM: LEDC channel 1 / timer 1, 10-bit, 20 kHz, 25% duty
    // (50% still measured ~600 mA pack discharge with it on).
    static constexpr ledc_timer_t   kBlTimer   = LEDC_TIMER_1;
    static constexpr ledc_channel_t kBlChannel = LEDC_CHANNEL_1;
    static constexpr uint32_t       kBlDuty    = 256;

    static constexpr uint32_t kKeyCaps   = 0x8B;
    static constexpr uint32_t kKeyAlt    = 0x8C;
    static constexpr uint32_t kKeyCtrl   = 0x8D;
    static constexpr uint32_t kKeyFn     = 0x8E;
    static constexpr uint32_t kKeyWin    = 0x8F;
    static constexpr uint32_t kKeyShift  = 0x90;
    static constexpr uint32_t kKeyF11    = 0x91;
    static constexpr uint32_t kKeyRecord = 0x92;

    static constexpr size_t kMapLen =
        sizeof(Tca8418_Map_Lvgl) / sizeof(Tca8418_Map_Lvgl[0]);

    static constexpr size_t kRing = 16;

    void drain() {
        Cpp_Bus_Driver::Tca8418::Touch_Point tp;
        if (_kbd->get_multiple_touch_point(tp) == false) return;   // FIFO empty

        for (size_t i = 0; i < tp.info.size(); i++) {
            if (tp.info[i].event_type !=
                Cpp_Bus_Driver::Tca8418::Event_Type::KEYPAD) continue;
            const uint8_t num = tp.info[i].num;
            if ((num == 0) || (num > kMapLen)) continue;
            if (_raw) {
                // Raw joypad mode: both edges matter, modifiers ignored,
                // nothing enters the character ring.
                raw_event(Tca8418_Map_Lvgl[num - 1], tp.info[i].press_flag);
                continue;
            }
            if (tp.info[i].press_flag == false) continue;          // key-up
            const uint32_t code = translate(num);
            if (code != 0) push(code);
        }

        _kbd->clear_irq_flag(Cpp_Bus_Driver::Tca8418::Irq_Flag::KEY_EVENTS);
    }

    // Map a base keycode (Tca8418_Map_Lvgl entry, no modifier processing)
    // to its GBC joypad bit and apply the edge. Esc is a press-edge exit
    // latch rather than a joypad bit. Keys outside the mapping are ignored
    // entirely in raw mode. Mapping: arrows = d-pad, K = A, J = B,
    // Enter = Start, Space = Select, Esc = exit.
    void raw_event(uint32_t base, bool pressed) {
        uint8_t bit = 0;
        switch (base) {
            case 'k':          bit = 0x01; break;   // A
            case 'j':          bit = 0x02; break;   // B
            case ' ':          bit = 0x04; break;   // Select
            case LV_KEY_ENTER: bit = 0x08; break;   // Start
            case LV_KEY_RIGHT: bit = 0x10; break;
            case LV_KEY_LEFT:  bit = 0x20; break;
            case LV_KEY_UP:    bit = 0x40; break;
            case LV_KEY_DOWN:  bit = 0x80; break;
            case LV_KEY_ESC:
                if (pressed) _raw_exit = true;
                return;
            case kKeyRecord:                        // microphone key
                if (pressed) _raw_mute = true;
                return;
            default:
                return;
        }
        if (pressed) _raw_mask |= bit;
        else         _raw_mask &= (uint8_t)~bit;
    }

    // Map a scan number to an LVGL key code, applying modifier state.
    // Returns 0 for keys that produce no character (modifiers, F-keys).
    uint32_t translate(uint8_t num) {
        const uint32_t base = Tca8418_Map_Lvgl[num - 1];

        switch (base) {
            case kKeyCaps:  _caps  = !_caps; return 0;
            case kKeyShift: _shift = true;   return 0;
            case kKeyFn:    _fn    = true;   return 0;
            case kKeyWin:   // LilyGo key: toggle the keyboard backlight
                set_backlight(!_backlight);
                return 0;
            case kKeyRecord:
                // Microphone/record key: forwarded to the UI, which uses it
                // to toggle the canned-messages overlay on the compose
                // screens. Previously swallowed like the other specials.
                return kKeyRecord;
            case kKeyAlt:
            case kKeyCtrl:
            case kKeyF11:
                return 0;
            default:
                break;
        }
        if ((base >= 0x81) && (base <= 0x8A)) return 0;   // F1-F10

        // Fn selects the symbol layer for this one key.
        if (_fn) {
            _fn    = false;
            _shift = false;
            const uint32_t sym = Tca8418_Map_Lvgl_Shift[num - 1];
            return ((sym >= 0x20) && (sym <= 0x7E)) ? sym : 0;
        }

        // Letters: Shift (one-shot) and Caps Lock both select upper case, and
        // together they cancel, as on a normal keyboard.
        if ((base >= 'a') && (base <= 'z')) {
            const bool upper = (_shift != _caps);
            _shift = false;
            return upper ? (base - 'a' + 'A') : base;
        }

        // Everything else with Shift held takes the symbol layer if that entry
        // is printable, otherwise falls through unshifted.
        if (_shift) {
            _shift = false;
            const uint32_t sym = Tca8418_Map_Lvgl_Shift[num - 1];
            if ((sym >= 0x20) && (sym <= 0x7E)) return sym;
        }

        return base;
    }

    // Backlight on = the stored duty level (default kBlDuty = 25%), off = 0.
    // On/off is session state only -- it comes up off after every boot; the
    // level is applied from prefs at keyboard init (set_backlight_level_pct).
    void set_backlight(bool on) {
        ledc_set_duty(LEDC_LOW_SPEED_MODE, kBlChannel, on ? _bl_duty : 0);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, kBlChannel);
        _backlight = on;
    }

    void push(uint32_t code) {
        const size_t next = (_tail + 1) % kRing;
        if (next == _head) return;            // ring full: drop oldest input
        _ring[_tail] = code;
        _tail = next;
    }

    uint32_t pop() {
        const uint32_t code = _ring[_head];
        _head = (_head + 1) % kRing;
        return code;
    }

    std::shared_ptr<Cpp_Bus_Driver::Software_Iic> _xl_bus;
    std::shared_ptr<Cpp_Bus_Driver::Software_Iic> _kbd_bus;
    std::unique_ptr<Cpp_Bus_Driver::Xl95x5>       _xl;
    std::unique_ptr<Cpp_Bus_Driver::Tca8418>      _kbd;

    bool _detected = false;
    bool _shift    = false;
    bool _caps     = false;
    bool _fn       = false;
    bool _backlight = false;
    uint32_t _bl_duty = kBlDuty;   // current on-level (LEDC duty, 10-bit)

    uint32_t _ring[kRing] = {0};
    size_t   _head = 0;
    size_t   _tail = 0;

    // GBC raw joypad mode state. The mask and exit latch are byte-sized
    // and read from the emulator task on the other core while drain()
    // writes them in the LVGL task, hence volatile.
    bool             _raw      = false;
    volatile uint8_t _raw_mask = 0;
    volatile bool    _raw_exit = false;
    volatile bool    _raw_mute = false;
};

#endif // CONFIG_BOARD_TYPE_T_DISPLAY_P4_KEYBOARD