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

private:
    // Custom codes assigned to the modifier keys by Tca8418_Map_Lvgl in
    // t_display_p4_keyboard_config.h.
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
            if (tp.info[i].press_flag == false) continue;          // key-up
            const uint8_t num = tp.info[i].num;
            if ((num == 0) || (num > kMapLen)) continue;
            const uint32_t code = translate(num);
            if (code != 0) push(code);
        }

        _kbd->clear_irq_flag(Cpp_Bus_Driver::Tca8418::Irq_Flag::KEY_EVENTS);
    }

    // Map a scan number to an LVGL key code, applying modifier state.
    // Returns 0 for keys that produce no character (modifiers, F-keys).
    uint32_t translate(uint8_t num) {
        const uint32_t base = Tca8418_Map_Lvgl[num - 1];

        switch (base) {
            case kKeyCaps:  _caps  = !_caps; return 0;
            case kKeyShift: _shift = true;   return 0;
            case kKeyFn:    _fn    = true;   return 0;
            case kKeyAlt:
            case kKeyCtrl:
            case kKeyWin:
            case kKeyRecord:
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

    uint32_t _ring[kRing] = {0};
    size_t   _head = 0;
    size_t   _tail = 0;
};

#endif // CONFIG_BOARD_TYPE_T_DISPLAY_P4_KEYBOARD