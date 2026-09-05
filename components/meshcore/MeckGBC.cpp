/*
 * MeckGBC.cpp -- Game Boy Color emulator (Peanut-GB gbc-rtc-fix core)
 * ----------------------------------------------------------------------------
 * Scope:
 *   - Compiled on every board type. Input: the K270 (MeckP4Keyboard's raw
 *     joypad mode: arrows = d-pad, K = A, J = B, Enter = Start,
 *     Space = Select, Esc = exit) when a keyboard was detected at boot,
 *     otherwise on-screen touch controls (see "Touch controls" below).
 *     Both are OR'd into the joypad, so a docked keyboard and a thumb
 *     both work when the controls are shown.
 *   - Sound via minigb_apu (MIT, vendored): the core's audio_read/
 *     audio_write hooks feed the APU, and once per emulated frame the task
 *     renders 16-bit stereo at 32768 Hz and writes it to the ES8311 through
 *     the firmware's existing I2S helpers (meck_audio_i2s_write). The codec
 *     is woken and reclocked to 32768 Hz on launch and restored to the
 *     audiobook player's 44100 Hz and put back to sleep on exit. If the
 *     audiobook player is playing or paused, it is stopped first (a game
 *     launch is foreground intent; its resume position is kept by the
 *     player). Frame pacing stays on the esp_timer deadline: the APU
 *     produces exactly one frame's worth of samples per frame, so the two
 *     rates balance by construction whether or not the I2S write blocks.
 *   - No battery saves yet: cart RAM starts zeroed each launch and is
 *     discarded on exit. Saves/resume are a later stage.
 *   - Menu and browser are touch-navigated in v1; in-game input is fully
 *     keyboard. Full keyboard nav of the menu/browser is a follow-up.
 *
 * Architecture (all mechanisms verified against this tree and LVGL 9.3.0
 * source during the design session):
 *   - The emulator runs in its own FreeRTOS task pinned to core 1 at
 *     priority 2; every other task in the firmware is created unpinned and
 *     migrates freely to core 0. The 16 KB stack lives in PSRAM
 *     (xTaskCreatePinnedToCoreWithCaps + MALLOC_CAP_SPIRAM): plain task
 *     creation takes the stack from internal RAM, and no contiguous 16 KB
 *     internal block reliably survives in the full firmware ("task create
 *     failed" on hardware). CONFIG_FREERTOS_TASK_CREATE_ALLOW_EXT_MEM
 *     defaults to y on the P4, and this task performs no flash operations,
 *     which is the restriction on PSRAM-stacked tasks. A WithCaps task
 *     must not plain-self-delete (the caps-allocated stack would leak), so
 *     on stop it parks in vTaskSuspend and the LVGL timer reaps it with
 *     vTaskDeleteWithCaps. The task paces gb_run_frame() to the GBC's
 *     59.73 Hz (16742 us/frame): each frame arms a one-shot esp_timer
 *     for the remaining time to the accumulated deadline and blocks on a
 *     semaphore the timer gives. esp_timer is microsecond-resolution, so
 *     pacing is exact regardless of the 10 ms FreeRTOS tick -- the
 *     earlier vTaskDelay(1) loop rounded every wait up to a tick and ran
 *     ~57 fps, audibly slow in tempo-critical titles (Halo).
 *   - Per scanline the core hands 160 palette indices; the draw callback
 *     resolves them through gb->cgb.fixPalette (RGB555, red high -- the
 *     core swaps the GBC's native red/blue when latching palette writes)
 *     into a 160x144 PSRAM buffer. Per frame that buffer is converted to
 *     RGB565 (four bit-ops per pixel -- the bench used a 64 KB internal
 *     LUT instead, but no 64 KB contiguous internal block survives in the
 *     full firmware, which is what "buffer allocation failed" was) and
 *     3x-scaled into a 480x432 PSRAM buffer.
 *   - An lv_canvas on the emulator screen wraps the 480x432 buffer. An
 *     lv_timer (LVGL task context, so no locking needed) invalidates the
 *     canvas at 50 Hz -- the panel refreshes at 49.8 Hz, so pushing faster
 *     is waste -- and calls lv_display_trigger_activity so the idle
 *     watcher never blanks the screen mid-game. LVGL's normal partial
 *     flush carries the pixels to the panel; no new panel interaction.
 *   - Single framebuffer in v1: the emulator task may be writing while
 *     LVGL copies, worst case a brief horizontal shear on fast motion. If
 *     visible on glass, the follow-up is a double buffer + pointer swap.
 *
 * The bundled ROM is ucity v1.3 (GPL-3.0, github.com/AntonioND/ucity),
 * 128 KB, CGB-only, MBC5. It is embedded in the app image and copied to
 * /sdcard/roms/ucity.gbc the first time the Games menu is opened with an
 * SD card present, so every user has a game before downloading anything.
 * The distributed Meck binary is already effectively GPL-3.0 (see the
 * release-notes licence section), so bundling a GPL-3.0 ROM is clean.
 */

#include "MeckGBC.h"

#include "sdkconfig.h"

#include "lvgl.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/idf_additions.h"   // xTaskCreatePinnedToCoreWithCaps
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "freertos/semphr.h"

#include <dirent.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <string.h>
#include <strings.h>
#include <stdlib.h>
#include <stdio.h>

// The vendored core trips two warning classes that this tree's -Werror=all
// promotes to errors: -Wsequence-point on its "x = (++x) & 0x3F" palette
// auto-increment lines (well-defined under the C++17-and-later sequencing
// this build uses, gnu++2b) and -Wmisleading-indentation. Both are cosmetic
// here, and suppressing them keeps peanut_gb.h byte-identical to upstream.
// The APU declarations must precede the core: with ENABLE_SOUND the core
// calls audio_read/audio_write directly. minigb_apu.h has no C++ linkage
// guard of its own, and minigb_apu.c is compiled as C.
extern "C" {
#include "minigb_apu.h"
}
#define PEANUT_FULL_GBC_SUPPORT 1
#define ENABLE_LCD 1
#define ENABLE_SOUND 1
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wsequence-point"
#pragma GCC diagnostic ignored "-Wmisleading-indentation"
#include "peanut_gb.h"
#pragma GCC diagnostic pop

#include "MeckAudio.h"       // meck_audio_get_state / meck_audio_stop / meck_audio_ready
#include "driver/i2s_std.h"  // i2s_slot_mode_t for the reconfig helper

// ---- Audio path (es8311.cpp + main.cpp, all extern "C") --------------------
extern "C" esp_err_t meck_audio_i2s_write(void *audio_buffer, size_t len,
                                          size_t *bytes_written, uint32_t timeout_ticks);
extern "C" esp_err_t meck_audio_i2s_reconfig(uint32_t rate, uint32_t bps,
                                             i2s_slot_mode_t channels);
extern "C" void meck_audio_codec_wake(void);
extern "C" void meck_audio_codec_sleep(void);

// ---- Shared helpers from MeckUI.cpp ----------------------------------------
extern "C" void meck_ui_set_font(lv_obj_t* obj, const lv_font_t* base,
                                 lv_style_selector_t part);
extern "C" {
    extern lv_font_t meck_montserrat_18;
    extern lv_font_t meck_montserrat_24;
    extern lv_font_t meck_montserrat_28;
}
extern "C" void lock_screen_scroll(lv_obj_t *scr);

// Raw joypad access, implemented in MeckUI.cpp as thin wrappers over the
// keyboard instance. Bit layout matches Peanut-GB direct.joypad exactly
// (a 0x01, b 0x02, select 0x04, start 0x08, right 0x10, left 0x20,
// up 0x40, down 0x80); a set bit means held.
extern "C" void    meck_p4kbd_set_raw_joypad(bool on);
extern "C" uint8_t meck_p4kbd_raw_joypad(void);
extern "C" bool    meck_p4kbd_raw_exit_pressed(void);
extern "C" bool    meck_p4kbd_raw_mute_pressed(void); // mic key, press edge
extern "C" bool    meck_p4kbd_present(void);       // detected at boot

// Embedded bundled ROM (see components/meshcore/CMakeLists.txt).
extern const uint8_t ucity_rom_start[] asm("_binary_ucity_rom_gbc_start");
extern const uint8_t ucity_rom_end[]   asm("_binary_ucity_rom_gbc_end");

// ---- Constants --------------------------------------------------------------
#define GBC_ROM_DIR       "/sdcard/roms"
#define GBC_SEED_PATH     GBC_ROM_DIR "/ucity.gbc"
#define GBC_MAX_ROMS      64
#define GBC_NAME_MAX      64

#define GB_W   160
#define GB_H   144
#define SCALE  3
#define OUT_W  (GB_W * SCALE)
#define OUT_H  (GB_H * SCALE)

// Cart RAM: allocate the largest standard bank set (RAM size code 0x04,
// 128 KB) regardless of the header, so every MBC configuration fits and
// launch never fails on an odd size byte. PSRAM is plentiful.
#define GBC_CRAM_SIZE 0x20000

#define GBC_FRAME_US 16742   // 59.73 Hz

// ---- Screens (created on show, deleted on exit) -----------------------------
static lv_obj_t *scr_games_menu  = NULL;
static lv_obj_t *scr_rom_browser = NULL;
static lv_obj_t *scr_gbc         = NULL;
static lv_obj_t *g_return_screen = NULL;   // screen active before the menu
static lv_obj_t *g_gbc_canvas    = NULL;
static lv_obj_t *g_gbc_status    = NULL;   // load-error label on the emu screen

// ---- Touch controls ---------------------------------------------------------
// main.cpp's touch read callback publishes EVERY finger (raw panel
// coordinates, before LVGL's rotation) on each poll, including "no
// fingers", via meck_touch_publish(). The emulator task snapshots that per
// frame, applies LVGL's own rotation transform (lv_indev.c
// indev_pointer_proc: 180/270 mirror both axes, 90/270 swap with
// x = ver_res - y - 1) so zones are defined in screen coordinates, and
// hit-tests the fingers against the control zones into a held-key mask.
// Because the fingers are read from the controller directly, several can
// be held at once (d-pad + A), which LVGL's single-point pointer could not
// deliver. LVGL still receives the single-finger case exactly as before,
// so the back chevron keeps working; the drawn controls are non-clickable
// so LVGL ignores them. While a game runs the touch poll period is lowered
// from the UI's 50 ms to 16 ms so short taps are never missed, and
// restored on exit.
#define GBC_TOUCH_MAX 5
static portMUX_TYPE     s_touch_mux = portMUX_INITIALIZER_UNLOCKED;
static uint8_t          s_touch_n   = 0;
static uint16_t         s_touch_x[GBC_TOUCH_MAX];
static uint16_t         s_touch_y[GBC_TOUCH_MAX];
static bool             s_touch_ui  = false;   // controls shown this session
static volatile uint8_t s_touch_mask = 0;      // last mask, for highlighting
static uint32_t         s_touch_period_restore = 50;   // UI poll period (ms)

// Zone geometry in screen coordinates, chosen per orientation at launch.
static struct {
    int16_t dpad_cx, dpad_cy, dpad_half, dpad_dead;
    int16_t arm_len, arm_thick, arm_gap;     // drawn cross: arm size, gap from hub
    int16_t a_cx, a_cy, b_cx, b_cy, btn_r;
    int16_t sel_cx, sel_cy, start_cx, start_cy, pill_w, pill_h;
    int16_t mute_cx, mute_cy;                // MUTE pill (toggle, press edge)
} s_zone;

// Overlay widgets (children of scr_gbc), by joypad bit for highlighting.
static lv_obj_t *s_ov_dpad[4] = { NULL, NULL, NULL, NULL };   // R, L, U, D arms
static lv_obj_t *s_ov_a = NULL, *s_ov_b = NULL, *s_ov_sel = NULL, *s_ov_start = NULL;
static lv_obj_t *s_ov_mute = NULL;

// Mute: toggled by the MUTE pill (touch, press edge) or the K270 microphone
// key (raw mode, press edge). Both paths land in the LVGL timer, which owns
// the codec call (meck_audio_set_dac_mute). The emulator task only reports
// the touch press edge through s_touch_mute_req.
static bool             s_muted = false;
static volatile uint8_t s_touch_mute_req = 0;    // incremented on press edge
static uint8_t          s_touch_mute_ack = 0;

extern "C" void meck_touch_publish(uint8_t count, const uint16_t *xs, const uint16_t *ys)
{
    if (count > GBC_TOUCH_MAX) count = GBC_TOUCH_MAX;
    taskENTER_CRITICAL(&s_touch_mux);
    s_touch_n = count;
    for (uint8_t i = 0; i < count; i++) { s_touch_x[i] = xs[i]; s_touch_y[i] = ys[i]; }
    taskEXIT_CRITICAL(&s_touch_mux);
}

// Held-key mask from the current fingers (emulator task, per frame).
static uint8_t touch_joypad_mask(void)
{
    uint16_t xs[GBC_TOUCH_MAX], ys[GBC_TOUCH_MAX];
    uint8_t n;
    taskENTER_CRITICAL(&s_touch_mux);
    n = s_touch_n;
    for (uint8_t i = 0; i < n; i++) { xs[i] = s_touch_x[i]; ys[i] = s_touch_y[i]; }
    taskEXIT_CRITICAL(&s_touch_mux);

    const lv_display_rotation_t rot = lv_display_get_rotation(NULL);
    const int32_t phys_w = lv_display_get_physical_horizontal_resolution(NULL);
    const int32_t phys_h = lv_display_get_physical_vertical_resolution(NULL);

    static bool s_mute_prev = false;
    bool mute_hit = false;
    uint8_t mask = 0;
    for (uint8_t i = 0; i < n; i++) {
        int32_t x = xs[i], y = ys[i];
        if (rot == LV_DISPLAY_ROTATION_180 || rot == LV_DISPLAY_ROTATION_270) {
            x = phys_w - x - 1;
            y = phys_h - y - 1;
        }
        if (rot == LV_DISPLAY_ROTATION_90 || rot == LV_DISPLAY_ROTATION_270) {
            const int32_t t = y;
            y = x;
            x = phys_h - t - 1;
        }
        // D-pad: square zone, dead centre, diagonals allowed.
        {
            const int32_t dx = x - s_zone.dpad_cx, dy = y - s_zone.dpad_cy;
            if (dx >= -s_zone.dpad_half && dx <= s_zone.dpad_half &&
                dy >= -s_zone.dpad_half && dy <= s_zone.dpad_half) {
                if (dx >  s_zone.dpad_dead) mask |= 0x10;   // right
                if (dx < -s_zone.dpad_dead) mask |= 0x20;   // left
                if (dy < -s_zone.dpad_dead) mask |= 0x40;   // up
                if (dy >  s_zone.dpad_dead) mask |= 0x80;   // down
            }
        }
        // A / B: circles.
        {
            const int32_t r2 = (int32_t)s_zone.btn_r * s_zone.btn_r;
            int32_t dx = x - s_zone.a_cx, dy = y - s_zone.a_cy;
            if (dx * dx + dy * dy <= r2) mask |= 0x01;
            dx = x - s_zone.b_cx; dy = y - s_zone.b_cy;
            if (dx * dx + dy * dy <= r2) mask |= 0x02;
        }
        // Select / Start / Mute: pills (rect test with a little slack).
        {
            const int32_t hw = s_zone.pill_w / 2 + 10, hh = s_zone.pill_h / 2 + 10;
            if (x >= s_zone.sel_cx - hw && x <= s_zone.sel_cx + hw &&
                y >= s_zone.sel_cy - hh && y <= s_zone.sel_cy + hh) mask |= 0x04;
            if (x >= s_zone.start_cx - hw && x <= s_zone.start_cx + hw &&
                y >= s_zone.start_cy - hh && y <= s_zone.start_cy + hh) mask |= 0x08;
            if (x >= s_zone.mute_cx - hw && x <= s_zone.mute_cx + hw &&
                y >= s_zone.mute_cy - hh && y <= s_zone.mute_cy + hh) mute_hit = true;
        }
    }
    // Mute is a toggle: report the press edge only.
    if (mute_hit && !s_mute_prev) s_touch_mute_req = (uint8_t)(s_touch_mute_req + 1);
    s_mute_prev = mute_hit;
    return mask;
}

// Choose zone geometry for the current orientation.
static void touch_layout(int32_t w, int32_t h, bool portrait)
{
    // The drawn cross: each arm starts arm_gap from the hub centre and is
    // arm_len long by arm_thick wide. arm_gap > arm_thick / 2 keeps the
    // arms from overlapping at the hub (the first version overlapped).
    if (portrait) {
        // Game at the top; thumbs below. 568 x 1232 reference.
        s_zone.dpad_cx = 160;      s_zone.dpad_cy = h - 430;
        s_zone.dpad_half = 150;    s_zone.dpad_dead = 30;
        s_zone.arm_len = 100;      s_zone.arm_thick = 80;   s_zone.arm_gap = 48;
        s_zone.btn_r = 66;
        s_zone.a_cx = w - 100;     s_zone.a_cy = h - 470;
        s_zone.b_cx = w - 205;     s_zone.b_cy = h - 345;
        s_zone.pill_w = 110;       s_zone.pill_h = 48;
        s_zone.sel_cx = w / 2 - 85;   s_zone.sel_cy = h - 170;
        s_zone.start_cx = w / 2 + 85; s_zone.start_cy = h - 170;
        s_zone.mute_cx = w - 70;      s_zone.mute_cy = 50;   // top-right, opposite Back
    } else {
        // Game centred; d-pad in the left margin, buttons in the right.
        s_zone.dpad_cx = 188;      s_zone.dpad_cy = h / 2;
        s_zone.dpad_half = 150;    s_zone.dpad_dead = 30;
        s_zone.arm_len = 90;       s_zone.arm_thick = 72;   s_zone.arm_gap = 44;
        s_zone.btn_r = 60;
        s_zone.a_cx = w - 100;     s_zone.a_cy = h / 2 - 70;
        s_zone.b_cx = w - 235;     s_zone.b_cy = h / 2 + 50;
        s_zone.pill_w = 100;       s_zone.pill_h = 40;
        s_zone.sel_cx = w / 2 - 100;  s_zone.sel_cy = h - 34;
        s_zone.start_cx = w / 2 + 100; s_zone.start_cy = h - 34;
        s_zone.mute_cx = w - 70;      s_zone.mute_cy = 34;
    }
}

static lv_obj_t* touch_make_shape(lv_obj_t *parent, int32_t cx, int32_t cy,
                                  int32_t wdt, int32_t hgt, int32_t radius,
                                  const char *label)
{
    lv_obj_t *o = lv_obj_create(parent);
    lv_obj_remove_flag(o, LV_OBJ_FLAG_CLICKABLE);     // LVGL ignores it
    lv_obj_remove_flag(o, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_size(o, wdt, hgt);
    lv_obj_set_pos(o, cx - wdt / 2, cy - hgt / 2);
    lv_obj_set_style_radius(o, radius, 0);
    lv_obj_set_style_bg_color(o, lv_color_white(), 0);
    lv_obj_set_style_bg_opa(o, LV_OPA_20, 0);
    lv_obj_set_style_border_width(o, 2, 0);
    lv_obj_set_style_border_color(o, lv_color_white(), 0);
    lv_obj_set_style_border_opa(o, LV_OPA_40, 0);
    lv_obj_set_style_pad_all(o, 0, 0);
    if (label) {
        lv_obj_t *l = lv_label_create(o);
        lv_label_set_text(l, label);
        lv_obj_set_style_text_color(l, lv_color_white(), 0);
        lv_obj_set_style_text_opa(l, LV_OPA_70, 0);
        meck_ui_set_font(l, &meck_montserrat_24, 0);
        lv_obj_center(l);
    }
    return o;
}

// Draw the controls on the emulator screen from the chosen zones.
static void touch_build_overlay(lv_obj_t *scr)
{
    const int32_t L = s_zone.arm_len, T = s_zone.arm_thick;
    const int32_t off = s_zone.arm_gap + L / 2;     // arm centre from hub
    // D-pad arms: right, left, up, down (index = bit order 0x10..0x80).
    s_ov_dpad[0] = touch_make_shape(scr, s_zone.dpad_cx + off, s_zone.dpad_cy, L, T, 14, NULL);
    s_ov_dpad[1] = touch_make_shape(scr, s_zone.dpad_cx - off, s_zone.dpad_cy, L, T, 14, NULL);
    s_ov_dpad[2] = touch_make_shape(scr, s_zone.dpad_cx, s_zone.dpad_cy - off, T, L, 14, NULL);
    s_ov_dpad[3] = touch_make_shape(scr, s_zone.dpad_cx, s_zone.dpad_cy + off, T, L, 14, NULL);
    s_ov_mute  = touch_make_shape(scr, s_zone.mute_cx, s_zone.mute_cy, s_zone.pill_w, s_zone.pill_h, LV_RADIUS_CIRCLE, "MUTE");
    s_ov_a     = touch_make_shape(scr, s_zone.a_cx, s_zone.a_cy, s_zone.btn_r * 2, s_zone.btn_r * 2, LV_RADIUS_CIRCLE, "A");
    s_ov_b     = touch_make_shape(scr, s_zone.b_cx, s_zone.b_cy, s_zone.btn_r * 2, s_zone.btn_r * 2, LV_RADIUS_CIRCLE, "B");
    s_ov_sel   = touch_make_shape(scr, s_zone.sel_cx, s_zone.sel_cy, s_zone.pill_w, s_zone.pill_h, LV_RADIUS_CIRCLE, "SELECT");
    s_ov_start = touch_make_shape(scr, s_zone.start_cx, s_zone.start_cy, s_zone.pill_w, s_zone.pill_h, LV_RADIUS_CIRCLE, "START");
}

static void touch_set_highlight(lv_obj_t *o, bool held)
{
    if (o) lv_obj_set_style_bg_opa(o, held ? LV_OPA_60 : LV_OPA_20, 0);
}

// Reflect the held mask in the overlay (LVGL timer context).
static void touch_update_overlay(void)
{
    const uint8_t m = s_touch_mask;
    touch_set_highlight(s_ov_dpad[0], m & 0x10);
    touch_set_highlight(s_ov_dpad[1], m & 0x20);
    touch_set_highlight(s_ov_dpad[2], m & 0x40);
    touch_set_highlight(s_ov_dpad[3], m & 0x80);
    touch_set_highlight(s_ov_a,     m & 0x01);
    touch_set_highlight(s_ov_b,     m & 0x02);
    touch_set_highlight(s_ov_sel,   m & 0x04);
    touch_set_highlight(s_ov_start, m & 0x08);
    touch_set_highlight(s_ov_mute,  s_muted);    // lit while muted
}

static void touch_clear_overlay_refs(void)
{
    for (int i = 0; i < 4; i++) s_ov_dpad[i] = NULL;
    s_ov_a = s_ov_b = s_ov_sel = s_ov_start = s_ov_mute = NULL;
}


// Pointer indev read period: 16 ms while a game runs, UI value on exit.
static void touch_set_poll_period(uint32_t ms)
{
    lv_indev_t *ind = NULL;
    while ((ind = lv_indev_get_next(ind)) != NULL) {
        if (lv_indev_get_type(ind) != LV_INDEV_TYPE_POINTER) continue;
        lv_timer_t *t = lv_indev_get_read_timer(ind);
        if (t) lv_timer_set_period(t, ms);
    }
}

// ---- Emulator state ---------------------------------------------------------
// The core context (49952 bytes on-target) and the 160x144 RGB555 scanline
// buffer (46080 bytes) are heap-allocated in PSRAM, NOT statics: as statics
// they were ~98 KB of internal-SRAM .bss, which starved the esp_psram 32 KB
// internal/DMA reserve at boot and the device aborted before app_main
// ("Could not reserve internal/DMA pool"). The bench ran them internal, so
// PSRAM placement spends some of the measured 2.14x speed margin; if the
// frame rate visibly sags, the fallback is a runtime internal alloc for
// just the core context.
static struct gb_s     *s_gb   = NULL;
static uint16_t        *s_gbfb = NULL;             // GB_H*GB_W RGB555
static uint16_t        *s_outfb   = NULL;          // 480x432 RGB565, PSRAM
// ROM arena: PSRAM, grow-only, RETAINED across sessions (never freed on
// exit). Freeing a 2 MB ROM buffer per session left a hole that the
// screen-off framebuffer guard could be carved out of, after which no
// 2 MB contiguous block existed and the next launch failed ("ROM alloc
// failed" after a screen off/on cycle, seen on hardware). Allocated once
// at the first launch, when PSRAM is least fragmented, and reused.
static uint8_t         *s_rom     = NULL;
static size_t           s_rom_cap = 0;             // arena capacity
static size_t           s_rom_size = 0;            // bytes of the loaded ROM
static uint8_t         *s_cram    = NULL;          // 128 KB, PSRAM
static bool             s_audio_on  = false;  // codec taken for this session
static int16_t         *s_audio_buf = NULL;   // one frame of stereo s16
#define GBC_AUDIO_FRAME_BYTES (AUDIO_SAMPLES * 2 * sizeof(int16_t))
static size_t           s_save_size = 0;      // battery-save bytes (0 = none)
static char             s_save_path[192];
static volatile bool    s_stop         = false;
static volatile bool    s_task_stopped = false;
static bool             s_running      = false;
static TaskHandle_t     s_task    = NULL;
static esp_timer_handle_t s_pace_timer = NULL;   // one-shot frame pacer
static SemaphoreHandle_t  s_pace_sem   = NULL;   // given by the pacer

// Flip mute (LVGL timer context). DAC volume 0 <-> the user's level.
static void gbc_toggle_mute(void)
{
    if (!s_audio_on) return;
    s_muted = !s_muted;
    meck_audio_set_dac_mute(s_muted);
    printf("[GBC] %s\n", s_muted ? "muted" : "unmuted");
}

static lv_timer_t      *s_timer   = NULL;

// ---- Core callbacks (validated natively and on-target in the bench) ---------
static uint8_t rom_read(struct gb_s *gb, const uint_fast32_t addr)
{
    (void)gb;
    return s_rom[addr];
}

static uint8_t cram_read(struct gb_s *gb, const uint_fast32_t addr)
{
    (void)gb;
    return s_cram[addr];
}

static void cram_write(struct gb_s *gb, const uint_fast32_t addr, const uint8_t val)
{
    (void)gb;
    s_cram[addr] = val;
}

static void gb_error_cb(struct gb_s *gb, const enum gb_error_e err, const uint16_t addr)
{
    (void)gb;
    // Core fault mid-run (bad opcode / unsupported MBC behaviour). Flag the
    // task to stop; the LVGL timer notices and unwinds to the browser.
    printf("[GBC] core error %d at 0x%04X -- stopping\n", (int)err, (unsigned)addr);
    s_stop = true;
}

static void draw_line(struct gb_s *gb, const uint8_t *pixels, const uint_fast8_t line)
{
    uint16_t *dst = &s_gbfb[(size_t)line * GB_W];
    for (int x = 0; x < GB_W; x++)
        dst[x] = gb->cgb.fixPalette[pixels[x]];
}

// LUT-convert and 3x-scale the frame into the canvas buffer (the exact
// code path benchmarked in gbc_bench_test).
static void blit_scaled(void)
{
    for (int y = 0; y < GB_H; y++) {
        uint16_t       *row0 = &s_outfb[(size_t)(y * SCALE) * OUT_W];
        const uint16_t *src  = &s_gbfb[(size_t)y * GB_W];
        uint16_t       *d    = row0;
        for (int x = 0; x < GB_W; x++) {
            // RGB555 (R14-10 G9-5 B4-0) -> RGB565: R and G shift up one
            // (G widens to 6 bits), G's top bit fills the new low G bit,
            // B keeps its 5 bits.
            const uint16_t v = src[x];
            const uint16_t c = (uint16_t)(((v & 0x7FE0) << 1) |
                                          ((v & 0x0200) >> 4) |
                                          (v & 0x1F));
            d[0] = c;
            d[1] = c;
            d[2] = c;
            d += SCALE;
        }
        memcpy(row0 + OUT_W,     row0, OUT_W * sizeof(uint16_t));
        memcpy(row0 + 2 * OUT_W, row0, OUT_W * sizeof(uint16_t));
    }
}

// ---- Emulator task ----------------------------------------------------------
static void gbc_pace_cb(void *arg)
{
    (void)arg;
    // esp_timer dispatches from its own task on this build, so a plain give
    // is correct (not the FromISR variant).
    if (s_pace_sem) xSemaphoreGive(s_pace_sem);
}

static void gbc_task(void *arg)
{
    (void)arg;
    printf("[GBC] task start (core %d)\n", (int)xPortGetCoreID());
    int64_t next = esp_timer_get_time();
    while (!s_stop) {
        // Held keys (keyboard raw mode) OR touch zones -> joypad. Peanut-GB's
        // register is active-low.
        {
            uint8_t m = meck_p4kbd_raw_joypad();
            if (s_touch_ui) {
                const uint8_t t = touch_joypad_mask();
                s_touch_mask = t;
                m |= t;
            }
            s_gb->direct.joypad = (uint8_t)~m;
        }
        gb_run_frame(s_gb);
        blit_scaled();
        if (s_audio_on) {
            // Render this frame's APU output and hand it to the codec.
            audio_callback(NULL, (uint8_t*)s_audio_buf, (int)GBC_AUDIO_FRAME_BYTES);
            size_t put = 0;
            meck_audio_i2s_write(s_audio_buf, GBC_AUDIO_FRAME_BYTES, &put, 100);
        }
        next += GBC_FRAME_US;
        int64_t now = esp_timer_get_time();
        if (next <= now) {
            next = now;               // fell behind: resync, don't spiral
            continue;
        }
        // Sleep precisely until the deadline: arm the one-shot pacer for the
        // remaining microseconds and block on its semaphore. The take has a
        // bounded timeout so a lost give can never wedge the task.
        esp_timer_start_once(s_pace_timer, (uint64_t)(next - now));
        xSemaphoreTake(s_pace_sem, pdMS_TO_TICKS(50));
    }
    printf("[GBC] task stop\n");
    s_task_stopped = true;
    // Park; the LVGL timer deletes this task with vTaskDeleteWithCaps,
    // which frees the PSRAM stack. Plain vTaskDelete(NULL) here would
    // leak it (statically-created tasks are not reaped by the idle task).
    vTaskSuspend(NULL);
}

// ---- Buffers ----------------------------------------------------------------
static bool gbc_alloc_buffers(void)
{
    if (!s_outfb)
        s_outfb = (uint16_t*)heap_caps_malloc((size_t)OUT_W * OUT_H * sizeof(uint16_t),
                                              MALLOC_CAP_SPIRAM);
    if (!s_gb)
        s_gb = (struct gb_s*)heap_caps_malloc(sizeof(struct gb_s),
                                              MALLOC_CAP_SPIRAM);
    if (!s_gbfb)
        s_gbfb = (uint16_t*)heap_caps_malloc((size_t)GB_H * GB_W * sizeof(uint16_t),
                                             MALLOC_CAP_SPIRAM);
    if (!s_cram)
        s_cram = (uint8_t*)heap_caps_malloc(GBC_CRAM_SIZE, MALLOC_CAP_SPIRAM);
    if (!s_audio_buf)
        s_audio_buf = (int16_t*)heap_caps_malloc(GBC_AUDIO_FRAME_BYTES, MALLOC_CAP_SPIRAM);
    // Name the failing buffer: "buffer allocation failed" alone cost a
    // debugging round trip when the old internal-RAM LUT was the culprit.
    if (!s_outfb) printf("[GBC] outfb alloc failed\n");
    if (!s_gb)    printf("[GBC] gb_s alloc failed\n");
    if (!s_gbfb)  printf("[GBC] gbfb alloc failed\n");
    if (!s_cram)  printf("[GBC] cram alloc failed\n");
    if (!s_audio_buf) printf("[GBC] audio buf alloc failed\n");
    return s_outfb && s_cram && s_gb && s_gbfb && s_audio_buf;
}

// ---- Screen plumbing --------------------------------------------------------
static void gbc_show_browser(void);         // forward
static void gbc_build_and_load_menu(void);  // forward

static lv_obj_t* gbc_make_screen(const char *title_text, lv_event_cb_t back_cb)
{
    lv_obj_t *scr = lv_obj_create(NULL);
    lock_screen_scroll(scr);
    lv_obj_set_style_bg_color(scr, lv_color_black(), 0);

    lv_obj_t *btn_back = lv_button_create(scr);
    lv_obj_set_size(btn_back, 80, 50);
    lv_obj_set_pos(btn_back, 10, 25);
    lv_obj_set_style_bg_opa(btn_back, 0, 0);
    lv_obj_t *back_lbl = lv_label_create(btn_back);
    lv_label_set_text(back_lbl, LV_SYMBOL_LEFT);
    lv_obj_set_style_text_color(back_lbl, lv_palette_main(LV_PALETTE_GREEN), 0);
    meck_ui_set_font(back_lbl, &meck_montserrat_24, 0);
    lv_obj_center(back_lbl);
    lv_obj_add_event_cb(btn_back, back_cb, LV_EVENT_CLICKED, NULL);

    lv_obj_t *title = lv_label_create(scr);
    lv_label_set_text(title, title_text);
    lv_obj_set_style_text_color(title, lv_palette_main(LV_PALETTE_GREEN), 0);
    meck_ui_set_font(title, &meck_montserrat_28, 0);
    lv_obj_align(title, LV_ALIGN_TOP_LEFT, 100, 28);

    return scr;
}

static lv_obj_t* gbc_make_row(lv_obj_t *parent, const char *text, int y,
                              lv_event_cb_t cb, void *user_data)
{
    // Same look as the settings rows: dark pill, light text, full width.
    lv_obj_t *btn = lv_button_create(parent);
    lv_obj_set_size(btn, lv_display_get_horizontal_resolution(NULL) - 40, 60);
    lv_obj_set_pos(btn, 20, y);
    lv_obj_set_style_bg_color(btn, lv_color_make(25, 25, 35), 0);
    lv_obj_set_style_radius(btn, 10, 0);
    lv_obj_set_style_border_width(btn, 1, 0);
    lv_obj_set_style_border_color(btn, lv_color_make(50, 50, 60), 0);
    if (cb) lv_obj_add_event_cb(btn, cb, LV_EVENT_CLICKED, user_data);

    lv_obj_t *lbl = lv_label_create(btn);
    lv_label_set_text(lbl, text);
    lv_obj_set_style_text_color(lbl, lv_color_white(), 0);
    meck_ui_set_font(lbl, &meck_montserrat_18, 0);
    lv_obj_align(lbl, LV_ALIGN_LEFT_MID, 10, 0);
    return btn;
}

static void gbc_delete_screen(lv_obj_t **scr)
{
    if (*scr) {
        lv_obj_delete(*scr);
        *scr = NULL;
    }
}

// ---- SD seeding -------------------------------------------------------------
// Ensure /sdcard/roms exists and carries the bundled ucity ROM. Quiet no-op
// when the SD card is absent (mkdir fails) or the file is already there.
static void gbc_seed_sd(void)
{
    struct stat st;
    if (stat(GBC_ROM_DIR, &st) != 0) {
        if (mkdir(GBC_ROM_DIR, 0775) != 0) {
            printf("[GBC] no SD card or cannot create %s\n", GBC_ROM_DIR);
            return;
        }
    }
    if (stat(GBC_SEED_PATH, &st) == 0) return;   // already seeded

    const size_t sz = (size_t)(ucity_rom_end - ucity_rom_start);
    FILE *f = fopen(GBC_SEED_PATH, "wb");
    if (!f) {
        printf("[GBC] cannot write %s\n", GBC_SEED_PATH);
        return;
    }
    size_t wrote = fwrite(ucity_rom_start, 1, sz, f);
    fclose(f);
    printf("[GBC] seeded %s (%u bytes)\n", GBC_SEED_PATH, (unsigned)wrote);
}

// ---- Emulator launch / teardown --------------------------------------------
static void gbc_stop_and_leave(void)
{
    // Called from the LVGL timer once the task has confirmed it stopped
    // (parked in vTaskSuspend). Reap it: frees the PSRAM stack and TCB.
    if (s_task) vTaskDeleteWithCaps(s_task);
    if (s_pace_timer) { esp_timer_stop(s_pace_timer); esp_timer_delete(s_pace_timer); s_pace_timer = NULL; }
    if (s_pace_sem)   { vSemaphoreDelete(s_pace_sem); s_pace_sem = NULL; }
    // Hand the codec back: audiobook clock, then sleep (the player wakes
    // it again on its own next play).
    if (s_audio_on) {
        if (s_muted) { meck_audio_set_dac_mute(false); s_muted = false; }
        meck_audio_i2s_reconfig(44100, 16, I2S_SLOT_MODE_STEREO);
        meck_audio_codec_sleep();
        s_audio_on = false;
    }
    // Flush the battery save now that the emulator task is gone and cart
    // RAM is stable.
    if (s_save_size > 0) {
        FILE *sf = fopen(s_save_path, "wb");
        if (sf) {
            size_t put = fwrite(s_cram, 1, s_save_size, sf);
            fclose(sf);
            printf("[GBC] save written: %u bytes to %s\n",
                   (unsigned)put, s_save_path);
        } else {
            printf("[GBC] save write FAILED: cannot open %s\n", s_save_path);
        }
    }
    meck_p4kbd_set_raw_joypad(false);
    if (s_timer) { lv_timer_delete(s_timer); s_timer = NULL; }
    s_rom_size = 0;                   // arena retained (see s_rom comment)
    s_task    = NULL;
    s_running = false;
    g_gbc_canvas = NULL;
    g_gbc_status = NULL;
    if (s_touch_ui) {
        touch_set_poll_period(s_touch_period_restore);
        touch_clear_overlay_refs();
        s_touch_ui = false;
    }
    gbc_show_browser();               // rebuild fresh and load it
    gbc_delete_screen(&scr_gbc);
}

static void gbc_frame_timer_cb(lv_timer_t *t)
{
    (void)t;
    lv_display_trigger_activity(NULL);   // keep the idle watcher away

    if (s_running && (s_stop || meck_p4kbd_raw_exit_pressed())) {
        s_stop = true;
        if (s_task_stopped) gbc_stop_and_leave();
        return;                          // wait for the task to confirm
    }
    if (g_gbc_canvas) lv_obj_invalidate(g_gbc_canvas);
    // Mute toggles: touch pill press edges (counted by the emulator task)
    // and the K270 microphone key latch.
    {
        const uint8_t req = s_touch_mute_req;
        if (req != s_touch_mute_ack) { s_touch_mute_ack = req; gbc_toggle_mute(); }
        if (meck_p4kbd_raw_mute_pressed()) gbc_toggle_mute();
    }
    if (s_touch_ui) touch_update_overlay();
}

static void on_gbc_back(lv_event_t *e)
{
    (void)e;
    // Touch back on the emulator screen behaves like Esc.
    s_stop = true;
}

static void gbc_launch(const char *path)
{
    printf("[GBC] launch %s\n", path);
    if (!gbc_alloc_buffers()) {
        printf("[GBC] buffer allocation failed\n");
        return;
    }

    // Load the ROM into PSRAM.
    FILE *f = fopen(path, "rb");
    if (!f) { printf("[GBC] cannot open %s\n", path); return; }
    fseek(f, 0, SEEK_END);
    long sz = ftell(f);
    fseek(f, 0, SEEK_SET);
    if (sz < 0x8000) { fclose(f); printf("[GBC] file too small\n"); return; }
    if (s_rom && s_rom_cap < (size_t)sz) {
        // Larger ROM than the arena has held so far: grow (rare).
        heap_caps_free(s_rom);
        s_rom = NULL;
        s_rom_cap = 0;
    }
    if (!s_rom) {
        // First allocation: reserve 2 MB (the largest common GB/GBC ROM
        // size) even for a smaller game, so a later 2 MB title never has
        // to grow the arena in a fragmented pool -- growing means freeing
        // this block and hoping a bigger one exists. Fall back to exactly
        // the needed size if 2 MB is not available right now.
        size_t want = (size_t)sz > 0x200000 ? (size_t)sz : 0x200000;
        s_rom = (uint8_t*)heap_caps_malloc(want, MALLOC_CAP_SPIRAM);
        if (!s_rom && want > (size_t)sz) {
            want = (size_t)sz;
            s_rom = (uint8_t*)heap_caps_malloc(want, MALLOC_CAP_SPIRAM);
        }
        if (!s_rom) {
            fclose(f);
            printf("[GBC] ROM alloc failed (%ld bytes; largest free PSRAM block %u, free %u)\n",
                   sz,
                   (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM),
                   (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
            return;
        }
        s_rom_cap = want;
        printf("[GBC] ROM arena reserved: %u bytes\n", (unsigned)want);
    }
    size_t got = fread(s_rom, 1, (size_t)sz, f);
    fclose(f);
    if (got != (size_t)sz) {
        printf("[GBC] short read (%u/%ld)\n", (unsigned)got, sz);
        return;
    }
    s_rom_size = (size_t)sz;
    memset(s_cram, 0, GBC_CRAM_SIZE);

    // Battery saves: raw cart RAM as a .sav sidecar next to the ROM (the
    // de facto emulator standard, portable to desktop emulators). Size
    // comes from the cartridge header's RAM-size code (0x149); MBC2 carts
    // declare 0 there but carry 512 bytes internally. Loaded here so the
    // game wakes up with its battery RAM intact; written back on exit
    // (flush-on-exit by design -- a mid-game power cut loses the session,
    // accepted). No RTC state yet: Pokemon Crystal's clock restarts each
    // boot until MBC3 RTC persistence is added.
    {
        static const size_t ram_sz[6] = { 0, 2048, 8192, 32768, 131072, 65536 };
        const uint8_t code = s_rom[0x149];
        s_save_size = (code < 6) ? ram_sz[code] : 0;
        if (s_save_size > GBC_CRAM_SIZE) s_save_size = GBC_CRAM_SIZE;
        snprintf(s_save_path, sizeof(s_save_path), "%s", path);
        char *dot = strrchr(s_save_path, '.');
        if (dot) snprintf(dot, sizeof(s_save_path) - (dot - s_save_path), ".sav");
        else     s_save_size = 0;   // no extension to swap: skip saves
    }

    enum gb_init_error_e e = gb_init(s_gb, rom_read, cram_read, cram_write,
                                     gb_error_cb, NULL);
    if (e != GB_INIT_NO_ERROR) {
        printf("[GBC] gb_init failed: %d\n", (int)e);
        s_rom_size = 0;               // arena retained
        return;
    }
    if (s_gb->mbc == 2 && s_save_size == 0) s_save_size = 512;
    if (s_save_size > 0) {
        FILE *sf = fopen(s_save_path, "rb");
        if (sf) {
            size_t got_sv = fread(s_cram, 1, s_save_size, sf);
            fclose(sf);
            printf("[GBC] save loaded: %u bytes from %s\n",
                   (unsigned)got_sv, s_save_path);
        } else {
            printf("[GBC] no save file (fresh start)\n");
        }
    }
    gb_init_lcd(s_gb, draw_line);
    memset(s_gbfb, 0, (size_t)GB_H * GB_W * sizeof(uint16_t));
    memset(s_outfb, 0, (size_t)OUT_W * OUT_H * sizeof(uint16_t));
    printf("[GBC] cgbMode=%d rom=%u bytes\n", (int)s_gb->cgb.cgbMode,
           (unsigned)s_rom_size);

    // Emulator screen: black, canvas centred, back button top-left.
    scr_gbc = gbc_make_screen("", on_gbc_back);
    g_gbc_canvas = lv_canvas_create(scr_gbc);
    lv_canvas_set_buffer(g_gbc_canvas, s_outfb, OUT_W, OUT_H,
                         LV_COLOR_FORMAT_RGB565);
    // Touch controls when no keyboard was detected at boot. Portrait: game
    // anchored near the top, controls in the space below. Landscape: game
    // centred, controls in the side margins. Keyboard present: game
    // centred, no overlay.
    {
        const int32_t w = lv_display_get_horizontal_resolution(NULL);
        const int32_t h = lv_display_get_vertical_resolution(NULL);
        const bool portrait = h > w;
        s_touch_ui = !meck_p4kbd_present();
        s_touch_mask = 0;
        s_muted = false;
        s_touch_mute_req = s_touch_mute_ack = 0;
        if (s_touch_ui) {
            touch_layout(w, h, portrait);
            if (portrait) lv_obj_align(g_gbc_canvas, LV_ALIGN_TOP_MID, 0, 80);
            else          lv_obj_center(g_gbc_canvas);
            touch_build_overlay(scr_gbc);
            touch_set_poll_period(16);
            printf("[GBC] touch controls on (%s)\n", portrait ? "portrait" : "landscape");
        } else {
            lv_obj_center(g_gbc_canvas);
        }
    }
    lv_screen_load(scr_gbc);

    // Sound: initialise the audio stack if nothing has yet (it is lazy --
    // MeckAudio brings itself up on the first play, so a game launched
    // before the audiobook player has ever run finds it "not ready"),
    // stop the player if it holds the codec, wake the codec, reclock to
    // the APU's rate, unmute, reset the APU. Only if init itself fails
    // does the game run silent.
    s_audio_on = false;
    if (meck_audio_ready() || meck_audio_init()) {
        MeckAudioState st = meck_audio_get_state();
        if (st == MECK_AUDIO_STATE_PLAYING || st == MECK_AUDIO_STATE_PAUSED ||
            st == MECK_AUDIO_STATE_LOADING) {
            printf("[GBC] stopping audiobook player for game audio\n");
            meck_audio_stop();
        }
        meck_audio_codec_wake();
        if (meck_audio_i2s_reconfig(AUDIO_SAMPLE_RATE, 16, I2S_SLOT_MODE_STEREO) == ESP_OK) {
            // The player's stop path runs its mute callback, which sets
            // the DAC volume register to ZERO; only the player's next play
            // would restore it. Unmute here (re-applies the user's volume).
            meck_audio_set_dac_mute(false);
            audio_init();
            s_audio_on = true;
            printf("[GBC] audio on (%u Hz, %u samples/frame)\n",
                   (unsigned)AUDIO_SAMPLE_RATE, (unsigned)AUDIO_SAMPLES);
        } else {
            meck_audio_codec_sleep();
            printf("[GBC] audio reconfig failed -- running silent\n");
        }
    } else {
        printf("[GBC] audio stack init failed -- running silent\n");
    }

    // Input over to the joypad, then start the machinery.
    meck_p4kbd_set_raw_joypad(true);
    s_stop = false;
    s_task_stopped = false;
    s_running = true;
    s_pace_sem = xSemaphoreCreateBinary();
    {
        esp_timer_create_args_t pa = {};
        pa.callback = gbc_pace_cb;
        pa.name     = "gbc_pace";
        esp_timer_create(&pa, &s_pace_timer);
    }
    s_timer = lv_timer_create(gbc_frame_timer_cb, 20, NULL);
    if (xTaskCreatePinnedToCoreWithCaps(gbc_task, "meck_gbc", 16 * 1024,
                                        NULL, 2, &s_task, 1,
                                        MALLOC_CAP_SPIRAM) != pdPASS) {
        printf("[GBC] task create failed\n");
        s_running = false;
        if (s_touch_ui) { touch_set_poll_period(s_touch_period_restore); touch_clear_overlay_refs(); s_touch_ui = false; }
        if (s_pace_timer) { esp_timer_delete(s_pace_timer); s_pace_timer = NULL; }
        if (s_pace_sem)   { vSemaphoreDelete(s_pace_sem); s_pace_sem = NULL; }
        if (s_audio_on) {
            meck_audio_i2s_reconfig(44100, 16, I2S_SLOT_MODE_STEREO);
            meck_audio_codec_sleep();
            s_audio_on = false;
        }
        meck_p4kbd_set_raw_joypad(false);
        lv_timer_delete(s_timer);
        s_timer = NULL;
        gbc_show_browser();
        gbc_delete_screen(&scr_gbc);
    }
}

// ---- ROM browser ------------------------------------------------------------
static char s_rom_names[GBC_MAX_ROMS][GBC_NAME_MAX];

static void on_rom_row(lv_event_t *e)
{
    const int idx = (int)(intptr_t)lv_event_get_user_data(e);
    if (idx < 0 || idx >= GBC_MAX_ROMS) return;
    char path[192];
    snprintf(path, sizeof(path), GBC_ROM_DIR "/%s", s_rom_names[idx]);
    gbc_launch(path);
}

static void on_browser_back(lv_event_t *e)
{
    (void)e;
    gbc_build_and_load_menu();
    gbc_delete_screen(&scr_rom_browser);
}

static void gbc_show_browser(void)
{
    gbc_seed_sd();

    scr_rom_browser = gbc_make_screen("GBC ROMs", on_browser_back);

    // Scrollable list container below the header.
    lv_obj_t *list = lv_obj_create(scr_rom_browser);
    const int32_t w = lv_display_get_horizontal_resolution(NULL);
    const int32_t h = lv_display_get_vertical_resolution(NULL);
    lv_obj_set_size(list, w - 20, h - 100);
    lv_obj_set_pos(list, 10, 90);
    lv_obj_set_style_bg_opa(list, 0, 0);
    lv_obj_set_style_border_width(list, 0, 0);
    lv_obj_set_style_pad_all(list, 0, 0);
    lv_obj_set_scroll_dir(list, LV_DIR_VER);

    int count = 0;
    DIR *d = opendir(GBC_ROM_DIR);
    if (d) {
        struct dirent *ent;
        while ((ent = readdir(d)) != NULL && count < GBC_MAX_ROMS) {
            const char *n = ent->d_name;
            const size_t len = strlen(n);
            if (len >= GBC_NAME_MAX) continue;
            // Skip macOS AppleDouble metadata ("._Name.gbc"): created
            // automatically when files are copied to FAT cards from a Mac,
            // and not ROMs.
            if (n[0] == '.' && n[1] == '_') continue;
            // Both cartridge formats: .gbc (Game Boy Color) and .gb (the
            // original DMG Game Boy -- the core runs those too, with the
            // GBC's built-in colourisation of DMG titles).
            bool is_rom = false;
            if (len >= 5 && strcasecmp(n + len - 4, ".gbc") == 0) is_rom = true;
            else if (len >= 4 && strcasecmp(n + len - 3, ".gb") == 0) is_rom = true;
            if (!is_rom) continue;
            snprintf(s_rom_names[count], GBC_NAME_MAX, "%s", n);
            gbc_make_row(list, n, count * 70, on_rom_row,
                         (void*)(intptr_t)count);
            count++;
        }
        closedir(d);
    }

    if (count == 0) {
        lv_obj_t *lbl = lv_label_create(list);
        lv_label_set_text(lbl,
            "No .gb / .gbc ROMs found.\n\nPut ROM files in /roms on the SD card.");
        lv_obj_set_style_text_color(lbl, lv_color_make(160, 160, 160), 0);
        meck_ui_set_font(lbl, &meck_montserrat_18, 0);
        lv_obj_align(lbl, LV_ALIGN_TOP_LEFT, 10, 10);
    }

    lv_screen_load(scr_rom_browser);
}

// ---- Games menu -------------------------------------------------------------
static void on_menu_back(lv_event_t *e)
{
    (void)e;
    if (g_return_screen) lv_screen_load(g_return_screen);
    gbc_delete_screen(&scr_games_menu);
}

static void on_menu_gbc_row(lv_event_t *e)
{
    (void)e;
    gbc_show_browser();
    gbc_delete_screen(&scr_games_menu);
}

// Build and load the Games menu. Does NOT touch g_return_screen: the
// return target is captured exactly once, on entry from outside the module
// (meck_gbc_show_menu), and must survive internal menu<->browser hops.
// Recapturing it here would record the about-to-be-deleted browser, and
// the next menu-back would lv_screen_load a freed object -- the load-fault
// crash seen on hardware after Esc-Esc-Esc out of a game.
static void gbc_build_and_load_menu(void)
{
    scr_games_menu = gbc_make_screen("Games", on_menu_back);
    gbc_make_row(scr_games_menu, LV_SYMBOL_KEYBOARD "  GBC Emulator", 100,
                 on_menu_gbc_row, NULL);
    // Future rows (Snake, Minesweeper) slot in below at y = 100 + n*70.
    lv_screen_load(scr_games_menu);
}

extern "C" void meck_gbc_show_menu(void)
{
    if (s_running) return;   // never re-enter over a live emulator
    g_return_screen = lv_screen_active();
    gbc_build_and_load_menu();
}

extern "C" bool meck_gbc_running(void)
{
    return s_running;
}