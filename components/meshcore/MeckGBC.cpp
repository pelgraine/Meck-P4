/*
 * MeckGBC.cpp -- Game Boy Color emulator (Peanut-GB gbc-rtc-fix core)
 * ----------------------------------------------------------------------------
 * v1 scope (as agreed):
 *   - Keyboard (K270) builds only. Controls come from MeckP4Keyboard's raw
 *     joypad mode (arrows = d-pad, K = A, J = B, Enter = Start,
 *     Space = Select, Esc = exit to the browser).
 *   - Silent: ENABLE_SOUND 0. Sound is a later stage.
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
 *     59.73 Hz (16742 us/frame) against esp_timer, yielding in 1-tick
 *     slices so the idle task is always fed.
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
 *   - Frame timing note: FreeRTOS ticks are 10 ms on this build, so the
 *     pacing loop has up to one tick of per-frame jitter; the accumulated
 *     deadline keeps the average rate exact. Fine for silent operation;
 *     the audio clock becomes the pacer when sound lands.
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

#if defined(CONFIG_BOARD_TYPE_T_DISPLAY_P4_KEYBOARD)

#include "lvgl.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/idf_additions.h"   // xTaskCreatePinnedToCoreWithCaps
#include "esp_heap_caps.h"
#include "esp_timer.h"

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
#define PEANUT_FULL_GBC_SUPPORT 1
#define ENABLE_LCD 1
#define ENABLE_SOUND 0
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wsequence-point"
#pragma GCC diagnostic ignored "-Wmisleading-indentation"
#include "peanut_gb.h"
#pragma GCC diagnostic pop

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
static uint8_t         *s_rom     = NULL;          // PSRAM copy of the ROM
static size_t           s_rom_size = 0;
static uint8_t         *s_cram    = NULL;          // 128 KB, PSRAM
static size_t           s_save_size = 0;      // battery-save bytes (0 = none)
static char             s_save_path[192];
static volatile bool    s_stop         = false;
static volatile bool    s_task_stopped = false;
static bool             s_running      = false;
static TaskHandle_t     s_task    = NULL;
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
static void gbc_task(void *arg)
{
    (void)arg;
    printf("[GBC] task start (core %d)\n", (int)xPortGetCoreID());
    int64_t next = esp_timer_get_time();
    while (!s_stop) {
        // Held keys -> joypad. Peanut-GB's register is active-low.
        s_gb->direct.joypad = (uint8_t)~meck_p4kbd_raw_joypad();
        gb_run_frame(s_gb);
        blit_scaled();
        next += GBC_FRAME_US;
        int64_t now = esp_timer_get_time();
        if (next < now) next = now;   // fell behind: resync, don't spiral
        while (!s_stop && esp_timer_get_time() < next) vTaskDelay(1);
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
    // Name the failing buffer: "buffer allocation failed" alone cost a
    // debugging round trip when the old internal-RAM LUT was the culprit.
    if (!s_outfb) printf("[GBC] outfb alloc failed\n");
    if (!s_gb)    printf("[GBC] gb_s alloc failed\n");
    if (!s_gbfb)  printf("[GBC] gbfb alloc failed\n");
    if (!s_cram)  printf("[GBC] cram alloc failed\n");
    return s_outfb && s_cram && s_gb && s_gbfb;
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
    if (s_rom)   { heap_caps_free(s_rom); s_rom = NULL; s_rom_size = 0; }
    s_task    = NULL;
    s_running = false;
    g_gbc_canvas = NULL;
    g_gbc_status = NULL;
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
    uint8_t *rom = (uint8_t*)heap_caps_malloc((size_t)sz, MALLOC_CAP_SPIRAM);
    if (!rom) { fclose(f); printf("[GBC] ROM alloc failed (%ld bytes)\n", sz); return; }
    size_t got = fread(rom, 1, (size_t)sz, f);
    fclose(f);
    if (got != (size_t)sz) {
        heap_caps_free(rom);
        printf("[GBC] short read (%u/%ld)\n", (unsigned)got, sz);
        return;
    }
    s_rom = rom;
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
        const uint8_t code = rom[0x149];
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
        heap_caps_free(s_rom);
        s_rom = NULL;
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
    lv_obj_center(g_gbc_canvas);
    lv_screen_load(scr_gbc);

    // Input over to the joypad, then start the machinery.
    meck_p4kbd_set_raw_joypad(true);
    s_stop = false;
    s_task_stopped = false;
    s_running = true;
    s_timer = lv_timer_create(gbc_frame_timer_cb, 20, NULL);
    if (xTaskCreatePinnedToCoreWithCaps(gbc_task, "meck_gbc", 16 * 1024,
                                        NULL, 2, &s_task, 1,
                                        MALLOC_CAP_SPIRAM) != pdPASS) {
        printf("[GBC] task create failed\n");
        s_running = false;
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

#else  // not CONFIG_BOARD_TYPE_T_DISPLAY_P4_KEYBOARD ------------------------

// Plain-board stubs: the Games tile keeps its placeholder on this build
// (see cb_todo_games in MeckUI.cpp), so these are never reached, but they
// keep the link happy if that ever changes.
extern "C" void meck_gbc_show_menu(void) {}
extern "C" bool meck_gbc_running(void) { return false; }

#endif // CONFIG_BOARD_TYPE_T_DISPLAY_P4_KEYBOARD