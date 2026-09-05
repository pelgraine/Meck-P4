/*
 * gbc_bench_test -- Game Boy Color emulator CPU feasibility probe
 * ----------------------------------------------------------------------------
 * Purpose. Establish whether the ESP32-P4 can run a Game Boy Color game at
 * full speed (59.73 fps) using the Peanut-GB core with GBC support (tvecera's
 * gbc-rtc-fix branch, the basis of Peanut-GB PR #93 and of pico-peanutGB,
 * which plays GBC games on an RP2350). No display, no input, no sound -- the
 * serial log carries the whole result.
 *
 * Workload. The embedded ROM is ucity v1.3 (open-source GBC city builder,
 * GPL-3.0, github.com/AntonioND/ucity). Its cartridge header declares
 * CGB-only (0xC0) and MBC5+RAM+BATTERY with 128 KB cart RAM, so it exercises
 * the GBC colour path and a banked mapper. It runs unattended at its title
 * screen; real gameplay may cost somewhat more per frame.
 *
 * Phases. Each phase runs 3 reps of 600 frames and prints fps per rep:
 *   PHASE A: core only. lcd_draw_line writes each 160-px scanline into a
 *            160x144 RGB555 buffer in internal SRAM (the fixPalette lookup
 *            the SDL example uses). Measures raw emulation speed.
 *   PHASE B: adds the display-shaped work: per frame, convert RGB555 to
 *            RGB565 through a 64 KB LUT and 3x-scale into a 480x432 buffer
 *            in PSRAM. Approximates what feeding the panel would cost,
 *            without any panel involved.
 *
 * How to read it. Full speed needs >= 59.73 fps in PHASE B. Well above that
 * means headroom for sound (minigb_apu) and the real blit path; well below
 * means the emulator idea dies here, cheaply.
 *
 * Memory placement (mirrors what a real integration would do):
 *   struct gb_s (~50 KB) and the 160x144 line buffer: static internal SRAM.
 *   ROM copy, 128 KB cart RAM, 480x432 output buffer: PSRAM.
 *   RGB555->RGB565 LUT (64 KB): internal SRAM heap.
 *
 * Watchdog. Frames run in chunks of 100 with a 1-tick untimed yield between
 * chunks, so the idle task is fed and the task WDT stays quiet; the yield is
 * outside the timed windows and does not pollute the measurement.
 *
 * Build: menuconfig -> Example Configuration -> gbc_bench_test.
 */

#include <stdio.h>
#include <string.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"

#define PEANUT_FULL_GBC_SUPPORT 1
#define ENABLE_LCD 1
#define ENABLE_SOUND 0
// The vendored core trips two warning classes that this tree's -Werror=all
// promotes to errors: -Wsequence-point on its "x = (++x) & 0x3F" palette
// auto-increment lines (well-defined under the C++17-and-later sequencing
// this build uses, gnu++2b) and -Wmisleading-indentation. Both are cosmetic
// here, and suppressing them keeps peanut_gb.h byte-identical to upstream.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wsequence-point"
#pragma GCC diagnostic ignored "-Wmisleading-indentation"
#include "peanut_gb.h"
#pragma GCC diagnostic pop

// Embedded ROM blob (see main/CMakeLists.txt: target_add_binary_data).
extern const uint8_t ucity_rom_start[] asm("_binary_ucity_rom_gbc_start");
extern const uint8_t ucity_rom_end[]   asm("_binary_ucity_rom_gbc_end");

// GBC LCD geometry.
#define GB_W 160
#define GB_H 144

// Scanline target: RGB555 straight from gb->cgb.fixPalette, internal SRAM.
static uint16_t s_gbfb[GB_H][GB_W];

// Phase B output: 3x integer scale, RGB565, PSRAM.
#define SCALE 3
#define OUT_W (GB_W * SCALE)
#define OUT_H (GB_H * SCALE)
static uint16_t *s_outfb = NULL;

// RGB555 -> RGB565 lookup table, internal SRAM heap. fixPalette entries are
// RGB555 with red in bits 14-10 (peanut_gb.h swaps the GBC's native red/blue
// order when latching palette writes -- see the "swap Red and Blue" lines in
// its BCPD/OCPD handlers).
static uint16_t *s_lut555 = NULL;

// Emulator state (~50 KB with GBC enabled): static internal SRAM.
static struct gb_s s_gb;

// Cart RAM: ucity's header RAM size code 0x04 = 128 KB. PSRAM.
#define CRAM_SIZE 0x20000
static uint8_t *s_cram = NULL;

// ROM: copied from the embedded blob into PSRAM, matching how SD-loaded
// ROMs would be held in a real integration.
static uint8_t *s_rom = NULL;
static size_t   s_rom_size = 0;

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
    printf("gbcbench: GB CORE ERROR %d at 0x%04X -- parking\n", (int)err, (unsigned)addr);
    for (;;) vTaskDelay(pdMS_TO_TICKS(10000));
}

static void draw_line(struct gb_s *gb, const uint8_t *pixels, const uint_fast8_t line)
{
    uint16_t *dst = s_gbfb[line];
    for (int x = 0; x < GB_W; x++)
        dst[x] = gb->cgb.fixPalette[pixels[x]];
}

// Phase B per-frame work: LUT-convert and 3x-scale the whole frame into the
// PSRAM buffer. Pixels are tripled in the row; the finished row is then
// memcpy'd twice for the vertical tripling.
static void blit_scaled(void)
{
    for (int y = 0; y < GB_H; y++) {
        uint16_t       *row0 = &s_outfb[(size_t)(y * SCALE) * OUT_W];
        const uint16_t *src  = s_gbfb[y];
        uint16_t       *d    = row0;
        for (int x = 0; x < GB_W; x++) {
            uint16_t c = s_lut555[src[x] & 0x7FFF];
            d[0] = c;
            d[1] = c;
            d[2] = c;
            d += SCALE;
        }
        memcpy(row0 + OUT_W,     row0, OUT_W * sizeof(uint16_t));
        memcpy(row0 + 2 * OUT_W, row0, OUT_W * sizeof(uint16_t));
    }
}

// One timed rep of `frames` frames. Chunked with an untimed 1-tick yield so
// the idle task (and thus the task watchdog) is serviced. Returns emulation
// time in microseconds, yields excluded.
static int64_t run_rep(int frames, bool with_blit)
{
    const int chunk    = 100;
    int64_t   total_us = 0;
    int       done     = 0;
    while (done < frames) {
        int n = frames - done;
        if (n > chunk) n = chunk;
        int64_t t0 = esp_timer_get_time();
        for (int i = 0; i < n; i++) {
            gb_run_frame(&s_gb);
            if (with_blit) blit_scaled();
        }
        total_us += esp_timer_get_time() - t0;
        done += n;
        vTaskDelay(1);
    }
    return total_us;
}

static void report(const char *label, int rep, int frames, int64_t us)
{
    double fps = (double)frames * 1000000.0 / (double)us;
    double pct = fps * 100.0 / 59.73;
    printf("gbcbench: %s rep %d: %d frames in %lu us -> %.2f fps (%.1f%% of real-time 59.73)\n",
           label, rep, frames, (unsigned long)us, fps, pct);
}

extern "C" void app_main(void)
{
    printf("\ngbcbench: GBC emulator CPU feasibility probe (Peanut-GB gbc-rtc-fix core)\n");
    printf("gbcbench: sizeof(struct gb_s) = %u bytes\n", (unsigned)sizeof(struct gb_s));

    s_rom_size = (size_t)(ucity_rom_end - ucity_rom_start);
    printf("gbcbench: embedded ROM size = %u bytes\n", (unsigned)s_rom_size);

    s_rom    = (uint8_t*)heap_caps_malloc(s_rom_size, MALLOC_CAP_SPIRAM);
    s_cram   = (uint8_t*)heap_caps_malloc(CRAM_SIZE, MALLOC_CAP_SPIRAM);
    s_outfb  = (uint16_t*)heap_caps_malloc((size_t)OUT_W * OUT_H * sizeof(uint16_t),
                                           MALLOC_CAP_SPIRAM);
    s_lut555 = (uint16_t*)heap_caps_malloc(32768 * sizeof(uint16_t), MALLOC_CAP_INTERNAL);
    if (!s_rom || !s_cram || !s_outfb || !s_lut555) {
        printf("gbcbench: allocation failed (rom=%p cram=%p outfb=%p lut=%p) -- parking\n",
               (void*)s_rom, (void*)s_cram, (void*)s_outfb, (void*)s_lut555);
        for (;;) vTaskDelay(pdMS_TO_TICKS(10000));
    }
    memcpy(s_rom, ucity_rom_start, s_rom_size);
    memset(s_cram, 0, CRAM_SIZE);

    // Build the RGB555 -> RGB565 table: red and blue keep their 5 bits;
    // green widens to 6 by (g << 1) | (g >> 4).
    for (uint32_t c = 0; c < 32768; c++) {
        uint32_t r = (c >> 10) & 0x1F;
        uint32_t g = (c >> 5) & 0x1F;
        uint32_t b = c & 0x1F;
        s_lut555[c] = (uint16_t)((r << 11) | (((g << 1) | (g >> 4)) << 5) | b);
    }

    enum gb_init_error_e e = gb_init(&s_gb, rom_read, cram_read, cram_write, gb_error_cb, NULL);
    if (e != GB_INIT_NO_ERROR) {
        printf("gbcbench: gb_init failed: %d -- parking\n", (int)e);
        for (;;) vTaskDelay(pdMS_TO_TICKS(10000));
    }
    gb_init_lcd(&s_gb, draw_line);

    gb_run_frame(&s_gb);
    printf("gbcbench: first frame ran, cgbMode=%d (1 = GBC colour path active)\n",
           (int)s_gb.cgb.cgbMode);
    printf("gbcbench: free heap after setup: internal=%u PSRAM=%u\n",
           (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
           (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));

    const int FRAMES = 600;

    printf("\ngbcbench: PHASE A -- core only (scanlines into 160x144 internal-SRAM buffer)\n");
    for (int rep = 1; rep <= 3; rep++)
        report("phase A", rep, FRAMES, run_rep(FRAMES, false));

    printf("\ngbcbench: PHASE B -- core + per-frame RGB555->RGB565 convert + 3x scale into 480x432 PSRAM buffer\n");
    for (int rep = 1; rep <= 3; rep++)
        report("phase B", rep, FRAMES, run_rep(FRAMES, true));

    printf("\ngbcbench: done. Full speed requires >= 59.73 fps in PHASE B.\n");
    printf("gbcbench: workload note: ucity idles at its title screen; gameplay may cost more per frame.\n");
    for (;;) vTaskDelay(pdMS_TO_TICKS(10000));
}