/*
 * warmhandoff_test -- RM69A10 rebuild SHIFT-LOCATION probe
 * ----------------------------------------------------------------------------
 * Background. The duty-cycled DPI mechanism is proven on hardware: deleting
 * only the DPI panel keeps the last frame lit (GRAM self-refresh), releases the
 * dsi_dpi CPU_FREQ_MAX lock, and the panel can be rebuilt on the still-live DSI
 * bus repeatedly with no hang. That is now live in the firmware and the CPU
 * measurably sits at 40 MHz while the screen is off.
 *
 * The open problem: after a wake (panel rebuild), the whole UI is drawn in the
 * wrong place -- every screen, both orientations, and navigating does not fix
 * it. Two hypotheses were already eliminated by reading the code:
 *   - LVGL is NOT drawing into the panel framebuffer (its buffer is a separate
 *     heap_caps_malloc), so a changed framebuffer address cannot explain it.
 *   - The panel's own window registers (CASET/RASET/PTLAR) are re-sent
 *     identically by panel_rm69a10_init on every rebuild.
 * A panel hardware reset before the rebuild was tried in the live firmware and
 * did NOT fix it.
 *
 * Remaining lead: an underrun ("can't fetch data from external memory fast
 * enough") fires on EVERY rebuild -- on the bench and in the live firmware, at
 * 360 MHz, and even with a CPU_FREQ_MAX lock held across the rebuild. It never
 * fires on the initial boot bring-up. If that underrun desynchronises the DPI
 * scan against the frame start, every subsequent frame would be displaced by a
 * fixed amount, which matches the symptom exactly.
 *
 * Why earlier tests missed it: they drew full-width colour bars. A horizontal
 * shift is invisible in uniform horizontal stripes, so three "clean" runs told
 * us nothing about position.
 *
 * WHAT THIS TEST ANSWERS. It draws a hard-edged POSITIONED marker with raw
 * esp_lcd_panel_draw_bitmap and no LVGL anywhere, then deletes and rebuilds the
 * panel twice, redrawing the identical marker each time.
 *
 *   - If the marker is DISPLACED after a rebuild, the shift is at the
 *     panel/stream level. LVGL is innocent and the fix belongs in the rebuild.
 *   - If the marker stays EXACTLY in place after every rebuild, the panel is
 *     fine and the shift in the live firmware comes from the LVGL layer, so
 *     that is where to look next.
 *
 * It also distinguishes two panel-level causes: the marker is redrawn after
 * each of two rebuilds, so if the displacement GROWS with each rebuild it is
 * cumulative (consistent with underrun desync), whereas a constant offset that
 * does not grow points to a fixed structural difference between the first
 * panel and a rebuilt one.
 *
 * The rebuild path mirrors the live firmware's wake path, including the panel
 * hardware reset pulse, so the test reproduces the real condition.
 *
 * THE MARKER (identical every draw, on a black field):
 *   - an 8 px WHITE border hugging all four screen edges
 *   - 64 px corner squares just inside it: RED top-left, GREEN top-right,
 *     BLUE bottom-left, YELLOW bottom-right
 *   - a white cross through the centre (both axes, so vertical and horizontal
 *     displacement are both obvious)
 *   - a small CYAN cycle block whose horizontal position steps each cycle, so
 *     you can confirm the redraw actually happened
 *
 * WHAT TO LOOK FOR at each OBSERVE: does the white border still touch all four
 * edges evenly, and are all four corner squares fully on screen? If the border
 * runs off one edge, or a gap of black appears along the opposite edge, that is
 * the shift -- note WHICH WAY and ROUGHLY HOW FAR.
 *
 * Colour fidelity does not matter here. Position is the entire measurement.
 *
 * RM69A10 AMOLED build only (568 x 1232). No radio is used.
 */

#include <stdio.h>
#include <string.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_err.h"
#include "esp_task_wdt.h"
#include "esp_heap_caps.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_mipi_dsi.h"
#include "t_display_p4_config.h"
#include "cpp_bus_driver_library.h"
#include "t_display_p4_driver.h"

#if !defined CONFIG_SCREEN_TYPE_RM69A10
#error "warmhandoff_test targets the RM69A10 AMOLED variant only"
#endif
#if defined CONFIG_SCREEN_PIXEL_FORMAT_RGB565
#define BPP 2
#else
#define BPP 3
#endif

#define STRIP_ROWS 16
#define BORDER_PX 8
#define CORNER_PX 64
#define CROSS_PX 8
#define CYCLE_BLK 48

static esp_lcd_panel_handle_t s_dpi_panel = NULL;
static esp_lcd_panel_io_handle_t s_dbi = NULL;

static auto IIC_Bus_0 = std::make_shared<Cpp_Bus_Driver::Hardware_Iic_1>(XL9535_SDA, XL9535_SCL, I2C_NUM_0);
static auto XL9535 = std::make_unique<Cpp_Bus_Driver::Xl95x5>(IIC_Bus_0, XL9535_IIC_ADDRESS, DEFAULT_CPP_BUS_DRIVER_VALUE);

// ------------- monitored read / del machinery -------------
struct Job { uint8_t cmd; uint8_t *rbuf; size_t n; esp_err_t err; };
static volatile Job s_job;
static SemaphoreHandle_t s_go, s_done;

static void io_worker(void *arg)
{
    (void)arg;
    for (;;) {
        xSemaphoreTake(s_go, portMAX_DELAY);
        Job j = *(Job *)&s_job;
        s_job.err = esp_lcd_panel_io_rx_param(s_dbi, j.cmd, j.rbuf, j.n);
        xSemaphoreGive(s_done);
    }
}

static bool mread(uint8_t cmd, uint8_t *buf, size_t n, uint32_t to_ms)
{
    memset(buf, 0xEE, n);
    s_job.cmd = cmd; s_job.rbuf = buf; s_job.n = n;
    xSemaphoreGive(s_go);
    if (xSemaphoreTake(s_done, pdMS_TO_TICKS(to_ms)) == pdTRUE) return s_job.err == ESP_OK;
    printf("warmhandoff: !! READ WEDGED cmd=0x%02X\n", cmd);
    xTaskCreate(io_worker, "iow", 4096, NULL, 5, NULL);
    return false;
}

static void del_worker(void *arg)
{
    esp_lcd_panel_del((esp_lcd_panel_handle_t)arg);
    xSemaphoreGive(s_done);
    vTaskDelete(NULL);
}

static bool monitored_del(esp_lcd_panel_handle_t p, uint32_t to_ms)
{
    xTaskCreate(del_worker, "delw", 4096, (void *)p, 5, NULL);
    if (xSemaphoreTake(s_done, pdMS_TO_TICKS(to_ms)) == pdTRUE) return true;
    printf("warmhandoff: !! dpi_panel_del WEDGED\n");
    return false;
}

static void interrogate(const char *when)
{
    uint8_t v[3];
    printf("warmhandoff: ===== %s =====\n", when);
    if (mread(0x0A, v, 1, 1500))
        printf("warmhandoff: RDDPM  (0x0A)=%02X  PTL=%d SLPOUT=%d NOR=%d DISPON=%d\n",
               v[0], (v[0]>>5)&1, (v[0]>>4)&1, (v[0]>>3)&1, (v[0]>>2)&1);
    if (mread(0x52, v, 1, 1500))
        printf("warmhandoff: brightness (0x52)=%02X\n", v[0]);
}

// ------------- the positioned marker -------------
// Identical geometry on every draw. Only the cyan cycle block moves, so a
// redraw is distinguishable from a stale retained frame.
static inline uint32_t marker_pixel(int x, int y, int cycle)
{
    const int W = SCREEN_WIDTH, H = SCREEN_HEIGHT;

    // border hugging all four edges
    if (x < BORDER_PX || x >= W - BORDER_PX || y < BORDER_PX || y >= H - BORDER_PX)
        return 0xFFFFFF;

    // corner squares just inside the border
    int ci = BORDER_PX, co = BORDER_PX + CORNER_PX;
    if (x >= ci && x < co && y >= ci && y < co)                     return 0xFF0000;  // TL red
    if (x >= W - co && x < W - ci && y >= ci && y < co)             return 0x00FF00;  // TR green
    if (x >= ci && x < co && y >= H - co && y < H - ci)             return 0x0000FF;  // BL blue
    if (x >= W - co && x < W - ci && y >= H - co && y < H - ci)     return 0xFFFF00;  // BR yellow

    // centre cross
    int cx = W / 2, cy = H / 2;
    if (x >= cx - CROSS_PX && x < cx + CROSS_PX) return 0xFFFFFF;
    if (y >= cy - CROSS_PX && y < cy + CROSS_PX) return 0xFFFFFF;

    // cycle indicator: steps right each cycle, sits above the centre
    int bx = (W / 4) + cycle * (W / 5);
    int by = H / 4;
    if (x >= bx && x < bx + CYCLE_BLK && y >= by && y < by + CYCLE_BLK) return 0x00FFFF;

    return 0x000000;
}

static void draw_marker(int cycle)
{
    size_t px_per_strip = (size_t)SCREEN_WIDTH * STRIP_ROWS;
    uint8_t *buf = (uint8_t *)heap_caps_malloc(px_per_strip * BPP, MALLOC_CAP_DMA);
    if (!buf) { printf("warmhandoff: marker malloc failed\n"); return; }

    for (int y = 0; y < SCREEN_HEIGHT; y += STRIP_ROWS) {
        int h = (y + STRIP_ROWS <= SCREEN_HEIGHT) ? STRIP_ROWS : (SCREEN_HEIGHT - y);
        for (int r = 0; r < h; r++) {
            int ay = y + r;
#if BPP == 2
            uint16_t *row = (uint16_t *)(buf + (size_t)r * SCREEN_WIDTH * BPP);
            for (int x = 0; x < SCREEN_WIDTH; x++) {
                uint32_t rgb = marker_pixel(x, ay, cycle);
                row[x] = (uint16_t)(((rgb >> 8) & 0xF800) | ((rgb >> 5) & 0x07E0) | ((rgb >> 3) & 0x001F));
            }
#else
            uint8_t *row = buf + (size_t)r * SCREEN_WIDTH * BPP;
            for (int x = 0; x < SCREEN_WIDTH; x++) {
                uint32_t rgb = marker_pixel(x, ay, cycle);
                row[x * 3] = rgb >> 16; row[x * 3 + 1] = rgb >> 8; row[x * 3 + 2] = rgb;
            }
#endif
        }
        esp_lcd_panel_draw_bitmap(s_dpi_panel, 0, y, SCREEN_WIDTH, y + h, buf);
    }
    free(buf);
}

// ------------- rebuild on the EXISTING bus (mirrors the live wake path) ------
static bool rebuild_dpi_panel(esp_lcd_panel_handle_t *out)
{
    esp_lcd_dsi_bus_handle_t bus = Screen_Get_Mipi_Dsi_Bus_Handle();
    if (!bus) { printf("warmhandoff: no DSI bus handle for rebuild\n"); return false; }
    esp_lcd_dpi_panel_config_t dpi_config = {
        .virtual_channel = 0,
        .dpi_clk_src = MIPI_DSI_DPI_CLK_SRC_DEFAULT,
        .dpi_clock_freq_mhz = SCREEN_MIPI_DSI_DPI_CLK_MHZ,
        .pixel_format = SCREEN_COLOR_RGB_PIXEL_FORMAT,
        .num_fbs = 0,
        .video_timing = {
            .h_size = SCREEN_WIDTH,
            .v_size = SCREEN_HEIGHT,
            .hsync_pulse_width = SCREEN_MIPI_DSI_HSYNC,
            .hsync_back_porch = SCREEN_MIPI_DSI_HBP,
            .hsync_front_porch = SCREEN_MIPI_DSI_HFP,
            .vsync_pulse_width = SCREEN_MIPI_DSI_VSYNC,
            .vsync_back_porch = SCREEN_MIPI_DSI_VBP,
            .vsync_front_porch = SCREEN_MIPI_DSI_VFP,
        },
        .flags = {
            .use_dma2d = true,
        }};
    rm69a10_vendor_config_t vendor_config = {
        .mipi_config = {
            .dsi_bus = bus,
            .dpi_config = &dpi_config,
        },
    };
    esp_lcd_panel_dev_config_t dev_config = {
        .reset_gpio_num = -1,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
        .bits_per_pixel = SCREEN_BITS_PER_PIXEL,
        .vendor_config = &vendor_config,
    };
    esp_err_t e = esp_lcd_new_panel_rm69a10(s_dbi, &dev_config, out);
    if (e != ESP_OK) { printf("warmhandoff: rebuild FAILED (%#X)\n", e); return false; }
    return true;
}

static void panel_hw_reset(void)
{
    XL9535->pin_write(XL9535_SCREEN_RST, Cpp_Bus_Driver::Xl95x5::Value::LOW);
    vTaskDelay(pdMS_TO_TICKS(200));
    XL9535->pin_write(XL9535_SCREEN_RST, Cpp_Bus_Driver::Xl95x5::Value::HIGH);
    vTaskDelay(pdMS_TO_TICKS(200));
}

static void init_and_draw(int cycle, const char *tag)
{
    ESP_ERROR_CHECK(esp_lcd_panel_init(s_dpi_panel));
    set_rm69a10_brightness(s_dpi_panel, 0xFF);
    draw_marker(cycle);
    vTaskDelay(pdMS_TO_TICKS(500));
    printf("warmhandoff: %s up, marker drawn (cycle %d)\n", tag, cycle);
}

static void delete_panel(int n)
{
    printf("\nwarmhandoff: deleting panel #%d (DSI bus kept)...\n", n);
    if (!monitored_del(s_dpi_panel, 5000)) {
        printf("warmhandoff: del #%d WEDGED. Parking.\n", n);
        for (;;) vTaskDelay(pdMS_TO_TICKS(10000));
    }
    s_dpi_panel = NULL;
}

extern "C" void app_main(void)
{
    printf("warmhandoff_test: RM69A10 rebuild SHIFT-LOCATION probe (%dx%d)\n", SCREEN_WIDTH, SCREEN_HEIGHT);
    esp_task_wdt_deinit();
    s_go = xSemaphoreCreateBinary(); s_done = xSemaphoreCreateBinary();
    xTaskCreate(io_worker, "iow", 4096, NULL, 5, NULL);

    // ---- rails + panel reset (proven bring-up) ----
    XL9535->begin();
    XL9535->pin_mode(XL9535_ESP32P4_VCCA_POWER_EN, Cpp_Bus_Driver::Xl95x5::Mode::OUTPUT);
    XL9535->pin_mode(XL9535_5_0_V_POWER_EN, Cpp_Bus_Driver::Xl95x5::Mode::OUTPUT);
    XL9535->pin_mode(XL9535_3_3_V_POWER_EN, Cpp_Bus_Driver::Xl95x5::Mode::OUTPUT);
    XL9535->pin_mode(XL9535_GPS_WAKE_UP, Cpp_Bus_Driver::Xl95x5::Mode::OUTPUT);
    XL9535->pin_write(XL9535_GPS_WAKE_UP, Cpp_Bus_Driver::Xl95x5::Value::LOW);
    XL9535->pin_mode(XL9535_ESP32C6_EN, Cpp_Bus_Driver::Xl95x5::Mode::OUTPUT);
    XL9535->pin_write(XL9535_ESP32C6_EN, Cpp_Bus_Driver::Xl95x5::Value::LOW);
    XL9535->pin_write(XL9535_ESP32P4_VCCA_POWER_EN, Cpp_Bus_Driver::Xl95x5::Value::LOW);
    XL9535->pin_write(XL9535_5_0_V_POWER_EN, Cpp_Bus_Driver::Xl95x5::Value::HIGH);
    vTaskDelay(pdMS_TO_TICKS(200));
    XL9535->pin_write(XL9535_5_0_V_POWER_EN, Cpp_Bus_Driver::Xl95x5::Value::LOW);
    vTaskDelay(pdMS_TO_TICKS(200));
    XL9535->pin_write(XL9535_5_0_V_POWER_EN, Cpp_Bus_Driver::Xl95x5::Value::HIGH);
    XL9535->pin_write(XL9535_3_3_V_POWER_EN, Cpp_Bus_Driver::Xl95x5::Value::LOW);
    vTaskDelay(pdMS_TO_TICKS(200));
    XL9535->pin_write(XL9535_3_3_V_POWER_EN, Cpp_Bus_Driver::Xl95x5::Value::HIGH);
    vTaskDelay(pdMS_TO_TICKS(200));
    XL9535->pin_write(XL9535_3_3_V_POWER_EN, Cpp_Bus_Driver::Xl95x5::Value::LOW);
    vTaskDelay(pdMS_TO_TICKS(200));
    Init_Ldo_Channel_Power(3, 1830);
    vTaskDelay(pdMS_TO_TICKS(100));
    XL9535->pin_mode(XL9535_SCREEN_RST, Cpp_Bus_Driver::Xl95x5::Mode::OUTPUT);
    panel_hw_reset();

    // ---- panel #1: the reference. This is the boot path, known-good. ----
    if (!Screen_Init(&s_dpi_panel)) { printf("warmhandoff: Screen_Init FAILED\n"); return; }
    esp_lcd_dbi_io_config_t dbi_cfg = { .virtual_channel = 0, .lcd_cmd_bits = 8, .lcd_param_bits = 8 };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_dbi(Screen_Get_Mipi_Dsi_Bus_Handle(), &dbi_cfg, &s_dbi));
    init_and_draw(0, "panel #1 (BOOT PATH, reference)");
    interrogate("STATION 1: panel #1, boot path");
    printf("warmhandoff: OBSERVE (12 s): REFERENCE FRAME.\n");
    printf("warmhandoff:   Border should touch all 4 edges evenly, all 4 corner\n");
    printf("warmhandoff:   squares fully visible, cross centred. Fix this in mind.\n");
    vTaskDelay(pdMS_TO_TICKS(12000));

    // ---- cycle 1: delete, reset, rebuild, redraw identical marker ----
    delete_panel(1);
    interrogate("STATION 2: panel #1 deleted (frame retained)");
    printf("warmhandoff: OBSERVE (6 s): retained frame -- should look UNCHANGED.\n");
    vTaskDelay(pdMS_TO_TICKS(6000));

    printf("\nwarmhandoff: REBUILD #2 (reset pulse + rebuild, mirrors live wake path)...\n");
    panel_hw_reset();
    if (!rebuild_dpi_panel(&s_dpi_panel)) { printf("warmhandoff: rebuild #2 failed. Parking.\n"); for (;;) vTaskDelay(pdMS_TO_TICKS(10000)); }
    init_and_draw(1, "panel #2 (REBUILT)");
    interrogate("STATION 3: panel #2 rebuilt");
    printf("warmhandoff: OBSERVE (12 s): *** THE MEASUREMENT ***\n");
    printf("warmhandoff:   Same marker redrawn. Is the border STILL hugging all 4\n");
    printf("warmhandoff:   edges, or has the whole image MOVED? If moved: which\n");
    printf("warmhandoff:   direction, and roughly how far?\n");
    vTaskDelay(pdMS_TO_TICKS(12000));

    // ---- cycle 2: does a second rebuild move it FURTHER? ----
    delete_panel(2);
    printf("\nwarmhandoff: REBUILD #3 (second rebuild -- does the offset GROW?)...\n");
    panel_hw_reset();
    if (!rebuild_dpi_panel(&s_dpi_panel)) { printf("warmhandoff: rebuild #3 failed. Parking.\n"); for (;;) vTaskDelay(pdMS_TO_TICKS(10000)); }
    init_and_draw(2, "panel #3 (REBUILT twice)");
    interrogate("STATION 4: panel #3, second rebuild");
    printf("warmhandoff: OBSERVE (15 s): compare with the previous station.\n");
    printf("warmhandoff:   Offset the SAME as after rebuild #2  -> fixed difference.\n");
    printf("warmhandoff:   Offset BIGGER (moved further)        -> cumulative.\n");
    printf("warmhandoff:   No offset at all on either rebuild   -> panel is fine,\n");
    printf("warmhandoff:                                           problem is LVGL.\n");
    vTaskDelay(pdMS_TO_TICKS(15000));

    printf("\nwarmhandoff: probe complete. Parked -- final marker left on screen.\n");
    printf("warmhandoff: Report: (a) reference OK? (b) shifted after rebuild #2?\n");
    printf("warmhandoff:         (c) shifted further after rebuild #3? (d) direction.\n");
    for (;;) vTaskDelay(pdMS_TO_TICKS(10000));
}