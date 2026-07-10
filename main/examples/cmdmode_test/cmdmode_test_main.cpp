/*
 * cmdmode_test v9 — corrected-register evidence capture
 * ----------------------------------------------------------------------------
 * Purpose: complete the dossier with exact per-chip register decodes.
 * Corrections vs v5-v8 (labels were shifted one register):
 *   0x0D = RDDIM  (image mode: D5 INVON, D4 ALLON, D3 ALLOFF)
 *   0x0E = RDDSM  (signal mode: D7 = TE line on)
 *   0x0F = RDDSDR (self-diagnostic: D0 CMP_BIT, register-checksum check
 *                  performed after Sleep Out; datasheet nominal 0)
 * New evidence this run captures:
 *   - 0x0F at every station (never actually read before)
 *   - full interrogation WHILE ALLPON is active: RDDIM.ALLON latching to 1
 *     with a black glass = register-level proof "command accepted, display
 *     engine dead" in one line
 *   - RDDPM.IDMON latching under idle mode, same class of proof
 * Everything else (bring-up, vendor init replay, monitored writes) is the
 * proven v5/v8 machinery.
 */

#include <stdio.h>
#include <string.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_err.h"
#include "esp_task_wdt.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_mipi_dsi.h"
#include "t_display_p4_config.h"
#include "cpp_bus_driver_library.h"
#include "t_display_p4_driver.h"

#if !defined CONFIG_SCREEN_TYPE_RM69A10
#error "cmdmode_test targets the RM69A10 AMOLED variant only"
#endif
#if defined CONFIG_SCREEN_PIXEL_FORMAT_RGB565
#define BPP 2
#define COLMOD_VAL 0x75
#else
#define BPP 3
#define COLMOD_VAL 0x77
#endif
#define CHUNK_BYTES 32

static esp_lcd_panel_handle_t s_dpi_panel = NULL;
static esp_lcd_panel_io_handle_t s_dbi = NULL;
static uint8_t s_px[CHUNK_BYTES];

static auto IIC_Bus_0 = std::make_shared<Cpp_Bus_Driver::Hardware_Iic_1>(XL9535_SDA, XL9535_SCL, I2C_NUM_0);
static auto XL9535 = std::make_unique<Cpp_Bus_Driver::Xl95x5>(IIC_Bus_0, XL9535_IIC_ADDRESS, DEFAULT_CPP_BUS_DRIVER_VALUE);

// ------------- monitored tx/rx machinery (proven) -------------
struct Job { uint8_t cmd; const uint8_t *wbuf; uint8_t *rbuf; size_t n; bool read; esp_err_t err; };
static volatile Job s_job;
static SemaphoreHandle_t s_go, s_done;
static void io_worker(void *arg)
{
    (void)arg;
    for (;;) {
        xSemaphoreTake(s_go, portMAX_DELAY);
        Job j = *(Job *)&s_job;
        if (j.read) s_job.err = esp_lcd_panel_io_rx_param(s_dbi, j.cmd, j.rbuf, j.n);
        else        s_job.err = esp_lcd_panel_io_tx_param(s_dbi, j.cmd, j.wbuf, j.n);
        xSemaphoreGive(s_done);
    }
}
static bool mdcs(uint8_t cmd, const uint8_t *buf, size_t n, uint32_t to_ms)
{
    s_job.cmd = cmd; s_job.wbuf = buf; s_job.rbuf = NULL; s_job.n = n; s_job.read = false;
    xSemaphoreGive(s_go);
    if (xSemaphoreTake(s_done, pdMS_TO_TICKS(to_ms)) == pdTRUE) return true;
    printf("v9: !! WRITE WEDGED cmd=0x%02X size=%u\n", cmd, (unsigned)n);
    xTaskCreate(io_worker, "iow", 4096, NULL, 5, NULL);
    return false;
}
static bool mread(uint8_t cmd, uint8_t *buf, size_t n, uint32_t to_ms)
{
    memset(buf, 0xEE, n);
    s_job.cmd = cmd; s_job.wbuf = NULL; s_job.rbuf = buf; s_job.n = n; s_job.read = true;
    xSemaphoreGive(s_go);
    if (xSemaphoreTake(s_done, pdMS_TO_TICKS(to_ms)) == pdTRUE) return s_job.err == ESP_OK;
    printf("v9: !! READ WEDGED cmd=0x%02X\n", cmd);
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
    printf("v9: !! dpi_panel_del WEDGED\n");
    return false;
}

// ------------- corrected, per-chip-decoded interrogation -------------
static void interrogate(const char *when)
{
    uint8_t v[3];
    printf("v9: ===== %s =====\n", when);
    if (mread(0x0A, v, 1, 1500))
        printf("v9: RDDPM  (0x0A)=%02X  BST=%d IDM=%d PTL=%d SLPOUT=%d NOR=%d DISPON=%d\n",
               v[0], (v[0]>>7)&1, (v[0]>>6)&1, (v[0]>>5)&1, (v[0]>>4)&1, (v[0]>>3)&1, (v[0]>>2)&1);
    if (mread(0x0C, v, 1, 1500))
        printf("v9: RDDCOLMOD (0x0C)=%02X\n", v[0]);
    if (mread(0x0D, v, 1, 1500))
        printf("v9: RDDIM  (0x0D)=%02X  INV=%d ALLON=%d ALLOFF=%d\n",
               v[0], (v[0]>>5)&1, (v[0]>>4)&1, (v[0]>>3)&1);
    if (mread(0x0E, v, 1, 1500))
        printf("v9: RDDSM  (0x0E)=%02X  TE_line_on=%d\n", v[0], (v[0]>>7)&1);
    if (mread(0x0F, v, 1, 1500))
        printf("v9: RDDSDR (0x0F)=%02X  CMP_BIT=%d (register checksum; nominal 0)\n",
               v[0], v[0] & 1);
    if (mread(0x52, v, 1, 1500))
        printf("v9: brightness (0x52)=%02X\n", v[0]);
}

// ------------- drawing (proven) -------------
static bool set_window(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
    static uint8_t ca[4], ra[4];
    ca[0] = x0 >> 8; ca[1] = x0; ca[2] = x1 >> 8; ca[3] = x1;
    ra[0] = y0 >> 8; ra[1] = y0; ra[2] = y1 >> 8; ra[3] = y1;
    return mdcs(0x2A, ca, 4, 1500) && mdcs(0x2B, ra, 4, 1500);
}
static void fill_px(uint32_t rgb888)
{
    for (int i = 0; i + BPP <= CHUNK_BYTES; i += BPP) {
#if BPP == 2
        uint16_t c = (uint16_t)(((rgb888 >> 8) & 0xF800) | ((rgb888 >> 5) & 0x07E0) | ((rgb888 >> 3) & 0x001F));
        s_px[i] = c >> 8; s_px[i + 1] = c;
#else
        s_px[i] = rgb888 >> 16; s_px[i + 1] = rgb888 >> 8; s_px[i + 2] = rgb888;
#endif
    }
}
static void stream_rect(uint16_t w, uint16_t h, uint32_t rgb888)
{
    fill_px(rgb888);
    uint32_t total = (uint32_t)w * h * BPP, sent = 0;
    long pkts = 0; bool first = true;
    while (sent < total) {
        uint32_t n = total - sent; if (n > CHUNK_BYTES) n = CHUNK_BYTES; n -= n % BPP;
        if (!mdcs(first ? 0x2C : 0x3C, s_px, n, 1500)) return;
        first = false; sent += n;
        if ((++pkts % 512) == 0) vTaskDelay(1);
    }
}

// ------------- vendor init (verbatim, proven) -------------
struct InitCmd { uint8_t cmd; uint8_t data[4]; uint8_t len; uint16_t delay_ms; };
static const InitCmd k_init[] = {
    {0xFE, {0xFD}, 1, 0}, {0x80, {0xFC}, 1, 0}, {0xFE, {0x00}, 1, 0},
    {0x2A, {0x00, 0x00, 0x02, 0x37}, 4, 0}, {0x2B, {0x00, 0x00, 0x04, 0xCF}, 4, 0},
    {0x31, {0x00, 0x03, 0x02, 0x34}, 4, 0}, {0x30, {0x00, 0x00, 0x04, 0xCF}, 4, 0},
    {0x12, {0x00}, 1, 0}, {0x35, {0x00}, 1, 0}, {0x3A, {COLMOD_VAL}, 1, 0},
    {0x51, {0x00}, 1, 0}, {0x11, {0x00}, 0, 120}, {0x29, {0x00}, 0, 0},
};
static bool replay_vendor_init(void)
{
    for (size_t i = 0; i < sizeof(k_init) / sizeof(k_init[0]); i++) {
        if (!mdcs(k_init[i].cmd, k_init[i].data, k_init[i].len, 1500)) return false;
        if (k_init[i].delay_ms) vTaskDelay(pdMS_TO_TICKS(k_init[i].delay_ms));
    }
    return true;
}
static void panel_hw_reset(void)
{
    XL9535->pin_write(XL9535_SCREEN_RST, Cpp_Bus_Driver::Xl95x5::Value::LOW);
    vTaskDelay(pdMS_TO_TICKS(200));
    XL9535->pin_write(XL9535_SCREEN_RST, Cpp_Bus_Driver::Xl95x5::Value::HIGH);
    vTaskDelay(pdMS_TO_TICKS(200));
}

extern "C" void app_main(void)
{
    printf("cmdmode_test v9: corrected-register evidence capture\n");
    esp_task_wdt_deinit();
    s_go = xSemaphoreCreateBinary(); s_done = xSemaphoreCreateBinary();
    xTaskCreate(io_worker, "iow", 4096, NULL, 5, NULL);

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

    if (!Screen_Init(&s_dpi_panel)) { printf("v9: Screen_Init FAILED\n"); return; }
    ESP_ERROR_CHECK(esp_lcd_panel_init(s_dpi_panel));
    esp_lcd_dbi_io_config_t dbi_cfg = { .virtual_channel = 0, .lcd_cmd_bits = 8, .lcd_param_bits = 8 };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_dbi(Screen_Get_Mipi_Dsi_Bus_Handle(), &dbi_cfg, &s_dbi));

    interrogate("STATION 1: under video (production-equivalent)");

    printf("\nv9: deleting DPI panel...\n");
    monitored_del(s_dpi_panel, 5000);
    s_dpi_panel = NULL;
    panel_hw_reset();
    if (!replay_vendor_init()) { printf("v9: init replay wedged. Parking.\n"); for (;;) vTaskDelay(pdMS_TO_TICKS(10000)); }
    mdcs(0x13, NULL, 0, 1500);                       // NORON
    uint8_t ctrld = 0x28, bmax = 0xFF;
    mdcs(0x53, &ctrld, 1, 1500);
    mdcs(0x51, &bmax, 1, 1500);
    interrogate("STATION 2: command-mode baseline (init+NORON+brightness FF)");

    printf("\nv9: painting RED+BLUE squares (evidence continuity)...\n");
    set_window(20, 20, 119, 119);   stream_rect(100, 100, 0xFF0000);
    set_window(150, 20, 249, 119);  stream_rect(100, 100, 0x0000FF);
    printf("v9: OBSERVE (8 s): squares on glass?\n");
    vTaskDelay(pdMS_TO_TICKS(8000));

    printf("\nv9: ALLPON (0x23) sent. Interrogating WHILE active:\n");
    mdcs(0x23, NULL, 0, 1500);
    interrogate("STATION 3: ALLPON ACTIVE — expect RDDIM.ALLON=1; glass should be WHITE");
    printf("v9: OBSERVE (8 s): is the glass white?\n");
    vTaskDelay(pdMS_TO_TICKS(8000));

    printf("\nv9: ALLPOFF (0x22) sent. Interrogating WHILE active:\n");
    mdcs(0x22, NULL, 0, 1500);
    interrogate("STATION 4: ALLPOFF ACTIVE — expect RDDIM.ALLOFF=1");
    mdcs(0x13, NULL, 0, 1500);                       // NORON clears pixel overrides
    mdcs(0x29, NULL, 0, 1500);                       // DISPON refresh

    printf("\nv9: IDMON (0x39) sent. Interrogating WHILE active:\n");
    mdcs(0x39, NULL, 0, 1500);
    interrogate("STATION 5: IDLE MODE ACTIVE — expect RDDPM.IDM=1; 8-colour squares if display alive");
    printf("v9: OBSERVE (8 s): crude-colour squares?\n");
    vTaskDelay(pdMS_TO_TICKS(8000));
    mdcs(0x38, NULL, 0, 1500);                       // IDMOFF

    interrogate("STATION 6: final state");
    printf("\nv9: dossier complete. Parked.\n");
    for (;;) vTaskDelay(pdMS_TO_TICKS(10000));
}