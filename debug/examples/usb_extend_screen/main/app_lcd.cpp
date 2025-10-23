/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "esp_ldo_regulator.h"
#include "esp_lcd_mipi_dsi.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_types.h"
#include "esp_cache.h"
#include "esp_dma_utils.h"
#include "esp_private/esp_cache_private.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "driver/ppa.h"
#include "driver/gpio.h"
#include "driver/jpeg_decode.h"
#include "app_lcd.h"
#include "app_usb.h"
#include "sdkconfig.h"
#include "t_display_p4_driver.h"
#include "driver/ppa.h"
#include <algorithm>
#include <cmath>

#define ALIGN_UP(num, align) (((num) + ((align) - 1)) & ~((align) - 1))

static const char *TAG = "app_lcd";

static esp_lcd_panel_handle_t display_handle;
static jpeg_decoder_handle_t jpgd_handle = NULL;

static jpeg_decode_cfg_t decode_cfg = {
    .output_format = JPEG_DECODE_OUT_FORMAT_RGB565,
    .rgb_order = JPEG_DEC_RGB_ELEMENT_ORDER_BGR,
};

ppa_client_handle_t ppa_srm_handle = NULL;
size_t data_cache_line_size = 0;

void app_lcd_draw(uint8_t *buf, uint32_t len, uint16_t width, uint16_t height)
{
    static int fps_count = 0;
    static int64_t start_time = 0;
    fps_count++;
    if (fps_count == 50)
    {
        int64_t end_time = esp_timer_get_time();
        ESP_LOGI(TAG, "fps: %f", 1000000.0 / ((end_time - start_time) / 50.0));
        start_time = end_time;
        fps_count = 0;
    }

    uint32_t input_img_width = EXAMPLE_LCD_H_RES;
    uint32_t input_img_height = EXAMPLE_LCD_V_RES;

    jpeg_decode_memory_alloc_cfg_t rx_mem_cfg = {
        .buffer_direction = JPEG_DEC_ALLOC_OUTPUT_BUFFER,
    };

    size_t input_buffer_size = input_img_width * input_img_height * (SCREEN_BITS_PER_PIXEL / 8);
    uint8_t *input_buffer = (uint8_t *)heap_caps_malloc(input_buffer_size, MALLOC_CAP_DMA | MALLOC_CAP_SPIRAM);
    if (input_buffer == NULL)
    {
        printf("heap_caps_malloc fail\n");
        return;
    }

    uint32_t out_size = 0;
    esp_err_t assert = jpeg_decoder_process(jpgd_handle, &decode_cfg, buf, len, input_buffer, input_buffer_size, &out_size);
    if (assert != ESP_OK)
    {
        printf("jpeg_decoder_process fail\n");
        heap_caps_free(input_buffer);
        return;
    }

    float scale_factor = 1;

    // 根据旋转角度确定输出尺寸，宽度和高度需要交换
    uint32_t output_img_width = static_cast<float>(input_img_height) * scale_factor;
    uint32_t output_img_height = static_cast<float>(input_img_width) * scale_factor;

    size_t output_buffer_size = output_img_width * output_img_height * (SCREEN_BITS_PER_PIXEL / 8);
    uint8_t *output_buffer = (uint8_t *)heap_caps_malloc(output_buffer_size, MALLOC_CAP_DMA | MALLOC_CAP_SPIRAM);
    if (output_buffer == NULL)
    {
        printf("heap_caps_malloc fail\n");
        heap_caps_free(input_buffer);
        return;
    }

    ppa_srm_oper_config_t srm_config =
        {
            .in =
                {
                    .buffer = input_buffer,
                    .pic_w = input_img_width,
                    .pic_h = input_img_height,
                    .block_w = input_img_width,
                    .block_h = input_img_height,
                    .block_offset_x = 0,
                    .block_offset_y = 0,
#if defined CONFIG_LCD_PIXEL_FORMAT_RGB565
                    .srm_cm = ppa_srm_color_mode_t::PPA_SRM_COLOR_MODE_RGB565,
#elif defined CONFIG_LCD_PIXEL_FORMAT_RGB888
                    .srm_cm = ppa_srm_color_mode_t::PPA_SRM_COLOR_MODE_RGB888,
#else
#error "unknown macro definition, please select the correct macro definition."
#endif
                },

            .out =
                {
                    .buffer = output_buffer,
                    .buffer_size = ALIGN_UP(output_buffer_size, data_cache_line_size),
                    .pic_w = output_img_width,
                    .pic_h = output_img_height,
                    .block_offset_x = 0,
                    .block_offset_y = 0,
#if defined CONFIG_LCD_PIXEL_FORMAT_RGB565
                    .srm_cm = ppa_srm_color_mode_t::PPA_SRM_COLOR_MODE_RGB565,
#elif defined CONFIG_LCD_PIXEL_FORMAT_RGB888
                    .srm_cm = ppa_srm_color_mode_t::PPA_SRM_COLOR_MODE_RGB888,
#else
#error "unknown macro definition, please select the correct macro definition."
#endif
                },

            .rotation_angle = PPA_SRM_ROTATION_ANGLE_90,
            .scale_x = scale_factor,
            .scale_y = scale_factor,
            .mirror_x = false,
            .mirror_y = false,
            .rgb_swap = false,
            .byte_swap = false,
            .mode = PPA_TRANS_MODE_BLOCKING,
        };

    assert = ppa_do_scale_rotate_mirror(ppa_srm_handle, &srm_config);
    if (assert != ESP_OK)
    {
        printf("ppa_do_scale_rotate_mirror fail (error code: 0x%X)\n", assert);
        heap_caps_free(input_buffer);
        heap_caps_free(output_buffer);
        return;
    }

    esp_lcd_panel_draw_bitmap(display_handle, 0, 0, output_img_width, output_img_height, output_buffer);

    heap_caps_free(input_buffer);
    heap_caps_free(output_buffer);
}

bool Ppa_Screen_Rotation_Init(void)
{
    ppa_client_config_t ppa_srm_config =
        {
            .oper_type = PPA_OPERATION_SRM,
        };
    esp_err_t assert = ppa_register_client(&ppa_srm_config, &ppa_srm_handle);
    if (assert != ESP_OK)
    {
        printf("ppa_register_client fail (error code: %#X)\n", assert);
        return false;
    }
    assert = esp_cache_get_alignment(MALLOC_CAP_SPIRAM, &data_cache_line_size);
    if (assert != ESP_OK)
    {
        printf("esp_cache_get_alignment fail (error code: %#X)\n", assert);
        return false;
    }

    return true;
}

esp_err_t app_lcd_init(esp_lcd_panel_handle_t *handle)
{
    Ppa_Screen_Rotation_Init();

    jpeg_decode_engine_cfg_t decode_eng_cfg =
        {
            .intr_priority = 1,
            .timeout_ms = 50,
        };

    jpeg_new_decoder_engine(&decode_eng_cfg, &jpgd_handle);

    Screen_Init(&display_handle);
    esp_err_t assert = esp_lcd_panel_init(display_handle);
    if (assert != ESP_OK)
    {
        printf("esp_lcd_panel_init fail (error code: %#X)\n", assert);
        return ESP_FAIL;
    }

    *handle = display_handle;

    return ESP_OK;
}
