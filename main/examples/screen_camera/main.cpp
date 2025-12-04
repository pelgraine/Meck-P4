/*
 * @Description: screen_camera
 * @Author: LILYGO_L
 * @Date: 2025-06-13 11:45:00
 * @LastEditTime: 2025-12-04 13:34:19
 * @License: GPL 3.0
 */
#include "esp_video_init.h"
#include "esp_lcd_mipi_dsi.h"
#include "esp_lcd_panel_ops.h"
#include "esp_cache.h"
#include "esp_heap_caps.h"
#include "esp_private/esp_cache_private.h"
#include "esp_timer.h"
#include "driver/ppa.h"
#include "esp_ldo_regulator.h"
#include "t_display_p4_config.h"
#include "cpp_bus_driver_library.h"
#include "t_display_p4_driver.h"
#include "app_video.h"
#if CONFIG_ENABLE_USB_DISPLAY == true
#include "esp_lcd_usb_display.h"
#endif

#define ALIGN_UP(num, align) (((num) + ((align) - 1)) & ~((align) - 1))

ppa_client_handle_t ppa_srm_handle = NULL;
size_t data_cache_line_size = 0;
void *lcd_buffer[CONFIG_EXAMPLE_CAM_BUF_COUNT];
int32_t video_cam_fd0;

int32_t fps_count;
int64_t start_time;

esp_lcd_panel_handle_t Screen_Mipi_Dpi_Panel = NULL;

auto IIC_Bus_0 = std::make_shared<Cpp_Bus_Driver::Hardware_Iic_1>(XL9535_SDA, XL9535_SCL, I2C_NUM_0);
auto IIC_Bus_1 = std::make_shared<Cpp_Bus_Driver::Hardware_Iic_1>(SGM38121_SDA, SGM38121_SCL, I2C_NUM_1);

auto XL9535 = std::make_unique<Cpp_Bus_Driver::Xl95x5>(IIC_Bus_0, XL9535_IIC_ADDRESS, DEFAULT_CPP_BUS_DRIVER_VALUE);
auto SGM38121 = std::make_unique<Cpp_Bus_Driver::Sgm38121>(IIC_Bus_1, SGM38121_IIC_ADDRESS, DEFAULT_CPP_BUS_DRIVER_VALUE);
auto ESP32P4 = std::make_unique<Cpp_Bus_Driver::Tool>();

void camera_video_frame_operation(uint8_t *camera_buf, uint8_t camera_buf_index, uint32_t camera_buf_hes, uint32_t camera_buf_ves,
                                  size_t camera_buf_len, void *user_data)
{
    fps_count++;
    if (fps_count == 50)
    {
        int64_t end_time = esp_timer_get_time();
        printf("fps: %f\n", 1000000.0 / ((end_time - start_time) / 50.0));
        start_time = end_time;
        fps_count = 0;

        printf("camera_buf_hes: %lu, camera_buf_ves: %lu, camera_buf_len: %d KB\n", camera_buf_hes, camera_buf_ves, camera_buf_len / 1024);
    }

    uint32_t input_img_block_width = (camera_buf_hes - SCREEN_WIDTH) / 2;
    uint32_t input_img_block_height = 0;
    uint32_t input_img_width = SCREEN_WIDTH;
    uint32_t input_img_height = camera_buf_ves;

    uint32_t output_img_width = input_img_width;
    uint32_t output_img_height = input_img_height;

    size_t output_buffer_size = output_img_width * output_img_height * (SCREEN_BITS_PER_PIXEL / 8);
    uint8_t *output_buffer = (uint8_t *)heap_caps_malloc(output_buffer_size, MALLOC_CAP_DMA | MALLOC_CAP_SPIRAM);
    if (output_buffer == NULL)
    {
        printf("heap_caps_malloc fail\n");
        return;
    }

    ppa_srm_oper_config_t srm_config =
        {
            .in =
                {
                    .buffer = camera_buf,
                    .pic_w = camera_buf_hes,
                    .pic_h = camera_buf_ves,
                    .block_w = input_img_width,
                    .block_h = input_img_height,
                    .block_offset_x = input_img_block_width,
                    .block_offset_y = input_img_block_height,
#if (defined CONFIG_CAMERA_TYPE_SC2336) || (defined CONFIG_CAMERA_TYPE_OV2710)
#if defined CONFIG_SCREEN_PIXEL_FORMAT_RGB565
                    .srm_cm = ppa_srm_color_mode_t::PPA_SRM_COLOR_MODE_RGB565,
#elif defined CONFIG_SCREEN_PIXEL_FORMAT_RGB888
                    .srm_cm = ppa_srm_color_mode_t::PPA_SRM_COLOR_MODE_RGB888,
#else
#error "unknown macro definition, please select the correct macro definition."
#endif
#elif defined CONFIG_CAMERA_TYPE_OV5645
                    .srm_cm = ppa_srm_color_mode_t::PPA_SRM_COLOR_MODE_RGB565,
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
#if defined CONFIG_SCREEN_PIXEL_FORMAT_RGB565
                    .srm_cm = ppa_srm_color_mode_t::PPA_SRM_COLOR_MODE_RGB565,
#elif defined CONFIG_SCREEN_PIXEL_FORMAT_RGB888
                    .srm_cm = ppa_srm_color_mode_t::PPA_SRM_COLOR_MODE_RGB888,
#else
#error "unknown macro definition, please select the correct macro definition."
#endif
                },

            .rotation_angle = PPA_SRM_ROTATION_ANGLE_0,
            .scale_x = 1,
            .scale_y = 1,
            .mirror_x = false,
#if defined SCREEN_ROTATION_DIRECTION_0
#if defined CONFIG_SCREEN_TYPE_HI8561
            .mirror_y = true,
#elif defined CONFIG_SCREEN_TYPE_RM69A10
            .mirror_y = false,
#else
#error "unknown macro definition, please select the correct macro definition."
#endif
#elif defined SCREEN_ROTATION_DIRECTION_90
            .mirror_y = false,
#else
#error "unknown macro definition, please select the correct macro definition."
#endif
            .rgb_swap = false,
            .byte_swap = false,
            .mode = PPA_TRANS_MODE_BLOCKING,
        };

    esp_err_t assert = ppa_do_scale_rotate_mirror(ppa_srm_handle, &srm_config);
    if (assert != ESP_OK)
    {
        printf("ppa_do_scale_rotate_mirror fail (error code: %#X)\n", assert);
        heap_caps_free(output_buffer);
        return;
    }

    assert = esp_lcd_panel_draw_bitmap(Screen_Mipi_Dpi_Panel, 0, (SCREEN_HEIGHT - output_img_height) / 2,
                                       output_img_width, output_img_height + ((SCREEN_HEIGHT - output_img_height) / 2),
                                       output_buffer);
    if (assert != ESP_OK)
    {
        printf("esp_lcd_panel_draw_bitmap fail (error code: %#X)\n", assert);
        heap_caps_free(output_buffer);
        return;
    }

    heap_caps_free(output_buffer);
}

bool App_Video_Init()
{
    esp_lcd_panel_handle_t mipi_dpi_panel = NULL;

    if (Camera_Init(&mipi_dpi_panel) == false)
    {
        printf("Camera_Init fail\n");
        return false;
    }

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

    assert = app_video_main(IIC_Bus_1->get_bus_handle());
    if (assert != ESP_OK)
    {
        printf("video_init fail (error code: %#X)\n", assert);
        return false;
    }

#if (defined CONFIG_CAMERA_TYPE_SC2336) || (defined CONFIG_CAMERA_TYPE_OV2710)
#if defined CONFIG_SCREEN_PIXEL_FORMAT_RGB565
    video_cam_fd0 = app_video_open(EXAMPLE_CAM_DEV_PATH, video_fmt_t::APP_VIDEO_FMT_RGB565);
    if (video_cam_fd0 < 0)
    {
        printf("video cam open fail (video_cam_fd0: %ld)\n", video_cam_fd0);
        return false;
    }
#elif defined CONFIG_SCREEN_PIXEL_FORMAT_RGB888
    video_cam_fd0 = app_video_open(EXAMPLE_CAM_DEV_PATH, video_fmt_t::APP_VIDEO_FMT_RGB888);
    if (video_cam_fd0 < 0)
    {
        printf("video cam open fail (video_cam_fd0: %ld)\n", video_cam_fd0);
        return false;
    }
#else
#error "unknown macro definition, please select the correct macro definition."
#endif
#elif defined CONFIG_CAMERA_TYPE_OV5645
    video_cam_fd0 = app_video_open(EXAMPLE_CAM_DEV_PATH, video_fmt_t::APP_VIDEO_FMT_RGB565);
    if (video_cam_fd0 < 0)
    {
        printf("video cam open fail (video_cam_fd0: %ld)\n", video_cam_fd0);
        return false;
    }
#else
#error "unknown macro definition, please select the correct macro definition."
#endif

#if CONFIG_EXAMPLE_CAM_BUF_COUNT == 2
    assert = esp_lcd_dpi_panel_get_frame_buffer(mipi_dpi_panel, 2, &lcd_buffer[0], &lcd_buffer[1]);
#else
    assert = esp_lcd_dpi_panel_get_frame_buffer(mipi_dpi_panel, 3, &lcd_buffer[0], &lcd_buffer[1], &lcd_buffer[2]);
#endif
    if (assert != ESP_OK)
    {
        printf("esp_lcd_dpi_panel_get_frame_buffer fail (error code: %#X)\n", assert);
        return false;
    }

    // #if CONFIG_EXAMPLE_USE_MEMORY_MAPPING
    //     ESP_LOGI(TAG, "Using map buffer");
    //     // When setting the camera video buffer, it can be written as NULL to automatically allocate the buffer using mapping
    //     assert = app_video_set_bufs(app_video_set_bufs(video_cam_fd0, EXAMPLE_CAM_BUF_NUM, NULL));
    //     if (assert != ESP_OK)
    //     {
    //         printf("app_video_set_bufs fail (error code: %#X)\n", assert);
    //         return false;
    //     }
    // #elif CONFIG_CAMERA_CAMERA_MIPI_RAW8_1280X720_30FPS
    //     printf("using user defined buffer\n");
    //     assert = app_video_set_bufs(video_cam_fd0, CONFIG_EXAMPLE_CAM_BUF_COUNT, (const void **)lcd_buffer);
    //     if (assert != ESP_OK)
    //     {
    //         printf("app_video_set_bufs fail (error code: %#X)\n", assert);
    //         return false;
    //     }
    // #else
    //     void *camera_buf[EXAMPLE_CAM_BUF_NUM];
    //     for (int i = 0; i < EXAMPLE_CAM_BUF_NUM; i++)
    //     {
    //         camera_buf[i] = heap_caps_aligned_calloc(data_cache_line_size, 1, app_video_get_buf_size(), MALLOC_CAP_SPIRAM);
    //     }
    //     assert = app_video_set_bufs(video_cam_fd0, EXAMPLE_CAM_BUF_NUM, (const void **)camera_buf);
    //     if (assert != ESP_OK)
    //     {
    //         printf("app_video_set_bufs fail (error code: %#X)\n", assert);
    //         return false;
    //     }
    // #endif

    assert = app_video_set_bufs(video_cam_fd0, CONFIG_EXAMPLE_CAM_BUF_COUNT, (const void **)lcd_buffer);
    if (assert != ESP_OK)
    {
        printf("app_video_set_bufs fail (error code: %#X)\n", assert);
        return false;
    }

    assert = app_video_register_frame_operation_cb(camera_video_frame_operation);
    if (assert != ESP_OK)
    {

        printf("app_video_register_frame_operation_cb fail (error code: %#X)\n", assert);
        return false;
    }

    assert = app_video_stream_task_start(video_cam_fd0, 0, NULL);
    if (assert != ESP_OK)
    {

        printf("app_video_stream_task_start fail (error code: %#X)\n", assert);
        return false;
    }

    // app_video_stream_task_stop(video_cam_fd0);

    return true;
}

#if CONFIG_ENABLE_USB_DISPLAY == true
bool Usb_Screen_Init(esp_lcd_panel_handle_t *mipi_dpi_panel)
{
    usb_display_vendor_config_t vendor_config_usb = DEFAULT_USB_DISPLAY_VENDOR_CONFIG(SCREEN_WIDTH, SCREEN_HEIGHT,
                                                                                      SCREEN_BITS_PER_PIXEL, *mipi_dpi_panel);

    if (esp_lcd_new_panel_usb_display(&vendor_config_usb, mipi_dpi_panel) != ESP_OK)
    {
        printf("esp_lcd_new_panel_usb_display fail\n");
        return false;
    }

    return true;
}
#endif

extern "C" void app_main(void)
{
    printf("Ciallo\n");
    XL9535->begin();

    XL9535->pin_mode(XL9535_SCREEN_RST, Cpp_Bus_Driver::Xl95x5::Mode::OUTPUT);
    XL9535->pin_write(XL9535_SCREEN_RST, Cpp_Bus_Driver::Xl95x5::Value::HIGH);
    vTaskDelay(pdMS_TO_TICKS(10));
    XL9535->pin_write(XL9535_SCREEN_RST, Cpp_Bus_Driver::Xl95x5::Value::LOW);
    vTaskDelay(pdMS_TO_TICKS(10));
    XL9535->pin_write(XL9535_SCREEN_RST, Cpp_Bus_Driver::Xl95x5::Value::HIGH);
    vTaskDelay(pdMS_TO_TICKS(10));

    XL9535->pin_mode(XL9535_ESP32P4_VCCA_POWER_EN, Cpp_Bus_Driver::Xl95x5::Mode::OUTPUT);
    XL9535->pin_mode(XL9535_5_0_V_POWER_EN, Cpp_Bus_Driver::Xl95x5::Mode::OUTPUT);
    XL9535->pin_mode(XL9535_3_3_V_POWER_EN, Cpp_Bus_Driver::Xl95x5::Mode::OUTPUT);
    // 开关3.3v电压时候必须先将GPS断电
    XL9535->pin_mode(XL9535_GPS_WAKE_UP, Cpp_Bus_Driver::Xl95x5::Mode::OUTPUT);
    XL9535->pin_write(XL9535_GPS_WAKE_UP, Cpp_Bus_Driver::Xl95x5::Value::LOW);
    // 开关3.3v电压时候必须先将ESP32C6断电
    XL9535->pin_mode(XL9535_ESP32C6_EN, Cpp_Bus_Driver::Xl95x5::Mode::OUTPUT);
    XL9535->pin_write(XL9535_ESP32C6_EN, Cpp_Bus_Driver::Xl95x5::Value::LOW);

    XL9535->pin_write(XL9535_ESP32P4_VCCA_POWER_EN, Cpp_Bus_Driver::Xl95x5::Value::LOW);

    XL9535->pin_write(XL9535_5_0_V_POWER_EN, Cpp_Bus_Driver::Xl95x5::Value::HIGH);
    vTaskDelay(pdMS_TO_TICKS(10));
    XL9535->pin_write(XL9535_5_0_V_POWER_EN, Cpp_Bus_Driver::Xl95x5::Value::LOW);
    vTaskDelay(pdMS_TO_TICKS(10));
    XL9535->pin_write(XL9535_5_0_V_POWER_EN, Cpp_Bus_Driver::Xl95x5::Value::HIGH);
    vTaskDelay(pdMS_TO_TICKS(10));

    XL9535->pin_write(XL9535_3_3_V_POWER_EN, Cpp_Bus_Driver::Xl95x5::Value::LOW);
    vTaskDelay(pdMS_TO_TICKS(10));
    XL9535->pin_write(XL9535_3_3_V_POWER_EN, Cpp_Bus_Driver::Xl95x5::Value::HIGH);
    vTaskDelay(pdMS_TO_TICKS(10));
    XL9535->pin_write(XL9535_3_3_V_POWER_EN, Cpp_Bus_Driver::Xl95x5::Value::LOW);
    vTaskDelay(pdMS_TO_TICKS(10));

    SGM38121->begin();
#if defined CONFIG_CAMERA_TYPE_SC2336
    SGM38121->set_output_voltage(Cpp_Bus_Driver::Sgm38121::Channel::AVDD_1, 1800);
    SGM38121->set_output_voltage(Cpp_Bus_Driver::Sgm38121::Channel::AVDD_2, 2800);
    SGM38121->set_channel_status(Cpp_Bus_Driver::Sgm38121::Channel::AVDD_1, Cpp_Bus_Driver::Sgm38121::Status::ON);
    SGM38121->set_channel_status(Cpp_Bus_Driver::Sgm38121::Channel::AVDD_2, Cpp_Bus_Driver::Sgm38121::Status::ON);
#elif defined CONFIG_CAMERA_TYPE_OV2710
    SGM38121->set_output_voltage(Cpp_Bus_Driver::Sgm38121::Channel::DVDD_1, 1500);
    SGM38121->set_output_voltage(Cpp_Bus_Driver::Sgm38121::Channel::AVDD_1, 1800);
    SGM38121->set_output_voltage(Cpp_Bus_Driver::Sgm38121::Channel::AVDD_2, 3100);
    SGM38121->set_channel_status(Cpp_Bus_Driver::Sgm38121::Channel::DVDD_1, Cpp_Bus_Driver::Sgm38121::Status::ON);
    SGM38121->set_channel_status(Cpp_Bus_Driver::Sgm38121::Channel::AVDD_1, Cpp_Bus_Driver::Sgm38121::Status::ON);
    SGM38121->set_channel_status(Cpp_Bus_Driver::Sgm38121::Channel::AVDD_2, Cpp_Bus_Driver::Sgm38121::Status::ON);
#elif defined CONFIG_CAMERA_TYPE_OV5645
    SGM38121->set_output_voltage(Cpp_Bus_Driver::Sgm38121::Channel::DVDD_1, 1500);
    SGM38121->set_output_voltage(Cpp_Bus_Driver::Sgm38121::Channel::AVDD_1, 1800);
    SGM38121->set_output_voltage(Cpp_Bus_Driver::Sgm38121::Channel::AVDD_2, 2800);
    SGM38121->set_channel_status(Cpp_Bus_Driver::Sgm38121::Channel::DVDD_1, Cpp_Bus_Driver::Sgm38121::Status::ON);
    SGM38121->set_channel_status(Cpp_Bus_Driver::Sgm38121::Channel::AVDD_1, Cpp_Bus_Driver::Sgm38121::Status::ON);
    SGM38121->set_channel_status(Cpp_Bus_Driver::Sgm38121::Channel::AVDD_2, Cpp_Bus_Driver::Sgm38121::Status::ON);
#else
#error "unknown macro definition, please select the correct macro definition."
#endif

#if defined CONFIG_SCREEN_TYPE_HI8561
    ESP32P4->create_pwm(HI8561_SCREEN_BL, ledc_channel_t::LEDC_CHANNEL_0, 2000);

#elif defined CONFIG_SCREEN_TYPE_RM69A10
#else
#error "unknown macro definition, please select the correct macro definition."
#endif

    Init_Ldo_Channel_Power(3, 1830);

    vTaskDelay(pdMS_TO_TICKS(100));

    if (App_Video_Init() == false)
    {
        printf("App_Video_Init fail\n");
    }

#if CONFIG_ENABLE_USB_DISPLAY == true
    Usb_Screen_Init(&Screen_Mipi_Dpi_Panel);
#else
    Screen_Init(&Screen_Mipi_Dpi_Panel);
#endif

    esp_err_t assert = esp_lcd_panel_init(Screen_Mipi_Dpi_Panel);
    if (assert != ESP_OK)
    {
        printf("esp_lcd_panel_init fail (error code: %#X)\n", assert);
    }

    // // 设置整个屏幕为白色
    // size_t screen_size = SCREEN_WIDTH * SCREEN_HEIGHT * 2; // RGB565: 2 bytes per pixel
    // size_t data_cache_line_size = 16;                      // 通常16或32，具体可查芯片手册
    // void *white_buf = heap_caps_aligned_calloc(data_cache_line_size, 1, screen_size, MALLOC_CAP_SPIRAM);
    // if (white_buf)
    // {
    //     uint16_t *p = (uint16_t *)white_buf;
    //     for (size_t i = 0; i < SCREEN_WIDTH * SCREEN_HEIGHT; ++i)
    //     {
    //         p[i] = 0xFFFF; // RGB565白色
    //     }
    //     esp_err_t assert = esp_lcd_panel_draw_bitmap(Screen_Mipi_Dpi_Panel, 0, 0, SCREEN_WIDTH, SCREEN_HEIGHT, white_buf);
    //     if (assert != ESP_OK)
    //     {
    //         printf("esp_lcd_panel_draw_bitmap white fail (error code: %#X)\n", assert);
    //     }
    //     heap_caps_free(white_buf);
    // }

#if CONFIG_ENABLE_USB_DISPLAY == true
#else
#if defined CONFIG_SCREEN_TYPE_HI8561
    ESP32P4->start_pwm_gradient_time(100, 500);
#elif defined CONFIG_SCREEN_TYPE_RM69A10
    for (uint8_t i = 0; i < 255; i += 5)
    {
        set_rm69a10_brightness(Screen_Mipi_Dpi_Panel, i);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
#else
#error "unknown macro definition, please select the correct macro definition."
#endif
#endif

    // assert = app_video_stream_task_restart(video_cam_fd0);
    // if (assert != ESP_OK)
    // {
    //     printf("app_video_stream_task_restart fail (error code: %#X)\n", assert);
    // }
    // else
    // {
    //     // Get the initial time for frame rate statistics
    //     start_time = esp_timer_get_time();
    // }

    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
