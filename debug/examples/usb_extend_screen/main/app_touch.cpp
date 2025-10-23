/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "esp_log.h"
#include "app_touch.h"
#include "app_usb.h"
#include "usb_descriptors.h"
#include "cpp_bus_driver_library.h"

static const char *TAG = "app_touch";

#if defined CONFIG_SCREEN_TYPE_HI8561

extern std::unique_ptr<Cpp_Bus_Driver::Hi8561_Touch> HI8561_T;

#elif defined CONFIG_SCREEN_TYPE_RM69A10

extern std::unique_ptr<Cpp_Bus_Driver::Gt9895> GT9895;

#else
#error "unknown macro definition, please select the correct macro definition."
#endif

static void app_touch_task(void *arg)
{
    uint8_t touchpad_cnt = 0;
    bool send_press = false;
    while (1)
    {
        bool touchpad_pressed;
#if defined CONFIG_SCREEN_TYPE_HI8561
        Cpp_Bus_Driver::Hi8561_Touch::Touch_Point tp;
        touchpad_pressed = HI8561_T->get_multiple_touch_point(tp);
        touchpad_cnt = tp.finger_count;

#elif defined CONFIG_SCREEN_TYPE_RM69A10
        Cpp_Bus_Driver::Gt9895::Touch_Point tp;
        touchpad_pressed = GT9895->get_multiple_touch_point(tp);
        touchpad_cnt = tp.finger_count;
#else
#error "unknown macro definition, please select the correct macro definition."
#endif

        hid_report_t report = {0};
        if (touchpad_pressed && touchpad_cnt > 0)
        {
            report.report_id = REPORT_ID_TOUCH;
            int i = 0;
            for (i = 0; i < touchpad_cnt; i++)
            {
#if defined CONFIG_SCREEN_TYPE_HI8561
                report.touch_report.data[i].index = i;
#elif defined CONFIG_SCREEN_TYPE_RM69A10
                report.touch_report.data[i].index = tp.info[i].finger_id;
#else
#error "unknown macro definition, please select the correct macro definition."
#endif
                report.touch_report.data[i].press_down = 1;
                report.touch_report.data[i].x = tp.info[i].x;
                report.touch_report.data[i].y = tp.info[i].y;
                report.touch_report.data[i].width = tp.info[i].pressure_value;
                report.touch_report.data[i].height = tp.info[i].pressure_value;
                /*!< >= LOG_LEVEL_DEBUG */
#if CONFIG_LOG_DEFAULT_LEVEL >= 4
                /*!< For debug */
                // printf("finger_count: %d edge_touch_flag: %d\nx: %d y: %d pressure_value: %d\n",
                //        tp.finger_count, tp.edge_touch_flag, tp.info[0].x, tp.info[0].y, tp.info[0].pressure_value);

                printf("touch finger: %d edge touch flag: %d\n", tp.finger_count, tp.edge_touch_flag);
                for (uint8_t i = 0; i < tp.info.size(); i++)
                {
                    printf("touch num [%d] x: %d y: %d p: %d\n", i + 1, tp.info[i].x, tp.info[i].y, tp.info[i].pressure_value);
                }
#endif
            }
#if CONFIG_LOG_DEFAULT_LEVEL >= 4
            /*!< For debug */
            printf("\n");
#endif
            ESP_LOGD(TAG, "touchpad cnt: %d\n", touchpad_cnt);
            report.touch_report.cnt = touchpad_cnt;
#if CFG_TUD_HID
            tinyusb_hid_keyboard_report(report);
#endif
            send_press = true;
        }
        else if (send_press)
        {
            send_press = false;
            report.report_id = REPORT_ID_TOUCH;
#if CFG_TUD_HID
            tinyusb_hid_keyboard_report(report);
#endif
            ESP_LOGD(TAG, "send release %d", touchpad_cnt);
        }
        // Reading from the GT911 at a time shorter than this may result in false reports.
        vTaskDelay(20 / portTICK_PERIOD_MS);
    }
}

esp_err_t app_touch_init(void)
{
    xTaskCreate(app_touch_task, "app_touch_task", 4096, NULL, CONFIG_TOUCH_TASK_PRIORITY, NULL);
    return ESP_OK;
}
