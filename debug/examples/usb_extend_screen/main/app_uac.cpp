/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "app_usb.h"
#include "esp_log.h"
#include "usb_device_uac.h"
#include "usb_descriptors.h"
#include "cpp_bus_driver_library.h"

extern std::unique_ptr<Cpp_Bus_Driver::Es8311> ES8311;

static const char *TAG = "app_uac";

static esp_err_t uac_device_output_cb(uint8_t *buf, size_t len, void *arg)
{
    if (ES8311->write_data(buf, len) == false)
    {
        printf("write_data fail\n");
        return ESP_FAIL;
    }

    printf("66666666666666666: %d\n", buf[0]);

    return ESP_OK;
}

static esp_err_t uac_device_input_cb(uint8_t *buf, size_t len, size_t *bytes_read, void *arg)
{
    size_t buffer = ES8311->read_data(buf, len);
    if (buffer == false)
    {
        printf("read_data fail\n");
        return ESP_FAIL;
    }

    *bytes_read = buffer;
    return ESP_OK;
}

static void uac_device_set_mute_cb(uint32_t mute, void *arg)
{
    ESP_LOGD(TAG, "uac_device_set_mute_cb: %" PRIu32 "", mute);

    ES8311->set_dac_power(!mute);
}

static void uac_device_set_volume_cb(uint32_t volume, void *arg)
{
    ESP_LOGD(TAG, "uac_device_set_volume_cb: %" PRIu32 "", volume);
    ES8311->set_dac_volume(volume + 190);
}

esp_err_t app_uac_init(void)
{
    uac_device_config_t config = {
        .skip_tinyusb_init = true,
        .output_cb = uac_device_output_cb,
        .input_cb = uac_device_input_cb,
        .set_mute_cb = uac_device_set_mute_cb,
        .set_volume_cb = uac_device_set_volume_cb,
        .cb_ctx = NULL,
#if CONFIG_UAC_SPEAKER_CHANNEL_NUM > 0
        .spk_itf_num = ITF_NUM_AUDIO_STREAMING_SPK,
#endif
#if CONFIG_UAC_MIC_CHANNEL_NUM > 0
        .mic_itf_num = ITF_NUM_AUDIO_STREAMING_MIC,
#endif
    };

    uac_device_init(&config);

    return ESP_OK;
}
