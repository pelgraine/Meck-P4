/*
 * @Description: None
 * @Author: LILYGO_L
 * @Date: 2024-12-18 10:22:46
 * @LastEditTime: 2025-09-05 17:01:46
 * @License: GPL 3.0
 */
#include "tool.h"

namespace Cpp_Bus_Driver
{
    void Tool::assert_log(Log_Level level, const char *file_name, size_t line_number, const char *format, ...)
    {
#if defined CPP_BUS_LOG_LEVEL_BUS || defined CPP_BUS_LOG_LEVEL_CHIP || defined CPP_BUS_LOG_LEVEL_INFO || defined CPP_BUS_LOG_LEVEL_DEBUG

        switch (level)
        {
#if defined CPP_BUS_LOG_LEVEL_DEBUG
        case Log_Level::DEBUG:
        {
            va_list args;
            va_start(args, format);
            auto buffer = std::make_unique<char[]>(MAX_LOG_BUFFER_SIZE);
            snprintf(buffer.get(), MAX_LOG_BUFFER_SIZE, "[cpp_bus_driver][log debug]->[%s][%u line]: %s", file_name, line_number, format);
            vprintf(buffer.get(), args);
            va_end(args);

            break;
        }
#endif
#if defined CPP_BUS_LOG_LEVEL_INFO
        case Log_Level::INFO:
        {
            va_list args;
            va_start(args, format);
            auto buffer = std::make_unique<char[]>(MAX_LOG_BUFFER_SIZE);
            snprintf(buffer.get(), MAX_LOG_BUFFER_SIZE, "[cpp_bus_driver][log info]->[%s][%u line]: %s", file_name, line_number, format);
            vprintf(buffer.get(), args);
            va_end(args);

            break;
        }
#endif
#if defined CPP_BUS_LOG_LEVEL_BUS
        case Log_Level::BUS:
        {
            va_list args;
            va_start(args, format);
            auto buffer = std::make_unique<char[]>(MAX_LOG_BUFFER_SIZE);
            snprintf(buffer.get(), MAX_LOG_BUFFER_SIZE, "[cpp_bus_driver][log bus]->[%s][%u line]: %s", file_name, line_number, format);
            vprintf(buffer.get(), args);
            va_end(args);

            break;
        }
#endif
#if defined CPP_BUS_LOG_LEVEL_CHIP
        case Log_Level::CHIP:
        {
            va_list args;
            va_start(args, format);
            auto buffer = std::make_unique<char[]>(MAX_LOG_BUFFER_SIZE);
            snprintf(buffer.get(), MAX_LOG_BUFFER_SIZE, "[cpp_bus_driver][log chip]->[%s][%u line]: %s", file_name, line_number, format);
            vprintf(buffer.get(), args);
            va_end(args);

            break;
        }
#endif
        default:
            break;
        }

#endif
    }

    bool Tool::search(const uint8_t *search_library, size_t search_library_length, const char *search_sample, size_t sample_length, size_t *search_index)
    {
        // 检查参数有效性
        if (search_sample == nullptr)
        {
            assert_log(Log_Level::DEBUG, __FILE__, __LINE__, "search invalid(search_sample == nullptr)\n");
            return false;
        }
        else if (search_library == nullptr)
        {
            assert_log(Log_Level::DEBUG, __FILE__, __LINE__, "search invalid(search_library == nullptr\n");
            return false;
        }
        else if (sample_length == 0)
        {
            assert_log(Log_Level::DEBUG, __FILE__, __LINE__, "search invalid(sample_length == 0)\n");
            return false;
        }
        else if (search_library_length == 0)
        {
            assert_log(Log_Level::DEBUG, __FILE__, __LINE__, "search invalid(search_library_length == 0)\n");
            return false;
        }
        else if (sample_length > search_library_length)
        {
            assert_log(Log_Level::DEBUG, __FILE__, __LINE__, "search invalid(sample_length > search_library_length)\n");
            return false;
        }

        auto buffer = std::search(search_library, search_library + search_library_length, search_sample, search_sample + sample_length);
        // 检查是否找到了数据
        if (buffer == (search_library + search_library_length))
        {
            // assert_log(Log_Level::DEBUG, __FILE__, __LINE__, "search fail\n");
            return false;
        }

        if (search_index != nullptr)
        {
            *search_index = buffer - search_library;
        }

        return true;
    }

    bool Tool::search(const char *search_library, size_t search_library_length, const char *search_sample, size_t sample_length, size_t *search_index)
    {
        // 检查参数有效性
        if (search_sample == nullptr)
        {
            assert_log(Log_Level::DEBUG, __FILE__, __LINE__, "search invalid(search_sample == nullptr)\n");
            return false;
        }
        else if (search_library == nullptr)
        {
            assert_log(Log_Level::DEBUG, __FILE__, __LINE__, "search invalid(search_library == nullptr\n");
            return false;
        }
        else if (sample_length == 0)
        {
            assert_log(Log_Level::DEBUG, __FILE__, __LINE__, "search invalid(sample_length == 0)\n");
            return false;
        }
        else if (search_library_length == 0)
        {
            assert_log(Log_Level::DEBUG, __FILE__, __LINE__, "search invalid(search_library_length == 0)\n");
            return false;
        }
        else if (sample_length > search_library_length)
        {
            assert_log(Log_Level::DEBUG, __FILE__, __LINE__, "search invalid(sample_length > search_library_length)\n");
            return false;
        }

        auto buffer = std::search(search_library, search_library + search_library_length, search_sample, search_sample + sample_length);
        // 检查是否找到了数据
        if (buffer == (search_library + search_library_length))
        {
            // assert_log(Log_Level::DEBUG, __FILE__, __LINE__, "search fail\n");
            return false;
        }

        if (search_index != nullptr)
        {
            *search_index = buffer - search_library;
        }

        return true;
    }

    bool Tool::pin_mode(uint32_t pin, Pin_Mode mode, Pin_Status status)
    {
#if defined DEVELOPMENT_FRAMEWORK_ESPIDF
        gpio_config_t config;
        config.pin_bit_mask = BIT64(pin);
        switch (mode)
        {
        case Pin_Mode::DISABLE:
            config.mode = GPIO_MODE_INPUT;
            break;
        case Pin_Mode::INPUT:
            config.mode = GPIO_MODE_INPUT;
            break;
        case Pin_Mode::OUTPUT:
            config.mode = GPIO_MODE_OUTPUT;
            break;
        case Pin_Mode::OUTPUT_OD:
            config.mode = GPIO_MODE_OUTPUT_OD;
            break;
        case Pin_Mode::INPUT_OUTPUT_OD:
            config.mode = GPIO_MODE_INPUT_OUTPUT_OD;
            break;
        case Pin_Mode::INPUT_OUTPUT:
            config.mode = GPIO_MODE_INPUT_OUTPUT;
            break;

        default:
            break;
        }
        switch (status)
        {
        case Pin_Status::DISABLE:
            config.pull_up_en = GPIO_PULLUP_DISABLE;
            config.pull_down_en = GPIO_PULLDOWN_DISABLE;
            break;
        case Pin_Status::PULLUP:
            config.pull_up_en = GPIO_PULLUP_ENABLE;
            config.pull_down_en = GPIO_PULLDOWN_DISABLE;
            break;
        case Pin_Status::PULLDOWN:
            config.pull_up_en = GPIO_PULLUP_DISABLE;
            config.pull_down_en = GPIO_PULLDOWN_ENABLE;
            break;

        default:
            break;
        }
        config.intr_type = GPIO_INTR_DISABLE;
#if SOC_GPIO_SUPPORT_PIN_HYS_FILTER
        config.hys_ctrl_mode = GPIO_HYS_SOFT_ENABLE;
#endif

        esp_err_t assert = gpio_config(&config);
        if (assert != ESP_OK)
        {
            assert_log(Log_Level::BUS, __FILE__, __LINE__, "gpio_config fail (error code: %#X)\n", assert);
            return false;
        }

        return true;

#else
        return false;
#endif
    }

    bool Tool::pin_write(uint32_t pin, bool value)
    {
#if defined DEVELOPMENT_FRAMEWORK_ESPIDF
        esp_err_t assert = gpio_set_level(static_cast<gpio_num_t>(pin), value);
        if (assert != ESP_OK)
        {
            assert_log(Log_Level::BUS, __FILE__, __LINE__, "gpio_config fail (error code: %#X)\n", assert);
            return false;
        }

        return true;
#else
        return false;
#endif
    }

    bool Tool::pin_read(uint32_t pin)
    {
#if defined DEVELOPMENT_FRAMEWORK_ESPIDF
        return gpio_get_level(static_cast<gpio_num_t>(pin));
#else
        return false;
#endif
    }

    void Tool::delay_ms(uint32_t value)
    {
#if defined DEVELOPMENT_FRAMEWORK_ESPIDF
        // 默认状态下vTaskDelay在小于10ms延时的时候不精确
        // vTaskDelay(pdMS_TO_TICKS(value));
        usleep(value * 1000);
#elif defined ARDUINO
        delay(value);
#endif
    }

    void Tool::delay_us(uint32_t value)
    {
#if defined DEVELOPMENT_FRAMEWORK_ESPIDF
        usleep(value);
#elif defined ARDUINO
        delayMicroseconds(value);
#endif
    }

#if defined DEVELOPMENT_FRAMEWORK_ESPIDF

    int64_t Tool::get_system_time_us(void)
    {
        return esp_timer_get_time();
    }

    int64_t Tool::get_system_time_ms(void)
    {
        return esp_timer_get_time() / 1000;
    }

    bool Tool::create_gpio_interrupt(uint32_t pin, Interrupt_Mode mode, void (*interrupt)(void *), void *args)
    {
        gpio_config_t config;
        config.pin_bit_mask = BIT64(pin);
        config.mode = GPIO_MODE_INPUT;
        switch (mode)
        {
        case Interrupt_Mode::DISABLE:
            config.pull_up_en = GPIO_PULLUP_DISABLE;
            config.pull_down_en = GPIO_PULLDOWN_DISABLE;
            config.intr_type = GPIO_INTR_DISABLE;
            break;
        case Interrupt_Mode::RISING:
            config.pull_up_en = GPIO_PULLUP_DISABLE;
            config.pull_down_en = GPIO_PULLDOWN_ENABLE;
            config.intr_type = GPIO_INTR_POSEDGE;
            break;
        case Interrupt_Mode::FALLING:
            config.pull_up_en = GPIO_PULLUP_ENABLE;
            config.pull_down_en = GPIO_PULLDOWN_DISABLE;
            config.intr_type = GPIO_INTR_NEGEDGE;
            break;
        case Interrupt_Mode::CHANGE:
            config.pull_up_en = GPIO_PULLUP_DISABLE;
            config.pull_down_en = GPIO_PULLDOWN_DISABLE;
            config.intr_type = GPIO_INTR_ANYEDGE;
            break;
        case Interrupt_Mode::ONLOW:
            // 只要 GPIO 引脚保持低电平，就会持续触发中断
            // 需要确保你的中断处理函数能够处理这种情况，或者你的外部信号不会长时间保持低电平
            // 否则系统将崩溃重启
            config.pull_up_en = GPIO_PULLUP_ENABLE;
            config.pull_down_en = GPIO_PULLDOWN_DISABLE;
            config.intr_type = GPIO_INTR_LOW_LEVEL;
            break;
        case Interrupt_Mode::ONHIGH:
            // 只要 GPIO 引脚保持高电平，就会持续触发中断
            // 需要确保你的中断处理函数能够处理这种情况，或者你的外部信号不会长时间保持高电平
            // 否则系统将崩溃重启
            config.pull_up_en = GPIO_PULLUP_DISABLE;
            config.pull_down_en = GPIO_PULLDOWN_ENABLE;
            config.intr_type = GPIO_INTR_HIGH_LEVEL;
            break;

        default:
            break;
        }
#if SOC_GPIO_SUPPORT_PIN_HYS_FILTER
        config.hys_ctrl_mode = GPIO_HYS_SOFT_ENABLE;
#endif

        esp_err_t assert = gpio_config(&config);
        if (assert != ESP_OK)
        {
            assert_log(Log_Level::BUS, __FILE__, __LINE__, "gpio_config fail (error code: %#X)\n", assert);
            return false;
        }

        assert = gpio_install_isr_service(0);
        if (assert != ESP_OK)
        {
            assert_log(Log_Level::BUS, __FILE__, __LINE__, "gpio_install_isr_service fail (error code: %#X)\n", assert);
            // return false;
        }

        assert = gpio_isr_handler_add(static_cast<gpio_num_t>(pin), interrupt, args);
        if (assert != ESP_OK)
        {
            assert_log(Log_Level::BUS, __FILE__, __LINE__, "gpio_isr_handler_add fail (error code: %#X)\n", assert);
            return false;
        }

        assert = gpio_intr_enable(static_cast<gpio_num_t>(pin));
        if (assert != ESP_OK)
        {
            assert_log(Log_Level::BUS, __FILE__, __LINE__, "gpio_intr_enable fail (error code: %#X)\n", assert);
            return false;
        }

        return true;
    }

    bool Tool::delete_gpio_interrupt(uint32_t pin)
    {
        esp_err_t assert = gpio_set_intr_type(static_cast<gpio_num_t>(pin), GPIO_INTR_DISABLE);
        if (assert != ESP_OK)
        {
            assert_log(Log_Level::BUS, __FILE__, __LINE__, "gpio_set_intr_type fail (error code: %#X)\n", assert);
            return false;
        }

        assert = gpio_isr_handler_remove(static_cast<gpio_num_t>(pin));
        if (assert != ESP_OK)
        {
            assert_log(Log_Level::BUS, __FILE__, __LINE__, "gpio_isr_handler_remove fail (error code: %#X)\n", assert);
            return false;
        }

        assert = gpio_intr_disable(static_cast<gpio_num_t>(pin));
        if (assert != ESP_OK)
        {
            assert_log(Log_Level::BUS, __FILE__, __LINE__, "gpio_intr_disable fail (error code: %#X)\n", assert);
            return false;
        }

        assert = gpio_reset_pin(static_cast<gpio_num_t>(pin));
        if (assert != ESP_OK)
        {
            assert_log(Log_Level::BUS, __FILE__, __LINE__, "gpio_reset_pin fail (error code: %#X)\n", assert);
            return false;
        }

        return true;
    }

    bool Tool::create_pwm(int32_t pin, ledc_channel_t channel, uint32_t freq_hz, uint32_t duty, ledc_mode_t speed_mode,
                          ledc_timer_bit_t duty_resolution, ledc_timer_t timer_num, ledc_sleep_mode_t sleep_mode)
    {
        const ledc_timer_config_t buffer_ledc_timer_config =
            {
                .speed_mode = speed_mode,
                .duty_resolution = duty_resolution, // LEDC驱动器占空比精度
                .timer_num = timer_num,             // ledc使用的定时器编号，若需要生成多个频率不同的PWM信号，则需要指定不同的定时器
                .freq_hz = freq_hz,                 // PWM频率
                .clk_cfg = LEDC_AUTO_CLK,           // 自动选择定时器的时钟源
                .deconfigure = false,
            };

        esp_err_t assert = ledc_timer_config(&buffer_ledc_timer_config);
        if (assert != ESP_OK)
        {
            assert_log(Log_Level::BUS, __FILE__, __LINE__, "ledc_timer_config fail (error code: %#X)\n", assert);
            return false;
        }

        const ledc_channel_config_t buffer_ledc_channel_config =
            {
                .gpio_num = pin,
                .speed_mode = speed_mode,
                .channel = channel,
                .intr_type = LEDC_INTR_DISABLE,
                .timer_sel = timer_num,
                .duty = duty,
                .hpoint = 0,
                .sleep_mode = sleep_mode,
                .flags =
                    {
                        .output_invert = false,
                    },
            };

        assert = ledc_channel_config(&buffer_ledc_channel_config);
        if (assert != ESP_OK)
        {
            assert_log(Log_Level::BUS, __FILE__, __LINE__, "ledc_channel_config fail (error code: %#X)\n", assert);
            return false;
        }

        _pwm.channel = channel;
        _pwm.freq_hz = freq_hz;
        _pwm.duty = duty;
        _pwm.speed_mode = speed_mode;
        _pwm.duty_resolution = duty_resolution;
        _pwm.timer_num = timer_num;
        _pwm.sleep_mode = sleep_mode;

        return true;
    }

    bool Tool::set_pwm_duty(uint8_t duty)
    {
        if (duty > 100)
        {
            duty = 100;
        }

        esp_err_t assert = ledc_set_duty(_pwm.speed_mode, _pwm.channel, (static_cast<float>(duty) / 100.0) * (1 << static_cast<uint8_t>(_pwm.duty_resolution)));
        if (assert != ESP_OK)
        {
            assert_log(Log_Level::BUS, __FILE__, __LINE__, "ledc_set_duty fail (error code: %#X)\n", assert);
            return false;
        }

        assert = ledc_update_duty(_pwm.speed_mode, _pwm.channel);
        if (assert != ESP_OK)
        {
            assert_log(Log_Level::BUS, __FILE__, __LINE__, "ledc_update_duty fail (error code: %#X)\n", assert);
            return false;
        }

        _pwm.duty = duty;

        return true;
    }

    bool Tool::set_pwm_frequency(uint32_t freq_hz)
    {
        esp_err_t assert = ledc_set_freq(_pwm.speed_mode, _pwm.timer_num, freq_hz);
        if (assert != ESP_OK)
        {
            assert_log(Log_Level::BUS, __FILE__, __LINE__, "ledc_set_duty fail (error code: %#X)\n", assert);
            return false;
        }

        _pwm.freq_hz = freq_hz;

        return true;
    }

    bool Tool::start_pwm_gradient_time(uint8_t target_duty, int32_t time_ms)
    {
        if (target_duty > 100)
        {
            target_duty = 100;
        }

        esp_err_t assert = ledc_fade_func_install(false);
        if (assert != ESP_OK)
        {
            assert_log(Log_Level::BUS, __FILE__, __LINE__, "ledc_fade_func_install fail (error code: %#X)\n", assert);
            // return false;
        }

        assert = ledc_set_fade_with_time(_pwm.speed_mode, _pwm.channel,
                                         (static_cast<float>(target_duty) / 100.0) * (1 << static_cast<uint8_t>(_pwm.duty_resolution)), time_ms);
        if (assert != ESP_OK)
        {
            assert_log(Log_Level::BUS, __FILE__, __LINE__, "ledc_set_fade fail (error code: %#X)\n", assert);
            return false;
        }

        assert = ledc_fade_start(_pwm.speed_mode, _pwm.channel, ledc_fade_mode_t::LEDC_FADE_WAIT_DONE);
        if (assert != ESP_OK)
        {
            assert_log(Log_Level::BUS, __FILE__, __LINE__, "ledc_fade_start fail (error code: %#X)\n", assert);
            return false;
        }

        _pwm.duty = target_duty;

        return true;
    }

    bool Tool::stop_pwm(uint32_t idle_level)
    {
        esp_err_t assert = ledc_stop(_pwm.speed_mode, _pwm.channel, idle_level);
        if (assert != ESP_OK)
        {
            assert_log(Log_Level::BUS, __FILE__, __LINE__, "ledc_stop fail (error code: %#X)\n", assert);
            return false;
        }

        ledc_fade_func_uninstall();

        return true;
    }
#endif

    bool Gnss::parse_rmc_info(const uint8_t *data, size_t length, Rmc &rmc)
    {
        assert_log(Log_Level::DEBUG, __FILE__, __LINE__, "parse_rmc_info(length: %d): \n---begin---\n%s\n---end---\n", length, data);

        size_t buffer_index = 0;
        const char *buffer_cmd = "$GNRMC";
        bool buffer_exit_flag = true;
        size_t buffer_used_size = 0;

        // 循环搜索数据里面所有的命令
        while (1)
        {
            if (search(data + buffer_used_size, length - buffer_used_size, buffer_cmd, std::strlen(buffer_cmd), &buffer_index) == false)
            {
                // assert_log(Log_Level::CHIP, __FILE__, __LINE__, "search fail\n");
                buffer_exit_flag = true;
            }
            else
            {
                buffer_used_size += buffer_index;
                assert_log(Log_Level::DEBUG, __FILE__, __LINE__, "$GNRMC index: %d\n", buffer_used_size);

                buffer_used_size += std::strlen(buffer_cmd);

                uint8_t buffer_field_count = 0; // 记录当前的字段号
                for (auto i = buffer_used_size; i < length; i++)
                {
                    if ((data[i] == '\r') && (data[i + 1] == '\n')) // 停止符
                    {
                        break;
                    }
                    else if (data[i] == ',')
                    {
                        buffer_field_count++;
                        // assert_log(Log_Level::CHIP, __FILE__, __LINE__, "buffer_field_count++\n");
                    }
                    else
                    {
                        switch (buffer_field_count)
                        {
                        case 1: //<UTC>
                        {
                            // 确保数据长度正确
                            size_t buffer_index_2 = 0;
                            const char *buffer_cmd_2 = ",";
                            if (search(data + i, length - i, buffer_cmd_2, std::strlen(buffer_cmd_2), &buffer_index_2) == false)
                            {
                                assert_log(Log_Level::CHIP, __FILE__, __LINE__, "search fail\n");
                                break;
                            }

                            switch (buffer_index_2)
                            {
                            case 9: // 可能为9位（hhmmss.ss）
                            {
                                char buffer_hour[] = {data[i], data[i + 1], '\0'};
                                char buffer_minute[] = {data[i + 2], data[i + 3], '\0'};
                                // data[i + 6]为小数点
                                char buffer_second[] =
                                    {
                                        data[i + 4],
                                        data[i + 5],
                                        data[i + 6],
                                        data[i + 7],
                                        data[i + 8],
                                        '\0',
                                    };

                                rmc.utc.hour = std::stoi(buffer_hour);
                                rmc.utc.minute = std::stoi(buffer_minute);
                                rmc.utc.second = std::stof(buffer_second, nullptr);
                                rmc.utc.update_flag = true;

                                i += (9 - 1);
                                break;
                            }
                            case 10: // 可能为10位（hhmmss.sss）
                            {

                                char buffer_hour[] = {data[i], data[i + 1], '\0'};
                                char buffer_minute[] = {data[i + 2], data[i + 3], '\0'};
                                // data[i + 6]为小数点
                                char buffer_second[] =
                                    {
                                        data[i + 4],
                                        data[i + 5],
                                        data[i + 6],
                                        data[i + 7],
                                        data[i + 8],
                                        data[i + 9],
                                        '\0',
                                    };

                                rmc.utc.hour = std::stoi(buffer_hour);
                                rmc.utc.minute = std::stoi(buffer_minute);
                                rmc.utc.second = std::stof(buffer_second, nullptr);
                                rmc.utc.update_flag = true;

                                i += (10 - 1);
                                break;
                            }
                            default:
                                assert_log(Log_Level::CHIP, __FILE__, __LINE__, "<UTC> length error((length = %d) != 9 or 10)\n", buffer_index_2);
                                i += (buffer_index_2 - 1);
                                buffer_exit_flag = false;
                                break;
                            }

                            break;
                        }
                        case 2: //<Status>
                            rmc.location_status = data[i];
                            break;
                        case 3: //<Lat>
                        {
                            // 确保数据长度正确
                            size_t buffer_index_2 = 0;
                            const char *buffer_cmd_2 = ",";
                            if (search(data + i, length - i, buffer_cmd_2, std::strlen(buffer_cmd_2), &buffer_index_2) == false)
                            {
                                assert_log(Log_Level::CHIP, __FILE__, __LINE__, "search fail\n");
                                break;
                            }
                            if (buffer_index_2 != 10)
                            {
                                assert_log(Log_Level::CHIP, __FILE__, __LINE__, "<Lat> length error((length = %d) != 10)\n", buffer_index_2);
                                i += (buffer_index_2 - 1);
                                buffer_exit_flag = false;
                                break;
                            }

                            char buffer_lat_degrees[] = {data[i], data[i + 1], '\0'};
                            char buffer_lat_minutes[] =
                                {
                                    data[i + 2],
                                    data[i + 3],
                                    data[i + 4],
                                    data[i + 5],
                                    data[i + 6],
                                    data[i + 7],
                                    data[i + 8],
                                    data[i + 9],
                                    '\0',
                                };

                            rmc.location.lat.degrees = std::stoi(buffer_lat_degrees);
                            rmc.location.lat.minutes = std::stof(buffer_lat_minutes, nullptr);
                            rmc.location.lat.degrees_minutes = static_cast<double>(rmc.location.lat.degrees) + (rmc.location.lat.minutes / 60.0);

                            rmc.location.lat.update_flag = true;

                            i += (10 - 1);
                            break;
                        }
                        case 4: //<N/S>
                            rmc.location.lat.direction = data[i];

                            rmc.location.lat.direction_update_flag = true;
                            break;
                        case 5: //<Lon>
                        {
                            // 确保数据长度正确
                            size_t buffer_index_2 = 0;
                            const char *buffer_cmd_2 = ",";
                            if (search(data + i, length - i, buffer_cmd_2, std::strlen(buffer_cmd_2), &buffer_index_2) == false)
                            {
                                assert_log(Log_Level::CHIP, __FILE__, __LINE__, "search fail\n");
                                break;
                            }
                            if (buffer_index_2 != 11)
                            {
                                assert_log(Log_Level::CHIP, __FILE__, __LINE__, "<Lon> length error((length = %d) != 11)\n", buffer_index_2);
                                i += (buffer_index_2 - 1);
                                buffer_exit_flag = false;
                                break;
                            }

                            char buffer_lon_degrees[] = {data[i], data[i + 1], data[i + 2], '\0'};
                            char buffer_lon_minutes[] =
                                {
                                    data[i + 3],
                                    data[i + 4],
                                    data[i + 5],
                                    data[i + 6],
                                    data[i + 7],
                                    data[i + 8],
                                    data[i + 9],
                                    data[i + 10],
                                    '\0',
                                };

                            rmc.location.lon.degrees = std::stoi(buffer_lon_degrees);
                            rmc.location.lon.minutes = std::stof(buffer_lon_minutes, nullptr);
                            rmc.location.lon.degrees_minutes = static_cast<double>(rmc.location.lon.degrees) + (rmc.location.lon.minutes / 60.0);

                            rmc.location.lon.update_flag = true;

                            i += (11 - 1);
                            break;
                        }
                        case 6: //<E/W>
                            rmc.location.lon.direction = data[i];

                            rmc.location.lon.direction_update_flag = true;
                            break;
                        case 7:
                            break;
                        case 8:
                            break;
                        case 9: //<Date>
                        {
                            // 确保数据长度正确
                            size_t buffer_index_2 = 0;
                            const char *buffer_cmd_2 = ",";
                            if (search(data + i, length - i, buffer_cmd_2, std::strlen(buffer_cmd_2), &buffer_index_2) == false)
                            {
                                assert_log(Log_Level::CHIP, __FILE__, __LINE__, "search fail\n");
                                break;
                            }
                            if (buffer_index_2 != 6)
                            {
                                assert_log(Log_Level::CHIP, __FILE__, __LINE__, "<Date> length error((length = %d) != 6)\n", buffer_index_2);
                                i += (buffer_index_2 - 1);
                                buffer_exit_flag = false;
                                break;
                            }

                            char buffer_day[] = {data[i], data[i + 1], '\0'};
                            char buffer_month[] = {data[i + 2], data[i + 3], '\0'};
                            char buffer_year[] = {data[i + 4], data[i + 5], '\0'};

                            rmc.data.day = std::stoi(buffer_day);
                            rmc.data.month = std::stoi(buffer_month);
                            rmc.data.year = std::stoi(buffer_year);
                            rmc.data.update_flag = true;

                            i += (6 - 1);
                            break;
                        }
                        default:
                            break;
                        }
                    }
                }

                assert_log(Log_Level::DEBUG, __FILE__, __LINE__, "parse_rmc_info finish(field count: %d)\n", buffer_field_count);
            }

            if (buffer_exit_flag == true)
            {
                break;
            }
        }

        return true;
    }

    bool Gnss::parse_gga_info(const uint8_t *data, size_t length, Gga &gga)
    {
        assert_log(Log_Level::DEBUG, __FILE__, __LINE__, "parse_gga_info(length: %d): \n---begin---\n%s\n---end---\n", length, data);

        size_t buffer_index = 0;
        const char *buffer_cmd = "$GNGGA";
        bool buffer_exit_flag = true;
        size_t buffer_used_size = 0;

        // 循环搜索数据里面所有的命令
        while (1)
        {
            if (search(data + buffer_used_size, length - buffer_used_size, buffer_cmd, std::strlen(buffer_cmd), &buffer_index) == false)
            {
                // assert_log(Log_Level::CHIP, __FILE__, __LINE__, "search fail\n");
                break;
            }
            else
            {
                buffer_used_size += buffer_index;
                assert_log(Log_Level::DEBUG, __FILE__, __LINE__, "$GNGGA index: %d\n", buffer_used_size);

                buffer_used_size += std::strlen(buffer_cmd);

                uint8_t buffer_field_count = 0; // 记录当前的字段号
                for (auto i = buffer_used_size; i < length; i++)
                {
                    if ((data[i] == '\r') && (data[i + 1] == '\n')) // 停止符
                    {
                        break;
                    }
                    else if (data[i] == ',')
                    {
                        buffer_field_count++;
                        // assert_log(Log_Level::CHIP, __FILE__, __LINE__, "buffer_field_count++\n");
                    }
                    else
                    {
                        switch (buffer_field_count)
                        {
                        case 1: //<UTC>
                        {
                            // 确保数据长度正确
                            size_t buffer_index_2 = 0;
                            const char *buffer_cmd_2 = ",";
                            if (search(data + i, length - i, buffer_cmd_2, std::strlen(buffer_cmd_2), &buffer_index_2) == false)
                            {
                                assert_log(Log_Level::CHIP, __FILE__, __LINE__, "search fail\n");
                                break;
                            }
                            if (buffer_index_2 != 10)
                            {
                                assert_log(Log_Level::CHIP, __FILE__, __LINE__, "<UTC> length error((length = %d) != 10)\n", buffer_index_2);
                                i += (buffer_index_2 - 1);
                                buffer_exit_flag = false;
                                break;
                            }

                            char buffer_hour[] = {data[i], data[i + 1], '\0'};
                            char buffer_minute[] = {data[i + 2], data[i + 3], '\0'};
                            // data[i + 6]为小数点
                            char buffer_second[] =
                                {
                                    data[i + 4],
                                    data[i + 5],
                                    data[i + 6],
                                    data[i + 7],
                                    data[i + 8],
                                    data[i + 9],
                                    '\0',
                                };

                            gga.utc.hour = std::stoi(buffer_hour);
                            gga.utc.minute = std::stoi(buffer_minute);
                            gga.utc.second = std::stof(buffer_second, nullptr);
                            gga.utc.update_flag = true;

                            i += (10 - 1);
                            break;
                        }
                        case 2: //<Lat>
                        {
                            // 确保数据长度正确
                            size_t buffer_index_2 = 0;
                            const char *buffer_cmd_2 = ",";
                            if (search(data + i, length - i, buffer_cmd_2, std::strlen(buffer_cmd_2), &buffer_index_2) == false)
                            {
                                assert_log(Log_Level::CHIP, __FILE__, __LINE__, "search fail\n");
                                break;
                            }
                            if (buffer_index_2 != 10)
                            {
                                assert_log(Log_Level::CHIP, __FILE__, __LINE__, "<Lat> length error((length = %d) != 10)\n", buffer_index_2);
                                i += (buffer_index_2 - 1);
                                buffer_exit_flag = false;
                                break;
                            }

                            char buffer_lat_degrees[] = {data[i], data[i + 1], '\0'};
                            char buffer_lat_minutes[] =
                                {
                                    data[i + 2],
                                    data[i + 3],
                                    data[i + 4],
                                    data[i + 5],
                                    data[i + 6],
                                    data[i + 7],
                                    data[i + 8],
                                    data[i + 9],
                                    '\0',
                                };

                            gga.location.lat.degrees = std::stoi(buffer_lat_degrees);
                            gga.location.lat.minutes = std::stof(buffer_lat_minutes, nullptr);
                            gga.location.lat.degrees_minutes = static_cast<double>(gga.location.lat.degrees) + (gga.location.lat.minutes / 60.0);

                            gga.location.lat.update_flag = true;

                            i += (10 - 1);
                            break;
                        }
                        case 3: //<N/S>
                            gga.location.lat.direction = data[i];

                            gga.location.lat.direction_update_flag = true;
                            break;
                        case 4: //<Lon>
                        {
                            // 确保数据长度正确
                            size_t buffer_index_2 = 0;
                            const char *buffer_cmd_2 = ",";
                            if (search(data + i, length - i, buffer_cmd_2, std::strlen(buffer_cmd_2), &buffer_index_2) == false)
                            {
                                assert_log(Log_Level::CHIP, __FILE__, __LINE__, "search fail\n");
                                break;
                            }
                            if (buffer_index_2 != 11)
                            {
                                assert_log(Log_Level::CHIP, __FILE__, __LINE__, "<Lon> length error((length = %d) != 11)\n", buffer_index_2);
                                i += (buffer_index_2 - 1);
                                buffer_exit_flag = false;
                                break;
                            }

                            char buffer_lon_degrees[] = {data[i], data[i + 1], data[i + 2], '\0'};
                            char buffer_lon_minutes[] =
                                {
                                    data[i + 3],
                                    data[i + 4],
                                    data[i + 5],
                                    data[i + 6],
                                    data[i + 7],
                                    data[i + 8],
                                    data[i + 9],
                                    data[i + 10],
                                    '\0',
                                };

                            gga.location.lon.degrees = std::stoi(buffer_lon_degrees);
                            gga.location.lon.minutes = std::stof(buffer_lon_minutes, nullptr);
                            gga.location.lon.degrees_minutes = static_cast<double>(gga.location.lon.degrees) + (gga.location.lon.minutes / 60.0);

                            gga.location.lon.update_flag = true;

                            i += (11 - 1);
                            break;
                        }
                        case 5: //<E/W>
                            gga.location.lon.direction = data[i];

                            gga.location.lon.direction_update_flag = true;
                            break;
                        case 6: //<Quality>
                        {
                            char buffer_gps_mode_status[] = {data[i], '\0'};

                            gga.gps_mode_status = std::stoi(buffer_gps_mode_status);
                            break;
                        }
                        case 7: //<NumSatUsed>
                        {
                            // 确保数据长度正确
                            size_t buffer_index_2 = 0;
                            const char *buffer_cmd_2 = ",";
                            if (search(data + i, length - i, buffer_cmd_2, std::strlen(buffer_cmd_2), &buffer_index_2) == false)
                            {
                                assert_log(Log_Level::CHIP, __FILE__, __LINE__, "search fail\n");
                                break;
                            }
                            if (buffer_index_2 != 2)
                            {
                                assert_log(Log_Level::CHIP, __FILE__, __LINE__, "<NumSatUsed> length error((length = %d) != 2)\n", buffer_index_2);
                                i += (buffer_index_2 - 1);
                                buffer_exit_flag = false;
                                break;
                            }

                            char buffer_online_satellite_count[] = {data[i], data[i + 1], '\0'};

                            gga.online_satellite_count = std::stoi(buffer_online_satellite_count);

                            i += (2 - 1);
                            break;
                        }
                        case 8: // <HDOP>
                        {
                            size_t buffer_index_3 = 0;

                            if (search(data + i, length - i, ",", 1, &buffer_index_3) == false) // 搜索下一个
                            {
                                assert_log(Log_Level::CHIP, __FILE__, __LINE__, "search fail\n");
                                break;
                            }

                            char buffer_1[buffer_index_3 + 1] = {0};

                            std::memcpy(buffer_1, data + i, buffer_index_3);

                            buffer_1[buffer_index_3] = '\0';

                            gga.hdop = std::stof(buffer_1, nullptr); // 转为float

                            i += (buffer_index_3 - 1);
                            break;
                        }
                        default:
                            break;
                        }
                    }
                }

                assert_log(Log_Level::DEBUG, __FILE__, __LINE__, "buffer_field_count: %d\n", buffer_field_count);
            }

            if (buffer_exit_flag == true)
            {
                break;
            }
        }

        return true;
    }
}