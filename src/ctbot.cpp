/*
 * This file is part of the ct-Bot teensy framework.
 * Copyright (c) 2018 Timo Sandmann
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 3.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

/**
 * @file    ctbot.cpp
 * @brief   Main class of ct-Bot teensy framework
 * @author  Timo Sandmann
 * @date    13.05.2018
 */

#include "ctbot.h"

#include "behavior/ctbot_behavior.h"

#include "cmd_parser.h"
#include "cmd_script.h"
#include "ctbot_cli.h"
#include "i2c_service.h"
#include "fs_service.h"
#include "logger.h"
#include "parameter_storage.h"
#include "scheduler.h"
#include "sensors.h"
#include "speed_control.h"

#include "driver/ena_i2c.h"
#include "driver/lc_display.h"
#include "driver/leds_i2c.h"
#include "driver/motor.h"
#include "driver/mpu_6050.h"
#include "driver/servo.h"
#include "driver/serial_io.h"
#include "driver/serial_t4.h"
#include "driver/tft_display.h"

#include "test/tests.h"

#include "timestamp.h"
#include "lua_wrapper.h"
#include "pprintpp.hpp"
#include "timers.h"
#include "crc32.h"

#include <array>
#include <charconv>
#include <chrono>
#include <cinttypes>
#include <cstdlib>
#include <cstring>
#include <string>
#include <thread>
#include <tuple>

#ifndef sei
#define sei() __enable_irq() // for Audio.h
#endif
#ifndef cli
#define cli() __disable_irq() // for Audio.h
#endif
#include "Audio.h"
#include "arduino_freertos.h" // cleanup of ugly macro stuff etc.
#include "tts.h"


void software_isr();

extern "C" {
FLASHMEM static int lua_wrapper_print(lua_State* L) {
    auto p_comm { ctbot::CtBot::get_instance().get_comm() };

    const int n = lua_gettop(L); /* number of arguments */
    lua_getglobal(L, PSTR("tostring"));
    for (int i = 1; i <= n; i++) {
        lua_pushvalue(L, -1); /* function to be called */
        lua_pushvalue(L, i); /* value to print */
        lua_call(L, 1, 1);
        size_t l;
        const char* s = lua_tolstring(L, -1, &l); /* get result */
        if (s == nullptr) {
            return luaL_error(L, PSTR("'tostring' must return a string to 'print'"));
        }
        if (i > 1) {
            p_comm->debug_print('\t', true);
        }
        p_comm->debug_print(s, true);
        lua_pop(L, 1); /* pop result */
    }
    p_comm->debug_print(PSTR("\r\n"), true);
    return 0;
}
} // extern C

namespace ctbot {
DMAMEM audio_block_t g_audio_mem[CtBotConfig::AUDIO_MEMORY_BLOCKS];
TaskHandle_t CtBot::audio_task_ {};

CtBot& CtBot::get_instance() {
    static CtBot* p_instance { CtBotConfig::BEHAVIOR_MODEL_AVAILABLE ? new CtBotBehavior : new CtBot };
    return *p_instance;
}

FLASHMEM CtBot::CtBot()
    // initializes serial connection here for debug purpose
    : shutdown_ {}, ready_ {}, task_id_ {}, p_scheduler_ {}, p_sensors_ {}, p_motors_ { nullptr, nullptr }, p_speedcontrols_ { nullptr, nullptr },
      p_servos_ { nullptr, nullptr }, p_i2c_1_svc_ {}, p_ena_ {}, p_ena_pwm_ {}, p_leds_ {}, p_lcd_ {}, p_tft_ {}, p_cli_ {},
      p_serial_usb_ { &freertos::get_serial<0>() }, p_serial_wifi_ {}, p_comm_ {}, p_parser_ {}, p_logger_ {}, p_fs_ {}, p_logger_file_ {}, p_parameter_ {},
      p_audio_output_dac_ {}, p_audio_output_i2s_ {}, p_audio_sine_ {}, p_play_wav_ {}, p_tts_ {}, p_watch_timer_ {}, p_clock_timer_ {}, clock_update_done_ {},
      p_lua_ {}, last_viewer_timestamp_ {}, last_taskstat_timestamp_ {}, p_taskstat_context_ {}, p_viewer_tasks_context_ {}, shutdown_timer_ {} {}

CtBot::~CtBot() {
    if constexpr (DEBUG_LEVEL_ > 1) {
        ::serialport_puts(PSTR("CtBot::~CtBot(): destroying CtBot instance.\r\n"));
    }
}

void CtBot::stop() {
    shutdown_ = true;
}

__attribute__((noinline, used)) void terminate_handler() {
    configASSERT(0);
}

extern "C" uint8_t external_psram_size;
FLASHMEM void CtBot::setup(const bool set_ready) {
    auto log_begin { [this]() { p_logger_->begin(PSTR("CtBot::setup(): ")); } };

    if (DEBUG_LEVEL_ > 2) {
        ::serialport_puts(PSTR("CtBot::setup()...\r\n"));
    }
    std::set_terminate(terminate_handler);
    std::atexit([]() FLASHMEM {
        if constexpr (DEBUG_LEVEL_ > 1) {
            ::serialport_puts(PSTR("exit() called.\r\n"));
        }

        CtBot* ptr = &get_instance();
        delete ptr;

        if constexpr (DEBUG_LEVEL_ > 0) {
            ::serialport_puts(PSTR("exit(): stopping scheduler, bye.\r\n"));
        }

        Scheduler::stop();
    });

#if defined ARDUINO_TEENSY41 && !defined TEENSY_NO_EXTRAM // TODO: abstraction?
    if (external_psram_size) {
        constexpr auto divider { calc_flexspi2_divider(CtBotConfig::PSRAM_FREQUENCY_MHZ) };
        CCM_CBCMR =
            (CCM_CBCMR & ~(CCM_CBCMR_FLEXSPI2_PODF_MASK | CCM_CBCMR_FLEXSPI2_CLK_SEL_MASK)) | CCM_CBCMR_FLEXSPI2_PODF(divider) | CCM_CBCMR_FLEXSPI2_CLK_SEL(3);
        FLEXSPI2_FLSHA1CR1 = FLEXSPI_FLSHCR1_CSINTERVAL(0) | FLEXSPI_FLSHCR1_TCSH(0) | FLEXSPI_FLSHCR1_TCSS(0); // reduce CS hold and setup times
    }
#endif // ARDUINO_TEENSY41

    if constexpr (DEBUG_LEVEL_ > 2) {
        ::serialport_puts(PSTR("CtBot::setup(): creating scheduler...\r\n"));
    }
    p_scheduler_ = new Scheduler;
    configASSERT(p_scheduler_);
    p_taskstat_context_ = new SchedulerStatContext;
    p_viewer_tasks_context_ = new SchedulerStatContext;
    configASSERT(p_taskstat_context_ && p_viewer_tasks_context_);
    task_id_ = p_scheduler_->task_add(PSTR("main"), TASK_PERIOD_MS_, TASK_PRIORITY_, STACK_SIZE_, [this]() { return run(); });
    p_scheduler_->task_register(PSTR("Tmr Svc"), true);
    p_scheduler_->task_register(PSTR("YIELD"));
    p_scheduler_->task_register(PSTR("EVENT"));

    if constexpr (DEBUG_LEVEL_ > 2) {
        ::serialport_puts(PSTR("CtBot::setup(): tasks registered.\r\n"));
    }

    p_cli_ = new CtBotCli { this };
    configASSERT(p_cli_);

    p_parser_ = new CmdParser;
    configASSERT(p_parser_);
    if constexpr (CtBotConfig::UART_WIFI_FOR_CMD) {
        p_serial_wifi_ = &freertos::get_serial<CtBotConfig::UART_WIFI>();
        configASSERT(p_serial_wifi_);
        p_serial_wifi_->setRX(CtBotConfig::UART_WIFI_PIN_RX);
        p_serial_wifi_->setTX(CtBotConfig::UART_WIFI_PIN_TX);
        if (!p_serial_wifi_->begin(CtBotConfig::UART_WIFI_BAUDRATE, 0, 12'288, 1'024)) {
            ::serialport_puts(PSTR("CtBot::setup(): p_serial_wifi_->begin() failed.\r\n"));
        }

        p_comm_ = new CommInterfaceCmdParser { *p_serial_wifi_, *p_parser_, true };
        if (CrashReport) {
#if defined ARDUINO_TEENSY40 || defined ARDUINO_TEENSY41
            p_serial_wifi_->get_stream().print(PSTR("SRC_SRSR=0x"));
            p_serial_wifi_->get_stream().println(SRC_SRSR, 16);
#endif // ARDUINO_TEENSY40 || ARDUINO_TEENSY41
            p_serial_wifi_->get_stream().print(CrashReport);
            p_serial_wifi_->write('\r');
            p_serial_wifi_->write('\n');
            p_serial_wifi_->write('\n');
            p_serial_wifi_->flush();
            p_serial_wifi_->flush_direct();
            p_serial_wifi_->clear();
        }

        add_post_hook(
            PSTR("viewer"),
            [this]() {
                const auto now { Timer::get_ms() };
                if (now - last_viewer_timestamp_ > VIEWER_SEND_INTERVAL_MS_) {
                    last_viewer_timestamp_ = now;

                    if (!publish_viewerdata(now)) {
                        p_logger_->begin(PSTR("CtBot::run(): "));
                        p_logger_->log(PSTR("publish_viewerdata() failed.\r\n"), true);
                    }
                }
            },
            false);
    } else {
        p_comm_ = new CommInterfaceCmdParser { *p_serial_usb_, *p_parser_, true };
        if (CrashReport) {
#if defined ARDUINO_TEENSY40 || defined ARDUINO_TEENSY41
            p_serial_usb_->get_stream().print(PSTR("SRC_SRSR=0x"));
            p_serial_usb_->get_stream().println(SRC_SRSR, 16);
#endif // ARDUINO_TEENSY40 || ARDUINO_TEENSY41
            p_serial_usb_->get_stream().print(CrashReport);
            p_serial_usb_->write('\r');
            p_serial_usb_->write('\n');
            p_serial_usb_->write('\n');
            p_serial_usb_->flush();
            p_serial_usb_->flush_direct();
            p_serial_usb_->clear();
        }
    }
    configASSERT(p_comm_);

    p_logger_ = new Logger;
    configASSERT(p_logger_);
    p_logger_->add_target(p_comm_);

    if constexpr (DEBUG_LEVEL_ > 2) {
        log_begin();
        p_logger_->log(PSTR("comm services inited.\r\n"));
    }

    if constexpr (CtBotConfig::SDCARD_AVAILABLE) {
        if constexpr (DEBUG_LEVEL_ > 2) {
            log_begin();
            p_logger_->log(PSTR("SD-card init...\r\n"));
        }
        p_fs_ = new FS_Service { SD };
        configASSERT(p_fs_);
        if (p_fs_->begin(SdioConfig(CtBotConfig::SDCARD_USE_DMA ? DMA_SDIO : FIFO_SDIO))) {
            p_scheduler_->task_register(PSTR("FS Svc"));
            p_parameter_ = new ParameterStorage { *p_fs_, PSTR("ctbot.jsn") };
            configASSERT(p_parameter_);
            if constexpr (DEBUG_LEVEL_ > 2) {
                log_begin();
                p_logger_->log(PSTR("ParameterStorage created.\r\n"), true);
            }
            if constexpr (CtBotConfig::LOG_TO_SDCARD_AVAILABLE) {
                p_logger_file_ = new LoggerTargetFile { *p_fs_, PSTR("logs.txt"), this };
                configASSERT(p_logger_file_);
                p_logger_->add_target(p_logger_file_);
            }
        } else {
            delete p_fs_;
            p_fs_ = nullptr;
            log_begin();
            p_logger_->log(PSTR("p_fs_->begin() failed.\r\n"), true);
        }
        if constexpr (DEBUG_LEVEL_ > 2) {
            log_begin();
            p_logger_->log(PSTR("SD Card init done.\r\n"));
        }

        if constexpr (CtBotConfig::CLI_HISTORY_ON_SDCARD_AVAILABLE) {
            if (p_fs_) {
                p_parser_->enable_nv_history(*p_fs_, PSTR("history.txt"));
            }
        }
    }

    p_i2c_1_svc_ = new I2C_Service { 0, CtBotConfig::I2C1_FREQ, CtBotConfig::I2C1_PIN_SDA, CtBotConfig::I2C1_PIN_SCL };
    configASSERT(p_i2c_1_svc_);
    p_i2c_2_svc_ = new I2C_Service { 1, CtBotConfig::I2C2_FREQ, CtBotConfig::I2C2_PIN_SDA, CtBotConfig::I2C2_PIN_SCL };
    configASSERT(p_i2c_2_svc_);

    switch (CtBotConfig::ENA_I2C_BUS) {
        case 1: p_ena_ = new EnaI2c { p_i2c_1_svc_, CtBotConfig::ENA_I2C_ADDR }; break;
    }
    configASSERT(p_ena_);
    if constexpr (DEBUG_LEVEL_ > 2) {
        log_begin();
        p_logger_->log(PSTR("ENA created.\r\n"));
    }

    switch (CtBotConfig::ENA_PWM_I2C_BUS) {
        case 1: p_ena_pwm_ = new LedsI2cEna<> { p_i2c_1_svc_, CtBotConfig::ENA_PWM_I2C_ADDR }; break;
    }
    configASSERT(p_ena_pwm_);
    if constexpr (DEBUG_LEVEL_ > 2) {
        log_begin();
        p_logger_->log(PSTR("ENA PWM created.\r\n"));
    }
    if constexpr (CtBotConfig::MAINBOARD_REVISION == 9'000) {
        p_ena_pwm_->set_pwm(static_cast<LedTypesEna<>>(LedTypesEna_9000::ENC_L) | static_cast<LedTypesEna<>>(LedTypesEna_9000::ENC_R), 128);
        p_ena_pwm_->off(static_cast<LedTypesEna<>>(LedTypesEna_9000::ENC_L) | static_cast<LedTypesEna<>>(LedTypesEna_9000::ENC_R));
        p_ena_pwm_->on(static_cast<LedTypesEna<>>(LedTypesEna_9000::ENC_L) | static_cast<LedTypesEna<>>(LedTypesEna_9000::ENC_R));
        if constexpr (DEBUG_LEVEL_ > 2) {
            log_begin();
            p_logger_->log(PSTR("ENA inited.\r\n"));
        }
    }

    p_sensors_ = new Sensors { *this, CtBotConfig::VL53L0X_I2C_BUS == 1 && CtBotConfig::VL6180X_I2C_BUS == 1 ? p_i2c_1_svc_ : nullptr, p_i2c_2_svc_ };
    configASSERT(p_sensors_);
    if constexpr (DEBUG_LEVEL_ > 2) {
        log_begin();
        p_logger_->log(PSTR("sensors created.\r\n"));
    }

    p_sensors_->enable_sensors();
    if constexpr (DEBUG_LEVEL_ > 2) {
        log_begin();
        p_logger_->log(PSTR("sensors enabled.\r\n"));
    }

    if constexpr (CtBotConfig::EXTERNAL_SPEEDCTRL) {
        p_speedcontrols_[0] = new SpeedControlExternal {};
        p_speedcontrols_[1] = new SpeedControlExternal {};
    } else {
        p_motors_[0] = new Motor { p_sensors_->get_enc_l(), CtBotConfig::MOT_L_PWM_PIN, CtBotConfig::MOT_L_DIR_PIN, CtBotConfig::MOT_L_DIR };
        p_motors_[1] = new Motor { p_sensors_->get_enc_r(), CtBotConfig::MOT_R_PWM_PIN, CtBotConfig::MOT_R_DIR_PIN, CtBotConfig::MOT_R_DIR };

        p_speedcontrols_[0] = new SpeedControl { p_sensors_->get_enc_l(), *p_motors_[0] };
        p_speedcontrols_[1] = new SpeedControl { p_sensors_->get_enc_r(), *p_motors_[1] };
    }

    p_servos_[0] = new Servo { CtBotConfig::SERVO_1_PIN };
    configASSERT(p_servos_[0]);
    if constexpr (CtBotConfig::SERVO_2_PIN != 255) {
        p_servos_[1] = new Servo { CtBotConfig::SERVO_2_PIN };
        configASSERT(p_servos_[1]);
    }

    switch (CtBotConfig::LED_I2C_BUS) {
        case 1: p_leds_ = new LedsI2c { p_i2c_1_svc_, CtBotConfig::LED_I2C_ADDR }; break;
    }
    configASSERT(p_leds_);

    if constexpr (CtBotConfig::LCD_AVAILABLE) {
        p_lcd_ = new LCDisplay;
        configASSERT(p_lcd_);
    }
    if constexpr (CtBotConfig::TFT_AVAILABLE) {
        p_tft_ = new TFTDisplay { *p_ena_pwm_ };
        configASSERT(p_tft_);
        p_tft_->set_backlight(CtBotConfig::TFT_BACKLIGHT_LEVEL);
        p_scheduler_->task_register(PSTR("TFT Svc"), true);
    } else {
        if constexpr (CtBotConfig::MAINBOARD_REVISION >= 9'002) {
            p_ena_pwm_->off(static_cast<LedTypesEna<CtBotConfig::MAINBOARD_REVISION>>(LedTypesEna<9'002>::TFT_BACKL));
            if constexpr (CtBotConfig::TFT_CONTROLLER_TYPE == 9341) {
                p_ena_pwm_->set_pwm(static_cast<LedTypesEna<CtBotConfig::MAINBOARD_REVISION>>(LedTypesEna<9'002>::TFT_BACKL), static_cast<int>(0.f));
            } else {
                p_ena_pwm_->set_pwm(static_cast<LedTypesEna<CtBotConfig::MAINBOARD_REVISION>>(LedTypesEna<9'002>::TFT_BACKL), static_cast<int>(255.f));
            }
            p_ena_pwm_->on(static_cast<LedTypesEna<CtBotConfig::MAINBOARD_REVISION>>(LedTypesEna<9'002>::TFT_BACKL));
        }
    }

    if constexpr (CtBotConfig::DATE_TIME_AVAILABLE) {
        update_clock();
        p_clock_timer_ = ::xTimerCreate(PSTR("clock_t"), pdMS_TO_TICKS(2'000UL), true, this, [](TimerHandle_t handle) {
            auto p_ctbot { static_cast<CtBot*>(::pvTimerGetTimerID(handle)) };
            configASSERT(p_ctbot);

            xTimerChangePeriod(handle, pdMS_TO_TICKS(60'000UL), 0);

            p_ctbot->update_clock();
        });
        xTimerStart(p_clock_timer_, 0);
    }

    if constexpr (CtBotConfig::AUDIO_AVAILABLE) {
        portENTER_CRITICAL();
        AudioStream::initialize_memory(g_audio_mem, CtBotConfig::AUDIO_MEMORY_BLOCKS);

        if constexpr (CtBotConfig::SDCARD_AVAILABLE) {
            p_play_wav_ = new AudioPlaySdWav;
            configASSERT(p_play_wav_);
        }
        p_tts_ = new TTS;
        configASSERT(p_tts_);
        if constexpr (CtBotConfig::AUDIO_I2S_AVAILABLE) {
            if constexpr (CtBotConfig::MAINBOARD_REVISION > 9'000) {
                p_audio_output_i2s_ = new AudioOutputI2S2;
            } else {
                p_audio_output_i2s_ = new AudioOutputI2S;
            }
            configASSERT(p_audio_output_i2s_);
        }
        if constexpr (CtBotConfig::AUDIO_ANALOG_AVAILABLE) {
            p_audio_output_dac_ = new AudioOutputAnalog;
            configASSERT(p_audio_output_dac_);
        }
        p_audio_mixer_.push_back(new AudioMixer4);
        configASSERT(*p_audio_mixer_.rbegin());
        if constexpr (CtBotConfig::AUDIO_CHANNELS > 1) {
            p_audio_mixer_.push_back(new AudioMixer4);
            configASSERT(*p_audio_mixer_.rbegin());
        }
        if constexpr (CtBotConfig::AUDIO_TEST_AVAILABLE) {
            p_audio_sine_ = new AudioSynthWaveformSine;
            configASSERT(p_audio_sine_);
            p_audio_sine_->frequency(0.f);
            p_audio_sine_->amplitude(.5f);

            p_audio_conn_.push_back(new AudioConnection { *p_audio_sine_, 0, *p_audio_mixer_[0], 3 });
            if constexpr (CtBotConfig::AUDIO_CHANNELS > 1) {
                p_audio_conn_.push_back(new AudioConnection { *p_audio_sine_, 0, *p_audio_mixer_[1], 3 });
            }
        }
        p_audio_conn_.push_back(new AudioConnection { *p_tts_, 0, *p_audio_mixer_[0], 2 });
        if constexpr (CtBotConfig::AUDIO_CHANNELS > 1) {
            p_audio_conn_.push_back(new AudioConnection { *p_tts_, 0, *p_audio_mixer_[1], 2 });
        }
        if constexpr (CtBotConfig::SDCARD_AVAILABLE) {
            p_audio_conn_.push_back(new AudioConnection { *p_play_wav_, 0, *p_audio_mixer_[0], 0 });
            if constexpr (CtBotConfig::AUDIO_CHANNELS > 1) {
                p_audio_conn_.push_back(new AudioConnection { *p_play_wav_, 1, *p_audio_mixer_[1], 0 });
            }
        }
        if constexpr (CtBotConfig::AUDIO_I2S_AVAILABLE) {
            p_audio_conn_.push_back(new AudioConnection { *p_audio_mixer_[0], 0, *p_audio_output_i2s_, 0 });
            if constexpr (CtBotConfig::AUDIO_CHANNELS > 1) {
                p_audio_conn_.push_back(new AudioConnection { *p_audio_mixer_[1], 0, *p_audio_output_i2s_, 1 });
            }
        } else if constexpr (CtBotConfig::AUDIO_ANALOG_AVAILABLE) {
            p_audio_conn_.push_back(new AudioConnection { *p_audio_mixer_[0], 0, *p_audio_output_dac_, 0 });
        }

        for (auto& e : p_audio_conn_) {
            configASSERT(e);
        }

        for (auto& e : p_audio_mixer_) {
            e->gain(0, 0.1f);
            e->gain(1, 0.1f);
            e->gain(2, 0.1f);
            e->gain(3, 0.1f);
        }

        get_scheduler()->task_add(PSTR("audio"), 1, Scheduler::MAX_PRIORITY, 4096, [this]() {
            while (get_ready()) {
                static_assert(configTASK_NOTIFICATION_ARRAY_ENTRIES > 2, "configTASK_NOTIFICATION_ARRAY_ENTRIES needs to be at least 3");
                if (::ulTaskNotifyTakeIndexed(configTASK_NOTIFICATION_ARRAY_ENTRIES - 2, pdTRUE, pdMS_TO_TICKS(500)) == 1) {
                    ::software_isr(); // AudioStream::update_all()
                }
            }
            if (shutdown_) {
                NVIC_DISABLE_IRQ(IRQ_SOFTWARE);
                audio_task_ = nullptr;
            }
        });
        audio_task_ = ::xTaskGetHandle(PSTR("audio"));

        if (audio_task_) {
            NVIC_SET_PRIORITY(IRQ_SOFTWARE, 13 << 4);
            ::attachInterruptVector(IRQ_SOFTWARE, []() FASTRUN {
                if (audio_task_) {
                    BaseType_t higher_woken { pdFALSE };
                    ::vTaskNotifyGiveIndexedFromISR(audio_task_, configTASK_NOTIFICATION_ARRAY_ENTRIES - 2, &higher_woken);
                    portYIELD_FROM_ISR(higher_woken);
                    portDATA_SYNC_BARRIER(); // mitigate arm errata #838869
                }
            });
        } else {
            NVIC_DISABLE_IRQ(IRQ_SOFTWARE);
        }
        portEXIT_CRITICAL();

        p_scheduler_->task_register(PSTR("speak"), true);
    }

    p_scheduler_->task_register(PSTR("I2C Svc 1"));
    p_scheduler_->task_register(PSTR("I2C Svc 2"));
    p_scheduler_->task_register(PSTR("I2C Svc 3"));

    if constexpr (CtBotConfig::LUA_AVAILABLE) {
        p_lua_ = new LuaWrapper;
        configASSERT(p_lua_);
        lua_register(p_lua_->get_state(), PSTR("print"), lua_wrapper_print);
    }

    p_cli_->init_commands();
    if constexpr (DEBUG_LEVEL_ > 2) {
        log_begin();
        p_logger_->log(PSTR("p_cli_->init_commands() done.\r\n"));
    }

    add_post_hook(
        PSTR("task"),
        [this]() FLASHMEM {
            const auto now { Timer::get_ms() };
            if (now - last_taskstat_timestamp_ > 1'000U) {
                last_taskstat_timestamp_ = now;

                auto p_runtime_stats { p_scheduler_->get_runtime_stats(p_taskstat_context_, true) };
                for (auto& e : *p_runtime_stats) {
                    const auto name { ::pcTaskGetName(e.first) };
                    const char* tabs { std::strlen(name) > 6 ? "\t" : "\t\t" };
                    const char* space { e.second >= 10.f ? "" : " " };

                    p_comm_->debug_printf<true>(PP_ARGS("{s}:{s}{s}{.2} %%\r\n", name, tabs, space, e.second));
                }

                if (p_runtime_stats->size()) {
                    p_comm_->debug_print(PSTR("\r\n"), true);
                }
            }
        },
        false);

    if constexpr (DEBUG_LEVEL_ > 2) {
        log_begin();
        p_logger_->log(PSTR("ct-Bot init done.\r\n"), true);
    }
    p_logger_->log(PSTR("\r\n"));
    p_logger_->begin();
    p_logger_->log(
        PSTR("*** Running FreeRTOS kernel " tskKERNEL_VERSION_NUMBER ". Built by gcc " __VERSION__ " (newlib " _NEWLIB_VERSION ") on " __DATE__ ". ***\r\n"),
        true);
    if constexpr (DEBUG_LEVEL_ > 3) {
#if defined ARDUINO_TEENSY40 || defined ARDUINO_TEENSY41
        log_begin();
        p_logger_->log("SRC_SRSR=0x%x\r\n", SRC_SRSR);
#endif // ARDUINO_TEENSY40 || ARDUINO_TEENSY41
    }
    p_logger_->flush();
    p_comm_->debug_print(PSTR("\r\nType \"help\" (or \"h\") to print help message.\r\n\n"), true);
    p_comm_->flush();

    ready_ = set_ready;

    if constexpr (DEBUG_LEVEL_ > 2) {
        log_begin();
        p_logger_->log(PSTR("done.\r\n"));
    }
}

bool CtBot::publish_viewerdata(uint32_t now) {
    static uint32_t last_task_stats_ {};

    if (!p_comm_ || !p_sensors_) {
        return false;
    }

    const auto ram1 { freertos::ram1_usage() };

    p_comm_->debug_printf<true>(PP_ARGS("<sens>dist: {} {}</sens>\r\n", p_sensors_->get_distance_l(), p_sensors_->get_distance_r()));
    if (CtBotConfig::EXTERNAL_SPEEDCTRL && p_speedcontrols_[0] && p_speedcontrols_[1]) {
        p_comm_->debug_printf<true>(PP_ARGS("<sens>enc: {} {}</sens>\r\n", static_cast<int32_t>(p_speedcontrols_[0]->get_enc_speed()),
            static_cast<int32_t>(p_speedcontrols_[1]->get_enc_speed())));
    } else {
        p_comm_->debug_printf<true>(PP_ARGS("<sens>enc: {} {}</sens>\r\n", p_sensors_->get_enc_l().get(), p_sensors_->get_enc_r().get()));
    }
    p_comm_->debug_printf<true>(PP_ARGS("<sens>border: {} {}</sens>\r\n", p_sensors_->get_border_l(), p_sensors_->get_border_r()));
    p_comm_->debug_printf<true>(PP_ARGS("<sens>line: {} {}</sens>\r\n", p_sensors_->get_line_l(), p_sensors_->get_line_r()));
    p_comm_->debug_printf<true>(PP_ARGS("<sens>trans: {} {}</sens>\r\n", p_sensors_->get_transport(), p_sensors_->get_transport_mm()));
    p_comm_->debug_printf<true>(
        PP_ARGS("<sens>rc5: {} {} {}</sens>\r\n", p_sensors_->get_rc5().get_addr(), p_sensors_->get_rc5().get_cmd(), p_sensors_->get_rc5().get_toggle()));
    p_comm_->debug_printf<true>(PP_ARGS("<sens>bat: {.2} {.2}</sens>\r\n", p_sensors_->get_bat_voltage(), p_sensors_->get_bat_voltage() / 4.f));
    p_comm_->debug_printf<true>(PP_ARGS("<sens>speed: {} {}</sens>\r\n", static_cast<int16_t>(p_speedcontrols_[0]->get_enc_speed()),
        static_cast<int16_t>(p_speedcontrols_[1]->get_enc_speed())));
    if constexpr (CtBotConfig::EXTERNAL_SPEEDCTRL) {
        const auto mcurrent { SpeedControlExternal::get_motor_current() };
        p_comm_->debug_printf<true>(PP_ARGS("<sens>mcurrent: {}</sens>\r\n", mcurrent > 5 ? mcurrent : 0));
    }
    if (CtBotConfig::MPU6050_AVAILABLE && p_sensors_->get_mpu6050()) {
        auto [e1, e2, e3] = p_sensors_->get_mpu6050()->get_euler();
        auto [y, p, r] = p_sensors_->get_mpu6050()->get_ypr();
        p_comm_->debug_printf<true>(PP_ARGS("<sens>mpu_e: {9.4} {9.4} {9.4}</sens>\r\n", e1, e2, e3));
        p_comm_->debug_printf<true>(PP_ARGS("<sens>mpu_ypr: {9.4} {9.4} {9.4}</sens>\r\n", y, p, r));
    }
    p_comm_->debug_printf<true>(PP_ARGS("<sens>currents: {} {}</sens>\r\n", p_sensors_->get_5v_current(), p_sensors_->get_servo_current()));
    if (!CtBotConfig::EXTERNAL_SPEEDCTRL && p_motors_[0] && p_motors_[1]) {
        p_comm_->debug_printf<true>(PP_ARGS("<act>motor: {} {}</act>\r\n", p_motors_[0]->get(), p_motors_[1]->get()));
    } else if (CtBotConfig::EXTERNAL_SPEEDCTRL && p_speedcontrols_[0] && p_speedcontrols_[1]) {
        p_comm_->debug_printf<true>(PP_ARGS(
            "<act>motor: {} {}</act>\r\n", static_cast<int32_t>(p_speedcontrols_[0]->get_speed()), static_cast<int32_t>(p_speedcontrols_[1]->get_speed())));
    }
    p_comm_->debug_printf<true>(
        PP_ARGS("<act>servo1: {} [{s}]</act>\r\n", p_servos_[0]->get_position(), p_servos_[0]->get_active() ? PSTR("on") : PSTR("off")));
    if (p_servos_[1]) {
        p_comm_->debug_printf<true>(
            PP_ARGS("<act>servo2: {} [{s}]</act>\r\n", p_servos_[1]->get_position(), p_servos_[1]->get_active() ? PSTR("on") : PSTR("off")));
    }
    if (p_leds_) {
        p_comm_->debug_printf<true>(PP_ARGS("<act>leds: {}</act>\r\n", static_cast<uint8_t>(p_leds_->get())));
    }

    if (now - last_task_stats_ > VIEWER_SEND_TASKS_INTERVAL_MS_) {
        last_task_stats_ = now;
        {
            auto p_runtime_stats { p_scheduler_->get_runtime_stats(p_viewer_tasks_context_, false) };
            for (auto& e : *p_runtime_stats) {
                const auto name { ::pcTaskGetName(e.first) };
                const auto id { p_scheduler_->task_get(name) };
                p_comm_->debug_printf<true>(PP_ARGS("<sys>task:{}:{s}:{.6}</sys>\r\n", id, name, e.second));
            }
            p_comm_->debug_print(PSTR("<sys>task:-1: :0.0</sys>\r\n"), true);
        }

        p_comm_->debug_printf<true>(
            PP_ARGS("<sys>ram:1:{}:{}:{}:{}:{}</sys>\r\n", std::get<6>(ram1) / 1'024UL /* size */, std::get<1>(ram1) / 1'024UL /* data used */,
                std::get<2>(ram1) / 1'024UL /* bss used */, std::get<3>(ram1) / 1'024UL /* heap used */, std::get<5>(ram1) / 1'024UL /* itcm */));
        const auto ram2 { freertos::ram2_usage() };
        p_comm_->debug_printf<true>(
            PP_ARGS("<sys>ram:2:{}:{}</sys>\r\n", std::get<1>(ram2) / 1'024UL /* size */, (std::get<1>(ram2) - std::get<0>(ram2)) / 1'024UL /* used */));
#ifdef ARDUINO_TEENSY41
        const auto ram3 { freertos::ram3_usage() };
        p_comm_->debug_printf<true>(
            PP_ARGS("<sys>ram:3:{}:{}</sys>\r\n", std::get<1>(ram3) / 1'024UL /* size */, (std::get<1>(ram3) - std::get<0>(ram3)) / 1'024UL /* used */));
#endif // ARDUINO_TEENSY41
    }

    return true;
}

void CtBot::run() {
    if (!ready_) {
        return;
    }

    for (const auto& e : pre_hooks_) {
        if (std::get<1>(e.second)) {
            std::get<0>(e.second)();
        }
    }

    p_sensors_->update();

    // if (CtBotConfig::LUA_AVAILABLE && p_lua_) {
    //     static uint32_t last_lua {};
    //     const auto now { Timer::get_ms() };
    //     if (now - last_lua > 2'000U) {
    //         last_lua = now;

    //         const auto ret { p_lua_->Lua_dostring("print('Hello world!', 42)") };
    //         if (ret.length()) {
    //             p_logger_->log(ret, true);
    //             p_logger_->log("\r\n", true);
    //         }
    //         lua_gc(p_lua_->get_state(), 2, 0);
    //     }
    // }

    for (const auto& e : post_hooks_) {
        if (std::get<1>(e.second)) {
            std::get<0>(e.second)();
        }
    }

    if (shutdown_) {
        shutdown();
    }
}

FLASHMEM bool CtBot::play_wav(const std::string_view& filename) {
    if constexpr (CtBotConfig::AUDIO_AVAILABLE && CtBotConfig::SDCARD_AVAILABLE) {
        if (!p_fs_) {
            return false;
        }

        if (p_play_wav_->isPlaying()) {
            p_logger_->log(PSTR("CtBot::play_wav(): Still playing, abort.\r\n"), false);
            return false;
        }

        const auto str_begin { filename.find_first_not_of(' ') };
        if (str_begin == filename.npos) {
            p_logger_->log(PSTR("CtBot::play_wav(): no file given, abort.\r\n"), false);
            return false;
        }

        const std::string file { filename.substr(str_begin) };
        File wavefile { p_fs_->open(file.c_str()) };
        if (!wavefile) {
            return false;
        }
        const bool res { p_play_wav_->play(wavefile) };

        if (res) {
            p_logger_->log<false>(PP_ARGS("CtBot::play_wav(): Playing file \"{s}\"\r\n", file.c_str()));
        } else {
            p_logger_->log<false>(PP_ARGS("CtBot::play_wav(): File \"{s}\" not found\r\n", file.c_str()));
        }

        return res;
    } else {
        return false;
    }
}

FLASHMEM bool CtBot::file_upload(freertos::SerialIO& io, const std::string_view& file_name, uint32_t file_size, uint32_t crc32, bool textmode) const {
    FS_Service::FileWrapper* p_file_wrapper;

    if (!get_fs()) {
        return false;
    }

    File file { get_fs()->open(std::string(file_name).c_str(), static_cast<uint8_t>(FILE_WRITE), 0, &p_file_wrapper) };
    if (!file) {
        return false;
    }

    if (!file_size) {
        file_size = std::numeric_limits<uint32_t>::max();
    }
    const bool hexfile { file_name.rfind(PSTR(".hex")) != file_name.npos };

    while (DEBUG_LEVEL_ > 2 && !io.available()) {
        ::vTaskDelay(1);
    }
    io.get_rx_overflow(true);
    const auto start { Timer::get_ms() };

    std::string last_line;
    last_line.reserve(46);
    uint32_t len {};
    uint32_t feedback_points {};
    for (; len < file_size;) {
        while (!io.available()) {
            ::vTaskDelay(1);
            if (Timer::get_ms() - start > (file_size / 10 + 1)) { // timeout for less than 10 kB/s
                file.close();
                std::string tmp { file_name.data(), file_name.size() };
                get_fs()->remove(tmp.c_str());
                return false;
            }
        }

        if (textmode) {
            const auto bytes_avail { io.available() };
            const auto n { last_line.size() };
            last_line.resize(last_line.size() + bytes_avail);
            len += io.read(last_line.data() + n, bytes_avail);

            if (last_line.back() == '\n') {
                p_file_wrapper->write(last_line.data(), last_line.size());
                if (hexfile && last_line.find(PSTR(":00000001FF")) != last_line.npos) {
                    break;
                }
                last_line.clear();
            }
        } else {
            const uint8_t tmp = io.read(true);
            ++len;
            p_file_wrapper->write(&tmp, 1);
        }

        if (io.get_rx_overflow(true)) {
            get_logger()->begin("CtBot::file_upload(): ");
            get_logger()->log<true>(PSTR("ERROR: RX overflow\r\n"));
            file.close();
            std::string tmp { file_name.data(), file_name.size() };
            get_fs()->remove(tmp.c_str());
            return false;
        }

        if (len * 100 / file_size >= feedback_points) {
            get_comm()->debug_print('*', false);
            ++feedback_points;
        }
    }
    file.close();
    get_comm()->debug_print(PSTR("\r\n"), true);

    if constexpr (DEBUG_LEVEL_ > 2) {
        const auto dt { Timer::get_ms() - start };
        get_comm()->debug_printf<true>(
            PSTR("upload of %u bytes took %u ms (%.2f kB/s).\r\n"), len, dt, static_cast<float>(len) / 1.024f / static_cast<float>(dt));
    }

    if (crc32) {
        const auto file_crc { file_crc32(file_name) };
        if (file_crc != crc32) {
            get_comm()->debug_printf<true>(PSTR("CRC32 of file: 0x%x\r\nCRC32 requsted: 0x%x\r\n"), file_crc, crc32);
            return false;
        }
    }

    return true;
}

FLASHMEM uint32_t CtBot::file_crc32(const std::string_view& file_name) const {
    FS_Service::FileWrapper* p_file_wrapper;

    if (!get_fs()) {
        return 0;
    }

    File file { get_fs()->open(std::string(file_name).c_str(), static_cast<uint8_t>(FILE_READ), 0, &p_file_wrapper) };
    if (!file) {
        return 0;
    }

    CRC32 crc;
    char buf[32];
    while (file.available()) { // TODO: timeout
        const auto n { file.read(buf, sizeof(buf)) }; // TODO: timeout
        if (n) {
            crc.update(buf, n);
        }
    }
    file.close();
    const auto crc32 { crc.finalize() };

    return crc32;
}

bool CtBot::create_shutdown_timer(uint32_t ms) {
    shutdown_timer_ = ::xTimerCreate(PSTR("shutdown_t"), pdMS_TO_TICKS(ms), false, nullptr, [](TimerHandle_t) {
        if constexpr (DEBUG_LEVEL_ > 1) {
            ::serialport_puts(PSTR("Shutdown timer expired.\r\n"));
        }

        std::exit(1);
    });
    configASSERT(shutdown_timer_);
    xTimerStart(shutdown_timer_, portMAX_DELAY);

    return shutdown_timer_ != nullptr;
}

FLASHMEM void CtBot::shutdown() {
    extern tests::ButtonTest* g_button_test;

    ready_ = false;

    if (!shutdown_timer_) {
        create_shutdown_timer(10'000);
    }

    if constexpr (CtBotConfig::LUA_AVAILABLE) {
        delete p_lua_;
    }

    if constexpr (CtBotConfig::AUDIO_AVAILABLE && CtBotConfig::SDCARD_AVAILABLE) {
        p_play_wav_->stop();
    }

    if constexpr (CtBotConfig::BUTTON_TEST_AVAILABLE) {
        if (g_button_test) {
            delete g_button_test;
            g_button_test = nullptr;
        }
    }

    p_logger_->begin();
    p_logger_->log(PSTR("System shutting down...\r\n"), false);

    p_comm_->flush();
    if (p_serial_wifi_) {
        p_serial_wifi_->flush();
    }
    p_serial_usb_->flush();

    p_speedcontrols_[0]->set_speed(0.f);
    p_speedcontrols_[1]->set_speed(0.f);
    if (p_motors_[0] && p_motors_[1]) {
        p_motors_[0]->set(0);
        p_motors_[1]->set(0);
    }
    p_servos_[0]->disable();
    if (p_servos_[1]) {
        p_servos_[1]->disable();
    }

    if constexpr (CtBotConfig::BUTTON_TEST_AVAILABLE || CtBotConfig::TFT_TEST_AVAILABLE) {
        p_scheduler_->task_remove(p_scheduler_->task_get(PSTR("TFT-Test")));
    }
    p_leds_->set(LedTypes::NONE);
    p_sensors_->disable_all();

    if constexpr (CtBotConfig::LCD_AVAILABLE) {
        p_lcd_->clear();
        p_lcd_->set_backlight(false);
    }
    if constexpr (CtBotConfig::TFT_AVAILABLE) {
        p_tft_->clear();
        p_tft_->set_backlight(0.f);
        p_tft_->flush();
    }

    if (p_clock_timer_) {
        xTimerStop(p_clock_timer_, 0);
        xTimerDelete(p_clock_timer_, 0);
    }

    ::vTaskPrioritySet(nullptr, configMAX_PRIORITIES - 2);

    if constexpr (CtBotConfig::AUDIO_AVAILABLE) {
        if constexpr (CtBotConfig::AUDIO_TEST_AVAILABLE) {
            delete p_audio_sine_;
            if constexpr (DEBUG_LEVEL_ > 1) {
                ::serialport_puts(PSTR("CtBot::shutdown(): p_audio_sine_ deleted.\r\n"));
            }
        }
        p_scheduler_->task_remove(p_scheduler_->task_get(PSTR("audio")));
        if constexpr (DEBUG_LEVEL_ > 1) {
            ::serialport_puts(PSTR("CtBot::shutdown(): audio task removed.\r\n"));
        }
        for (auto& e : p_audio_conn_) {
            delete e;
        }
        if constexpr (DEBUG_LEVEL_ > 1) {
            ::serialport_puts(PSTR("CtBot::shutdown(): p_audio_conn_ deleted.\r\n"));
        }

        for (auto& e : p_audio_mixer_) {
            delete e;
        }
        if constexpr (DEBUG_LEVEL_ > 1) {
            ::serialport_puts(PSTR("CtBot::shutdown(): p_audio_mixer_ deleted.\r\n"));
        }

        if constexpr (CtBotConfig::AUDIO_I2S_AVAILABLE) {
            delete p_audio_output_i2s_;
            if constexpr (DEBUG_LEVEL_ > 1) {
                ::serialport_puts(PSTR("CtBot::shutdown(): p_audio_output_i2s_ deleted.\r\n"));
            }
        }
        if constexpr (CtBotConfig::AUDIO_ANALOG_AVAILABLE) {
            delete p_audio_output_dac_;
            if (DEBUG_LEVEL_ > 1) {
                ::serialport_puts(PSTR("CtBot::shutdown(): p_audio_output_dac_ deleted.\r\n"));
            }
        }

        delete p_tts_;
        if constexpr (DEBUG_LEVEL_ > 1) {
            ::serialport_puts(PSTR("CtBot::shutdown(): p_tts_ deleted.\r\n"));
        }

        delete p_play_wav_;
        if constexpr (DEBUG_LEVEL_ > 1) {
            ::serialport_puts(PSTR("CtBot::shutdown(): p_play_wav_ deleted.\r\n"));
        }
    }

    if constexpr (CtBotConfig::SDCARD_AVAILABLE) {
        if constexpr (CtBotConfig::LOG_TO_SDCARD_AVAILABLE) {
            delete p_logger_file_;
            if constexpr (DEBUG_LEVEL_ > 1) {
                ::serialport_puts(PSTR("p_logger_file_ deleted.\r\n"));
            }
        }
        delete p_parameter_;
        if constexpr (DEBUG_LEVEL_ > 1) {
            ::serialport_puts(PSTR("p_parameter_ deleted.\r\n"));
        }
    }
    if constexpr (CtBotConfig::TFT_AVAILABLE) {
        delete p_tft_;
        if constexpr (DEBUG_LEVEL_ > 1) {
            ::serialport_puts(PSTR("p_tft_ deleted.\r\n"));
        }
    }
    if constexpr (CtBotConfig::LCD_AVAILABLE) {
        delete p_lcd_;
        if constexpr (DEBUG_LEVEL_ > 1) {
            ::serialport_puts(PSTR("p_lcd_ deleted.\r\n"));
        }
    };
    delete p_leds_;
    if constexpr (DEBUG_LEVEL_ > 1) {
        ::serialport_puts(PSTR("p_leds_ deleted.\r\n"));
    }
    delete p_servos_[0];
    delete p_servos_[1];
    if constexpr (DEBUG_LEVEL_ > 1) {
        ::serialport_puts(PSTR("p_servos_ deleted.\r\n"));
    }
    delete p_speedcontrols_[0];
    delete p_speedcontrols_[1];
    if constexpr (DEBUG_LEVEL_ > 1) {
        ::serialport_puts(PSTR("p_speedcontrols_ deleted.\r\n"));
    }
    delete p_motors_[0];
    delete p_motors_[1];
    if constexpr (DEBUG_LEVEL_ > 1) {
        ::serialport_puts(PSTR("p_motors_ deleted.\r\n"));
    }
    delete p_sensors_;
    if constexpr (DEBUG_LEVEL_ > 1) {
        ::serialport_puts(PSTR("p_sensors_ deleted.\r\n"));
    }
    delete p_logger_;
    if constexpr (DEBUG_LEVEL_ > 1) {
        ::serialport_puts(PSTR("p_logger_ deleted.\r\n"));
    }
    delete p_comm_;
    if constexpr (DEBUG_LEVEL_ > 1) {
        ::serialport_puts(PSTR("p_comm_ deleted.\r\n"));
    }
    delete p_parser_;
    if constexpr (DEBUG_LEVEL_ > 1) {
        ::serialport_puts(PSTR("p_parser_ deleted.\r\n"));
    }
    delete p_viewer_tasks_context_;
    delete p_taskstat_context_;
    delete p_scheduler_;
    if constexpr (DEBUG_LEVEL_ > 1) {
        ::serialport_puts(PSTR("p_scheduler_ deleted.\r\n"));
    }

    ::vTaskPrioritySet(nullptr, 2);
    ::xTaskCreate(
        [](void* shtd_timer) {
            auto timer { reinterpret_cast<TimerHandle_t>(shtd_timer) };
            configASSERT(timer);
            xTimerStop(timer, 0);
            xTimerDelete(timer, 0);

            std::exit(0);
        },
        PSTR("EXIT"), 512, shutdown_timer_, 1, nullptr);

    ::vTaskDelete(nullptr);
}

FLASHMEM void CtBot::add_pre_hook(const std::string& name, std::function<void()>&& hook, bool active) {
    pre_hooks_[name] = std::make_tuple(hook, active);
}

FLASHMEM void CtBot::add_post_hook(const std::string& name, std::function<void()>&& hook, bool active) {
    post_hooks_[name] = std::make_tuple(hook, active);
}

FLASHMEM void CtBot::update_clock() {
    std::time_t now;
    std::time(&now);
    if constexpr (DEBUG_LEVEL_ >= 4) {
        p_logger_->begin(PSTR("CtBot::update_clock(): "));
        p_logger_->log<false>(PSTR("now=%d%03d\r\n"), static_cast<uint32_t>(now / 1'000L), static_cast<uint32_t>(now % 1'000L));
    }
    if (clock_update_done_) {
        if (p_clock_timer_) {
            xTimerStop(p_clock_timer_, 0);
            xTimerDelete(p_clock_timer_, 0);
            p_clock_timer_ = nullptr;

            if constexpr (DEBUG_LEVEL_ >= 4) {
                p_logger_->begin(PSTR("CtBot::update_clock(): "));
                p_logger_->log(PSTR("timer disabled\r\n"), false);
            }
        }
        if constexpr (DEBUG_LEVEL_ >= 3) {
            p_logger_->begin(PSTR("CtBot::update_clock(): "));
            p_logger_->log(PSTR("System time updated.\r\n"), false);
        }
        return;
    }

    p_comm_->flush();
    p_comm_->debug_print(PSTR("\eC"), false); // send clock command to wifi controller
}

} // namespace ctbot
