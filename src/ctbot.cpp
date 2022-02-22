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
#include "driver/serial_t3.h"
#include "driver/serial_t4.h"
#include "driver/tft_display.h"

#include "test/tests.h"

#include "lua_wrapper.h"
#include "pprintpp.hpp"
#include "timers.h"

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
TaskHandle_t CtBot::audio_task_ {};

CtBot& CtBot::get_instance() {
    static CtBot* p_instance { CtBotConfig::BEHAVIOR_MODEL_AVAILABLE ? new CtBotBehavior : new CtBot };
    return *p_instance;
}

FLASHMEM CtBot::CtBot()
    // initializes serial connection here for debug purpose
    : shutdown_ {}, ready_ {}, task_id_ {}, p_scheduler_ {}, p_sensors_ {}, p_motors_ { nullptr, nullptr },
      p_speedcontrols_ { nullptr, nullptr }, p_servos_ { nullptr, nullptr }, p_ena_ {}, p_ena_pwm_ {}, p_leds_ {}, p_lcd_ {}, p_tft_ {}, p_cli_ {},
      p_serial_usb_ { &arduino::get_serial(0) }, p_serial_wifi_ {}, p_comm_ {}, p_parser_ {}, p_logger_ {}, p_logger_file_ {}, p_parameter_ {},
      p_audio_output_dac_ {}, p_audio_output_i2s_ {}, p_audio_sine_ {}, p_play_wav_ {}, p_tts_ {}, p_watch_timer_ {}, p_clock_timer_ {}, p_lua_ {} {}

CtBot::~CtBot() {
    if (DEBUG_LEVEL_ > 1) {
        ::serialport_puts(PSTR("CtBot::~CtBot(): destroying CtBot instance.\r\n"));
    }
}

void CtBot::stop() {
    shutdown_ = true;
}

__attribute__((noinline, used)) void terminate_handler() {
    configASSERT(0);
}

FLASHMEM void CtBot::setup(const bool set_ready) {
    if (DEBUG_LEVEL_ > 2) {
        ::serialport_puts(PSTR("CtBot::setup()...\r\n"));
    }
    std::set_terminate(terminate_handler);
    std::atexit([]() FLASHMEM {
        if (DEBUG_LEVEL_ > 1) {
            ::serialport_puts(PSTR("exit() called.\r\n"));
        }

        CtBot* ptr = &get_instance();
        delete ptr;

        if (DEBUG_LEVEL_ > 0) {
            ::serialport_puts(PSTR("exit(): stopping scheduler, bye.\r\n"));
        }

        Scheduler::stop();
    });

    if (DEBUG_LEVEL_ > 2) {
        ::serialport_puts(PSTR("CtBot::setup(): creating scheduler...\r\n"));
    }
    p_scheduler_ = new Scheduler;
    task_id_ = p_scheduler_->task_add(PSTR("main"), TASK_PERIOD_MS, TASK_PRIORITY, STACK_SIZE, [this]() { return run(); });
    p_scheduler_->task_register(PSTR("Tmr Svc"), true);
    p_scheduler_->task_register(PSTR("YIELD"));
    p_scheduler_->task_register(PSTR("EVENT"));

    if (DEBUG_LEVEL_ > 2) {
        ::serialport_puts(PSTR("CtBot::setup(): tasks registered.\r\n"));
    }

    p_cli_ = new CtBotCli { this };
    configASSERT(p_cli_);

    p_parser_ = new CmdParser;
    configASSERT(p_parser_);
    if (CtBotConfig::UART_FOR_CMD != 0) {
        p_serial_wifi_ = &arduino::get_serial(CtBotConfig::UART_FOR_CMD);
        configASSERT(p_serial_wifi_);
        p_serial_wifi_->setRX(CtBotConfig::UART_WIFI_PIN_RX);
        p_serial_wifi_->setTX(CtBotConfig::UART_WIFI_PIN_TX);
        if (!p_serial_wifi_->begin(CtBotConfig::UART_WIFI_BAUDRATE, 0, 64, 256)) {
            p_comm_->debug_print(PSTR("p_serial_wifi_->begin() failed.\r\n"), true);
        }

        p_comm_ = new CommInterfaceCmdParser { *p_serial_wifi_, *p_parser_, true };
        if (CrashReport) {
            p_serial_wifi_->get_stream().print(CrashReport);
        }
    } else {
        p_comm_ = new CommInterfaceCmdParser { *p_serial_usb_, *p_parser_, true };
        if (CrashReport) {
            p_serial_usb_->get_stream().print(CrashReport);
        }
    }
    configASSERT(p_comm_);

    p_logger_ = new Logger;
    configASSERT(p_logger_);
    p_logger_->add_target(p_comm_);

    if (DEBUG_LEVEL_ > 2) {
        ::serialport_puts(PSTR("CtBot::setup(): comm services inited.\r\n"));
    }

    p_ena_ = new EnaI2c { CtBotConfig::ENA_I2C_BUS, CtBotConfig::ENA_I2C_ADDR,
        CtBotConfig::ENA_I2C_BUS == 0 ?
            CtBotConfig::I2C0_FREQ :
            (CtBotConfig::ENA_I2C_BUS == 1 ? CtBotConfig::I2C1_FREQ : (CtBotConfig::ENA_I2C_BUS == 2 ? CtBotConfig::I2C2_FREQ : CtBotConfig::I2C3_FREQ)) };
    configASSERT(p_ena_);
    if (DEBUG_LEVEL_ > 2) {
        ::serialport_puts(PSTR("CtBot::setup(): ENA created.\r\n"));
    }
    p_ena_pwm_ = new LedsI2cEna { CtBotConfig::ENA_PWM_I2C_BUS, CtBotConfig::ENA_PWM_I2C_ADDR,
        CtBotConfig::ENA_PWM_I2C_BUS == 0 ?
            CtBotConfig::I2C0_FREQ :
            (CtBotConfig::ENA_PWM_I2C_BUS == 1 ? CtBotConfig::I2C1_FREQ :
                                                 (CtBotConfig::ENA_PWM_I2C_BUS == 2 ? CtBotConfig::I2C2_FREQ : CtBotConfig::I2C3_FREQ)) };
    configASSERT(p_ena_pwm_);
    if (DEBUG_LEVEL_ > 2) {
        ::serialport_puts(PSTR("CtBot::setup(): ENA PWM created.\r\n"));
    }
    p_ena_pwm_->set_pwm(LedTypesEna::ENC_L | LedTypesEna::ENC_R, 128);
    p_ena_pwm_->off(LedTypesEna::ENC_L | LedTypesEna::ENC_R);
    p_ena_pwm_->on(LedTypesEna::ENC_L | LedTypesEna::ENC_R);
    if (DEBUG_LEVEL_ > 2) {
        ::serialport_puts(PSTR("CtBot::setup(): ENA inited.\r\n"));
    }

    p_sensors_ = new Sensors { *this };
    configASSERT(p_sensors_);
    if (DEBUG_LEVEL_ > 2) {
        ::serialport_puts(PSTR("CtBot::setup(): sensors created.\r\n"));
    }

    p_sensors_->enable_sensors();
    if (DEBUG_LEVEL_ > 2) {
        ::serialport_puts(PSTR("CtBot::setup(): sensors enabled.\r\n"));
    }


    if (CtBotConfig::EXTERNAL_SPEEDCTRL) {
        p_speedcontrols_[0] = new SpeedControlExternal {};
        p_speedcontrols_[1] = new SpeedControlExternal {};
    } else {
        p_motors_[0] = new Motor { p_sensors_->get_enc_l(), CtBotConfig::MOT_L_PWM_PIN, CtBotConfig::MOT_L_DIR_PIN, CtBotConfig::MOT_L_DIR };
        p_motors_[1] = new Motor { p_sensors_->get_enc_r(), CtBotConfig::MOT_R_PWM_PIN, CtBotConfig::MOT_R_DIR_PIN, CtBotConfig::MOT_R_DIR };

        p_speedcontrols_[0] = new SpeedControl { p_sensors_->get_enc_l(), *p_motors_[0] };
        p_speedcontrols_[1] = new SpeedControl { p_sensors_->get_enc_r(), *p_motors_[1] };
    }

    p_servos_[0] = new Servo { CtBotConfig::SERVO_1_PIN };
    if (CtBotConfig::SERVO_2_PIN != 255) {
        p_servos_[1] = new Servo { CtBotConfig::SERVO_2_PIN };
    }

    p_leds_ = new LedsI2c { CtBotConfig::LED_I2C_BUS, CtBotConfig::LED_I2C_ADDR,
        CtBotConfig::LED_I2C_BUS == 0 ?
            CtBotConfig::I2C0_FREQ :
            (CtBotConfig::LED_I2C_BUS == 1 ? CtBotConfig::I2C1_FREQ : (CtBotConfig::LED_I2C_BUS == 2 ? CtBotConfig::I2C2_FREQ : CtBotConfig::I2C3_FREQ)) };

    if (CtBotConfig::LCD_AVAILABLE) {
        p_lcd_ = new LCDisplay;
    }
    if (CtBotConfig::TFT_AVAILABLE) {
        p_tft_ = new TFTDisplay;
    } else if (CtBotConfig::TFT_BACKLIGHT_PIN != 255) {
        arduino::pinMode(CtBotConfig::TFT_BACKLIGHT_PIN, arduino::OUTPUT);
        arduino::digitalWriteFast(CtBotConfig::TFT_BACKLIGHT_PIN, true);
    }

    if (DEBUG_LEVEL_ > 2) {
        ::serialport_puts(PSTR("CtBot::setup(): SD-card init...\r\n"));
    }
    if (CtBotConfig::SDCARD_AVAILABLE) {
        if (SD.sdfs.begin(SdioConfig(CtBotConfig::SDCARD_USE_DMA ? DMA_SDIO : FIFO_SDIO))) {
            p_parameter_ = new ParameterStorage { SD, PSTR("ctbot.jsn") };
            configASSERT(p_parameter_);
            p_logger_file_ = new LoggerTargetFile { SD, PSTR("logs.txt"), this };
            configASSERT(p_logger_file_);
            p_logger_->add_target(p_logger_file_);
        } else {
            p_logger_->log(PSTR("SD.sdfs.begin() failed.\r\n"), true);
        }
    }
    if (DEBUG_LEVEL_ > 2) {
        ::serialport_puts(PSTR("CtBot::setup(): SD Card init done.\r\n"));
    }


    if (CtBotConfig::AUDIO_AVAILABLE) {
        portENTER_CRITICAL();
        p_play_wav_ = new AudioPlaySdWav;
        configASSERT(p_play_wav_);
        p_tts_ = new TTS;
        configASSERT(p_tts_);
        if (CtBotConfig::AUDIO_I2S_AVAILABLE) {
            p_audio_output_i2s_ = new AudioOutputI2S;
            configASSERT(p_audio_output_i2s_);
        }
        if (CtBotConfig::AUDIO_ANALOG_AVAILABLE) {
            p_audio_output_dac_ = new AudioOutputAnalog;
            configASSERT(p_audio_output_dac_);
        }
        if (CtBotConfig::AUDIO_TEST_AVAILABLE) {
            p_audio_sine_ = new AudioSynthWaveformSine;
            configASSERT(p_audio_sine_);
            p_audio_sine_->frequency(0.f);
            p_audio_sine_->amplitude(.5f);
        }

        for (uint8_t i {}; i < CtBotConfig::AUDIO_CHANNELS; ++i) {
            p_audio_mixer_.push_back(new AudioMixer4);
            configASSERT(*p_audio_mixer_.rbegin());
        }
        for (auto& e : p_audio_mixer_) {
            e->gain(0, 0.1f);
            e->gain(1, 0.1f);
            e->gain(2, 0.1f);
        }

        for (uint8_t i {}; i < CtBotConfig::AUDIO_CHANNELS; ++i) {
            p_audio_conn_.push_back(new AudioConnection { *p_play_wav_, i, *p_audio_mixer_[i], 0 });
            configASSERT(*p_audio_conn_.rbegin());
            if (CtBotConfig::AUDIO_I2S_AVAILABLE) {
                p_audio_conn_.push_back(new AudioConnection { *p_audio_mixer_[i], 0, *p_audio_output_i2s_, i });
            } else if (CtBotConfig::AUDIO_ANALOG_AVAILABLE) {
                p_audio_conn_.push_back(new AudioConnection { *p_audio_mixer_[i], 0, *p_audio_output_dac_, i });
            }
            configASSERT(*p_audio_conn_.rbegin());
        }
        p_audio_conn_.push_back(new AudioConnection { *p_tts_, 0, *p_audio_mixer_[0], 1 });
        if (CtBotConfig::AUDIO_TEST_AVAILABLE) {
            p_audio_conn_.push_back(new AudioConnection { *p_audio_sine_, 0, *p_audio_mixer_[0], 2 });
            configASSERT(*p_audio_conn_.rbegin());
        }

        AudioMemory(CtBotConfig::AUDIO_CHANNELS * 2 + CtBotConfig::AUDIO_TEST_AVAILABLE ? 4 : 2);

        get_scheduler()->task_add(PSTR("audio"), 1, Scheduler::MAX_PRIORITY, 4096, [this]() {
            while (get_ready()) {
                if (::ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(500)) == 1) {
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
                    ::vTaskNotifyGiveFromISR(audio_task_, &higher_woken);
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

    p_scheduler_->task_register(PSTR("I2C Svc"));

    if (CtBotConfig::LUA_AVAILABLE) {
        p_lua_ = new LuaWrapper;
        configASSERT(p_lua_);
        lua_register(p_lua_->get_state(), PSTR("print"), lua_wrapper_print);
    }

    p_cli_->init_commands();
    if (DEBUG_LEVEL_ > 2) {
        ::serialport_puts(PSTR("CtBot::setup(): p_cli_->init_commands() done.\r\n"));
    }

    update_clock();
    p_clock_timer_ = ::xTimerCreate(PSTR("clock1_t"), pdMS_TO_TICKS(5'000UL), true, this, [](TimerHandle_t handle) {
        auto p_ctbot { static_cast<CtBot*>(::pvTimerGetTimerID(handle)) };
        configASSERT(p_ctbot);

        ::xTimerChangePeriod(handle, pdMS_TO_TICKS(60'000UL), 0);

        p_ctbot->update_clock();
    });
    ::xTimerStart(p_clock_timer_, 0);

    add_post_hook(
        PSTR("task"),
        [this]() FLASHMEM {
            static uint32_t last_ms {};
            const auto now { Timer::get_ms() };
            if (now - last_ms > 1'000U) {
                last_ms = now;

                auto p_runtime_stats { p_scheduler_->get_runtime_stats() };
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

    p_logger_->log(PSTR("\r\n"), true);
    p_logger_->begin();
    p_logger_->log(PSTR("*** ct-Bot init done. Running FreeRTOS kernel " tskKERNEL_VERSION_NUMBER ". ***\r\n"), true);
    p_logger_->flush();
    p_comm_->debug_print(PSTR("\r\nType \"help\" (or \"h\") to print help message.\r\n\n"), true);
    p_comm_->flush();

    ready_ = set_ready;

    if (DEBUG_LEVEL_ > 2) {
        ::serialport_puts(PSTR("CtBot::setup() done.\r\n"));
    }
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
    //     if (now - last_lua > 2000U) {
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
    if (CtBotConfig::AUDIO_AVAILABLE) {
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
        const bool res { p_play_wav_->play(file.c_str()) };

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

FLASHMEM void CtBot::shutdown() {
    ready_ = false;

    if (CtBotConfig::LUA_AVAILABLE) {
        delete p_lua_;
    }

    if (CtBotConfig::AUDIO_AVAILABLE) {
        p_play_wav_->stop();
    }

    p_logger_->log(PSTR("System shutting down...\r\n"), false);
    p_logger_->flush();
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
    if (CtBotConfig::BUTTON_TEST_AVAILABLE) {
        p_scheduler_->task_remove(p_scheduler_->task_get(PSTR("TFT-Test")));
    }
    if (CtBotConfig::LCD_AVAILABLE) {
        p_lcd_->clear();
        p_lcd_->set_backlight(false);
    }
    if (CtBotConfig::TFT_AVAILABLE) {
        p_tft_->clear();
        p_tft_->set_backlight(0);
    }
    p_leds_->set(LedTypes::NONE);
    p_sensors_->disable_all();

    ::vTaskPrioritySet(nullptr, configMAX_PRIORITIES - 1);

    if (CtBotConfig::AUDIO_AVAILABLE) {
        if (CtBotConfig::AUDIO_TEST_AVAILABLE) {
            delete p_audio_sine_;
            if (DEBUG_LEVEL_ > 1) {
                ::serialport_puts(PSTR("CtBot::shutdown(): p_audio_sine_ deleted.\r\n"));
            }
        }
        p_scheduler_->task_remove(p_scheduler_->task_get(PSTR("audio")));
        if (DEBUG_LEVEL_ > 1) {
            ::serialport_puts(PSTR("CtBot::shutdown(): audio task removed.\r\n"));
        }
        for (auto& e : p_audio_conn_) {
            delete e;
        }
        if (DEBUG_LEVEL_ > 1) {
            ::serialport_puts(PSTR("CtBot::shutdown(): p_audio_conn_ deleted.\r\n"));
        }

        for (auto& e : p_audio_mixer_) {
            delete e;
        }
        if (DEBUG_LEVEL_ > 1) {
            ::serialport_puts(PSTR("CtBot::shutdown(): p_audio_mixer_ deleted.\r\n"));
        }

        if (CtBotConfig::AUDIO_I2S_AVAILABLE) {
            delete p_audio_output_i2s_;
            if (DEBUG_LEVEL_ > 1) {
                ::serialport_puts(PSTR("CtBot::shutdown(): p_audio_output_i2s_ deleted.\r\n"));
            }
        }
        if (CtBotConfig::AUDIO_ANALOG_AVAILABLE) {
            delete p_audio_output_dac_;
            if (DEBUG_LEVEL_ > 1) {
                ::serialport_puts(PSTR("CtBot::shutdown(): p_audio_output_dac_ deleted.\r\n"));
            }
        }

        delete p_tts_;
        if (DEBUG_LEVEL_ > 1) {
            ::serialport_puts(PSTR("CtBot::shutdown(): p_tts_ deleted.\r\n"));
        }

        delete p_play_wav_;
        if (DEBUG_LEVEL_ > 1) {
            ::serialport_puts(PSTR("CtBot::shutdown(): p_play_wav_ deleted.\r\n"));
        }
    }

    if (CtBotConfig::SDCARD_AVAILABLE) {
        delete p_logger_file_;
        if (DEBUG_LEVEL_ > 1) {
            ::serialport_puts(PSTR("p_logger_file_ deleted.\r\n"));
        }
        delete p_parameter_;
        if (DEBUG_LEVEL_ > 1) {
            ::serialport_puts(PSTR("p_parameter_ deleted.\r\n"));
        }
    }
    if (CtBotConfig::TFT_AVAILABLE) {
        delete p_tft_;
        if (DEBUG_LEVEL_ > 1) {
            ::serialport_puts(PSTR("p_tft_ deleted.\r\n"));
        }
    }
    if (CtBotConfig::LCD_AVAILABLE) {
        delete p_lcd_;
        if (DEBUG_LEVEL_ > 1) {
            ::serialport_puts(PSTR("p_lcd_ deleted.\r\n"));
        }
    };
    delete p_leds_;
    if (DEBUG_LEVEL_ > 1) {
        ::serialport_puts(PSTR("p_leds_ deleted.\r\n"));
    }
    delete p_servos_[0];
    delete p_servos_[1];
    if (DEBUG_LEVEL_ > 1) {
        ::serialport_puts(PSTR("p_servos_ deleted.\r\n"));
    }
    delete p_speedcontrols_[0];
    delete p_speedcontrols_[1];
    if (DEBUG_LEVEL_ > 1) {
        ::serialport_puts(PSTR("p_speedcontrols_ deleted.\r\n"));
    }
    delete p_motors_[0];
    delete p_motors_[1];
    if (DEBUG_LEVEL_ > 1) {
        ::serialport_puts(PSTR("p_motors_ deleted.\r\n"));
    }
    delete p_sensors_;
    if (DEBUG_LEVEL_ > 1) {
        ::serialport_puts(PSTR("p_sensors_ deleted.\r\n"));
    }
    delete p_logger_;
    if (DEBUG_LEVEL_ > 1) {
        ::serialport_puts(PSTR("p_logger_ deleted.\r\n"));
    }
    delete p_comm_;
    if (DEBUG_LEVEL_ > 1) {
        ::serialport_puts(PSTR("p_comm_ deleted.\r\n"));
    }
    delete p_parser_;
    if (DEBUG_LEVEL_ > 1) {
        ::serialport_puts(PSTR("p_parser_ deleted.\r\n"));
    }
    delete p_scheduler_;
    if (DEBUG_LEVEL_ > 1) {
        ::serialport_puts(PSTR("p_scheduler_ deleted.\r\n"));
    }

    ::vTaskPrioritySet(nullptr, 2);
    xTaskCreate([](void*) { std::exit(0); }, PSTR("EXIT"), 512, nullptr, 1, nullptr);

    ::vTaskDelete(nullptr);
}

FLASHMEM void CtBot::add_pre_hook(const std::string& name, std::function<void()>&& hook, bool active) {
    pre_hooks_[name] = std::make_tuple(hook, active);
}

FLASHMEM void CtBot::add_post_hook(const std::string& name, std::function<void()>&& hook, bool active) {
    post_hooks_[name] = std::make_tuple(hook, active);
}

FLASHMEM void CtBot::update_clock() {
    struct Data {
        const CtBot* p_ctbot;
        bool old_echo;
    };

    std::time_t now;
    std::time(&now);
    if (DEBUG_LEVEL_ >= 4) {
        p_comm_->debug_printf<true>(PSTR("CtBot::update_clock(): now=%d%03d\r\n"), static_cast<uint32_t>(now / 1'000L), static_cast<uint32_t>(now % 1'000L));
    }
    if (now > 1'600'000'000LL) { // 2020-09
        if (p_clock_timer_) {
            xTimerStop(p_clock_timer_, 0);
            xTimerDelete(p_clock_timer_, 0);
            p_clock_timer_ = nullptr;

            if (DEBUG_LEVEL_ >= 4) {
                p_comm_->debug_print(PSTR("CtBot::update_clock(): timer disabled\r\n"), true);
            }
        }
        return;
    }

    Data* p_data { new Data { this, p_comm_->get_echo() } };
    configASSERT(p_data);

    p_comm_->flush();
    p_comm_->set_echo(false);
    p_comm_->debug_print(PSTR("\eC"), true); // send clock command to esp
    p_comm_->flush();

    auto timer2 = ::xTimerCreate(PSTR("clock2_t"), pdMS_TO_TICKS(50), false, p_data, [](TimerHandle_t handle) {
        auto p_data { static_cast<Data*>(::pvTimerGetTimerID(handle)) };
        configASSERT(p_data);

        p_data->p_ctbot->p_comm_->set_echo(p_data->old_echo);

        delete p_data;
        xTimerStop(handle, 0);
        xTimerDelete(handle, 0);

        if (DEBUG_LEVEL_ >= 3) {
            p_data->p_ctbot->p_comm_->debug_print(PSTR("CtBot::update_clock() done.\r\n"), true);
        }
    });
    ::xTimerStart(timer2, 0);
}

} // namespace ctbot
