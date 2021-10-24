/*
 * This file is part of the c't-Bot teensy framework.
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
 * @brief   Main class of c't-Bot teensy framework
 * @author  Timo Sandmann
 * @date    13.05.2018
 */

#include "ctbot.h"
#include "behavior/ctbot_behavior.h"
#include "ctbot_config.h"
#include "scheduler.h"
#include "leds_i2c.h"
#include "lc_display.h"
#include "tft_display.h"
#include "ena_i2c.h"
#include "sensors.h"
#include "mpu_6050.h"
#include "motor.h"
#include "speed_control.h"
#include "servo.h"
#include "help_texts.h"
#include "cmd_parser.h"
#include "cmd_script.h"
#include "parameter_storage.h"
#include "i2c_service.h"
#include "serial_io.h"
#include "serial_t3.h"
#include "serial_t4.h"
#include "tests.h"

#include "timers.h"
#include "pprintpp.hpp"
#include "lua_wrapper.h"

#include <cstdlib>
#include <cinttypes>
#include <string>
#include <charconv>
#include <cstring>
#include <array>
#include <tuple>
#include <thread>
#include <chrono>

#ifndef sei
#define sei() __enable_irq() // for Audio.h
#endif
#ifndef cli
#define cli() __disable_irq() // for Audio.h
#endif

#include "Audio.h"
#include "SD.h"
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
      p_speedcontrols_ { nullptr, nullptr }, p_servos_ { nullptr, nullptr }, p_ena_ {}, p_ena_pwm_ {}, p_leds_ {}, p_lcd_ {}, p_tft_ {},
      p_serial_usb_ { &arduino::get_serial(0) }, p_serial_wifi_ {}, p_comm_ {}, p_parser_ {}, p_parameter_ {}, p_audio_output_dac_ {}, p_audio_output_i2s_ {},
      p_audio_sine_ {}, p_play_wav_ {}, p_tts_ {}, p_watch_timer_ {}, p_lua_ {} {}

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
        } else {
            p_comm_->debug_print(PSTR("SD.sdfs.begin() failed.\r\n"), true);
        }
    }
    if (DEBUG_LEVEL_ > 2) {
        ::serialport_puts(PSTR("CtBot::setup(): SD-card init done.\r\n"));
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

    init_parser();
    if (DEBUG_LEVEL_ > 2) {
        ::serialport_puts(PSTR("CtBot::setup(): init_parser() done.\r\n"));
    }

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

    p_comm_->debug_print(PSTR("\r\n*** ct-Bot init done. Running FreeRTOS kernel " tskKERNEL_VERSION_NUMBER ". ***\r\n"), true);
    p_comm_->flush();
    p_comm_->debug_print(PSTR("\r\nType \"help\" (or \"h\") to print help message.\r\n\n"), true);
    p_comm_->flush();

    ready_ = set_ready;

    if (DEBUG_LEVEL_ > 2) {
        ::serialport_puts(PSTR("CtBot::setup() done.\r\n"));
    }
}

FLASHMEM void CtBot::init_parser() {
    CtBotHelpTexts::init();
    p_parser_->register_cmd(PSTR("help"), 'h', [this](const std::string_view&) FLASHMEM {
        CtBotHelpTexts::print(*p_comm_);
        return true;
    });

    p_parser_->register_cmd(PSTR("halt"), [this](const std::string_view&) {
        stop();
        return true;
    });

    p_parser_->register_cmd(PSTR("watch"), 'w', [this](const std::string_view& args) {
        if (args.size()) {
            auto p_cmd { new std::string { args } };
            if (p_watch_timer_) {
                auto ptr { static_cast<std::string*>(::pvTimerGetTimerID(p_watch_timer_)) };
                xTimerStop(p_watch_timer_, 0);
                delete ptr;
                xTimerDelete(p_watch_timer_, 0);
            }
            p_watch_timer_ = ::xTimerCreate(PSTR("watch_t"), pdMS_TO_TICKS(1'000UL), true, p_cmd, [](TimerHandle_t handle) {
                auto& ctbot { CtBot::get_instance() };
                auto ptr { static_cast<std::string*>(::pvTimerGetTimerID(handle)) };
                ctbot.get_cmd_parser()->execute_cmd(*ptr, *ctbot.get_comm());
            });
            if (!p_watch_timer_) {
                delete p_cmd;
                return false;
            }
            ::xTimerStart(p_watch_timer_, 0);
        } else {
            if (!p_watch_timer_) {
                return false;
            }
            auto ptr { static_cast<std::string*>(::pvTimerGetTimerID(p_watch_timer_)) };
            xTimerStop(p_watch_timer_, 0);
            delete ptr;
            xTimerDelete(p_watch_timer_, 0);
            p_watch_timer_ = nullptr;
        }
        return true;
    });

    p_parser_->register_cmd(PSTR("config"), 'c', [this](const std::string_view& args) {
        if (args.find(PSTR("echo")) == 0) {
            uint8_t v;
            CmdParser::split_args(args, v);
            p_comm_->set_echo(v);
        } else if (args.find(PSTR("task")) == 0) {
            const size_t s { args.find(' ') + 1 };
            const size_t e { args.find(' ', s) };
            const uint16_t task_id { get_scheduler()->task_get(args.substr(s, e - s)) };

            if (task_id < 0xffff) {
                uint8_t v;
                CmdParser::split_args(args.substr(s), v);
                if (!v) {
                    get_scheduler()->task_suspend(task_id);
                } else {
                    get_scheduler()->task_resume(task_id);
                }
            } else {
                return false;
            }
        } else if (args.find(PSTR("prehook")) == 0) {
            const size_t s { args.find(' ') + 1 };
            const size_t e { args.find(' ', s) };
            const std::string_view hookname { args.substr(s, e - s) };
            uint8_t v;
            CmdParser::split_args(args.substr(s), v);
            auto it { pre_hooks_.find(hookname) };
            if (it != pre_hooks_.end()) {
                std::get<1>(it->second) = v;
            } else {
                return false;
            }
        } else if (args.find(PSTR("posthook")) == 0) { // FIXME: unify with above?
            const size_t s { args.find(' ') + 1 };
            const size_t e { args.find(' ', s) };
            const std::string_view hookname { args.substr(s, e - s) };
            uint8_t v;
            CmdParser::split_args(args.substr(s), v);
            auto it { post_hooks_.find(hookname) };
            if (it != post_hooks_.end()) {
                std::get<1>(it->second) = v;
            } else {
                return false;
            }
        } else if (args.find(PSTR("kp")) == 0) {
            int16_t left, right;
            CmdParser::split_args(args, left, right);
            p_speedcontrols_[0]->set_parameters(static_cast<float>(left), p_speedcontrols_[0]->get_ki(), p_speedcontrols_[0]->get_kd());
            p_speedcontrols_[1]->set_parameters(static_cast<float>(right), p_speedcontrols_[1]->get_ki(), p_speedcontrols_[1]->get_kd());
        } else if (args.find(PSTR("ki")) == 0) {
            int16_t left, right;
            CmdParser::split_args(args, left, right);
            p_speedcontrols_[0]->set_parameters(p_speedcontrols_[0]->get_kp(), static_cast<float>(left), p_speedcontrols_[0]->get_kd());
            p_speedcontrols_[1]->set_parameters(p_speedcontrols_[1]->get_kp(), static_cast<float>(right), p_speedcontrols_[1]->get_kd());
        } else if (args.find(PSTR("kd")) == 0) {
            int16_t left, right;
            CmdParser::split_args(args, left, right);
            p_speedcontrols_[0]->set_parameters(p_speedcontrols_[0]->get_kp(), p_speedcontrols_[0]->get_ki(), static_cast<float>(left));
            p_speedcontrols_[1]->set_parameters(p_speedcontrols_[1]->get_kp(), p_speedcontrols_[1]->get_ki(), static_cast<float>(right));
        } else if (CtBotConfig::LCD_AVAILABLE && args.find(PSTR("lcdout")) != args.npos) {
            const size_t s { args.find(' ') + 1 };
            const size_t e { args.find(' ', s) };
            p_lcd_->set_output(args.substr(s, e - s));
        } else if (args.find(PSTR("led")) == 0) {
            uint8_t mask, pwm;
            CmdParser::split_args(args, mask, pwm);
            const auto led_mask { static_cast<LedTypes>(mask) };
            p_leds_->set_pwm(led_mask, pwm);
            if ((p_leds_->get() & led_mask) != LedTypes::NONE) { // FIXME: check for individual LEDs
                p_leds_->off(led_mask);
                p_leds_->on(led_mask);
            }
        } else if (args.find(PSTR("enapwm")) == 0) {
            uint8_t mask, pwm;
            CmdParser::split_args(args, mask, pwm);
            p_ena_pwm_->set_pwm(static_cast<LedTypesEna>(mask), pwm);
        } else {
            return false;
        }
        return true;
    });

    p_parser_->register_cmd(PSTR("get"), 'g', [this](const std::string_view& args) {
        if (args.find(PSTR("dist")) == 0) {
            p_comm_->debug_printf<true>(PP_ARGS("{} {}", p_sensors_->get_distance_l(), p_sensors_->get_distance_r()));
        } else if (args.find(PSTR("enc")) == 0) {
            p_comm_->debug_printf<true>(PP_ARGS("{} {}", p_sensors_->get_enc_l().get(), p_sensors_->get_enc_r().get()));
        } else if (args.find(PSTR("border")) == 0) {
            p_comm_->debug_printf<true>(PP_ARGS("{} {}", p_sensors_->get_border_l(), p_sensors_->get_border_r()));
        } else if (args.find(PSTR("line")) == 0) {
            p_comm_->debug_printf<true>(PP_ARGS("{} {}", p_sensors_->get_line_l(), p_sensors_->get_line_r()));
        } else if (args.find(PSTR("ldr")) == 0) {
            p_comm_->debug_printf<true>(PP_ARGS("{} {}", p_sensors_->get_ldr_l(), p_sensors_->get_ldr_r()));
        } else if (args.find(PSTR("speed")) == 0) {
            const auto l { static_cast<int16_t>(p_sensors_->get_enc_l().get_speed()) };
            const auto r { static_cast<int16_t>(p_sensors_->get_enc_r().get_speed()) };
            p_comm_->debug_printf<true>(PP_ARGS("{} {}", l, r));
        } else if (args.find(PSTR("mpu")) == 0 && p_sensors_->get_mpu6050()) {
            auto [e1, e2, e3] = p_sensors_->get_mpu6050()->get_euler();
            auto [y, p, r] = p_sensors_->get_mpu6050()->get_ypr();
            p_comm_->debug_printf<true>(PP_ARGS("  {9.4}   {9.4}   {9.4}\r\n", e1, e2, e3));
            p_comm_->debug_printf<true>(PP_ARGS("y={9.4} p={9.4} r={9.4}\r\n", y, p, r));
        } else if (args.find(PSTR("motor")) == 0) {
            if (p_motors_[0] && p_motors_[1]) {
                p_comm_->debug_printf<true>(PP_ARGS("{} {}", p_motors_[0]->get(), p_motors_[1]->get()));
            }
        } else if (args.find(PSTR("servo")) == 0) {
            p_comm_->debug_printf<true>(PP_ARGS("{}[{s}] ", p_servos_[0]->get_position(), p_servos_[0]->get_active() ? PSTR("on ") : PSTR("off")));
            if (p_servos_[1]) {
                p_comm_->debug_printf<true>(PP_ARGS("{}[{s}]", p_servos_[1]->get_position(), p_servos_[1]->get_active() ? PSTR("on") : PSTR("off")));
            }
        } else if (args.find(PSTR("rc5")) == 0) {
            p_comm_->debug_printf<true>(
                PP_ARGS("{} {} {}", p_sensors_->get_rc5().get_addr(), p_sensors_->get_rc5().get_cmd(), p_sensors_->get_rc5().get_toggle()));
        } else if (args.find(PSTR("transmm")) == 0) {
            p_comm_->debug_print(p_sensors_->get_transport_mm(), true);
        } else if (args.find(PSTR("trans")) == 0) {
            p_comm_->debug_print(p_sensors_->get_transport(), true);
            // } else if (args.find("door") == 0) {
            //     p_comm_->debug_print(p_sensors_->get_shutter(), true);
        } else if (args.find(PSTR("led")) == 0) {
            p_comm_->debug_printf<true>(PP_ARGS("{#x}", static_cast<uint8_t>(p_leds_->get())));
        } else if (args.find(PSTR("volt")) == 0) {
            p_comm_->debug_printf<true>(PP_ARGS("{.2} V", p_sensors_->get_bat_voltage()));
        } else if (args.find(PSTR("tasks")) == 0) {
            get_scheduler()->print_task_list(*p_comm_);
        } else if (args.find(PSTR("free")) == 0) {
            get_scheduler()->print_ram_usage(*p_comm_);
            const auto mem_use { AudioMemoryUsage() };
            const auto mem_use_max { AudioMemoryUsageMax() };
            p_comm_->debug_printf<true>(PP_ARGS("\r\nAudioMemoryUsage()={} blocks\tmax={} blocks\r\n", mem_use, mem_use_max));
            const float cpu_use { AudioProcessorUsage() };
            const float cpu_use_max { AudioProcessorUsageMax() };
            p_comm_->debug_printf<true>(PP_ARGS("AudioProcessorUsage()={.2} %%\tmax={.2} %%", cpu_use, cpu_use_max));
        } else if (args.find(PSTR("params")) == 0) {
            auto dump { p_parameter_->dump() };
            p_comm_->debug_printf<true>(PP_ARGS("dump=\"{s}\"", dump->c_str()));
        } else if (args.find(PSTR("paramf")) == 0) {
            size_t s { args.find(' ') };
            if (s == args.npos) {
                return false;
            }
            ++s;
            const size_t e { args.find(' ', s) };
            const std::string_view param { args.substr(s, e - s) };
            float x;
            if (p_parameter_->get(param, x)) {
                p_comm_->debug_printf<true>(PSTR("paramf \"%.*s\"=%f"), param.size(), param.data(), x);
            } else {
                return false;
            }
        } else if (args.find(PSTR("param")) == 0) {
            size_t s { args.find(' ') };
            if (s == args.npos) {
                return false;
            }
            ++s;
            const size_t e { args.find(' ', s) };
            const std::string_view param { args.substr(s, e - s) };
            int32_t x;
            if (p_parameter_->get(param, x)) {
                p_comm_->debug_printf<true>(PSTR("param \"%.*s\"=%" PRId32), param.size(), param.data(), x);
            } else {
                return false;
            }
        } else {
            return false;
        }
        p_comm_->debug_print(PSTR("\r\n"), true);
        return true;
    });

    p_parser_->register_cmd(PSTR("set"), 's', [this](const std::string_view& args) {
        if (args.find(PSTR("speed")) == 0) {
            const auto n { args.find(' ') };
            if (n == args.npos) {
                p_speedcontrols_[0]->set_speed(0.f);
                p_speedcontrols_[1]->set_speed(0.f);
                return false;
            }
            char* p_end;
            const float left { std::strtof(args.data() + n, &p_end) };
            const float right { std::strtof(p_end, nullptr) };
            p_speedcontrols_[0]->set_speed(left);
            p_speedcontrols_[1]->set_speed(right);
        } else if (args.find(PSTR("motor")) == 0) {
            if (p_motors_[0] && p_motors_[1]) {
                int16_t left, right;
                CmdParser::split_args(args, left, right);
                p_motors_[0]->set(left);
                p_motors_[1]->set(right);
            }
        } else if (args.find(PSTR("servo")) == 0) {
            uint8_t s1, s2;
            CmdParser::split_args(args, s1, s2);
            if (!s2) {
                if (args.find(' ', 7) == args.npos) {
                    /* s2 not set */
                    s2 = 255U;
                }
                if (!s1) {
                    if (args.length() < 7) {
                        /* s1 not set */
                        s1 = 255U;
                    }
                }
            }
            if (s1 <= 180) {
                p_servos_[0]->set(s1);
            } else {
                p_servos_[0]->disable();
            }
            if (p_servos_[1]) {
                if (s2 <= 180) {
                    p_servos_[1]->set(s2);
                } else {
                    p_servos_[1]->disable();
                }
            }
        } else if (args.find(PSTR("enapwm")) == 0) {
            uint8_t pin;
            bool value;
            CmdParser::split_args(args, pin, value);

            if (value) {
                p_ena_pwm_->on(static_cast<LedTypesEna>(1 << pin));
            } else {
                p_ena_pwm_->off(static_cast<LedTypesEna>(1 << pin));
            }
        } else if (args.find(PSTR("ena")) == 0) {
            uint8_t pin;
            bool value;
            CmdParser::split_args(args, pin, value);

            if (value) {
                p_ena_->on(static_cast<EnaI2cTypes>(1 << pin));
            } else {
                p_ena_->off(static_cast<EnaI2cTypes>(1 << pin));
            }
        } else if (args.find(PSTR("led")) == 0) {
            uint8_t led;
            CmdParser::split_args(args, led);
            p_leds_->set(static_cast<LedTypes>(led));
        } else if (CtBotConfig::LCD_AVAILABLE && args.find(PSTR("lcdbl")) == 0) {
            bool v;
            CmdParser::split_args(args, v);
            p_lcd_->set_backlight(v);
        } else if (CtBotConfig::LCD_AVAILABLE && args.find(PSTR("lcd")) == 0) {
            uint8_t line, column;
            auto sv { CmdParser::split_args(args, line, column) };
            if (!line && !column) {
                p_lcd_->clear();
                return true;
            }
            p_lcd_->set_cursor(line, column);
            if (sv.empty()) {
                return false;
            }
            p_lcd_->print(sv.substr(1));
        } else if (CtBotConfig::TFT_AVAILABLE && args.find(PSTR("tftbl")) == 0) {
            const auto n { args.find(' ') };
            if (n == args.npos) {
                return false;
            }
            const float v { std::strtof(args.data() + n, nullptr) };
            p_tft_->set_backlight(v);
        } else if (args.find(PSTR("paramf")) == 0) {
            size_t s { args.find(' ') };
            if (s == args.npos) {
                return false;
            }
            ++s;
            const size_t e { args.find(' ', s) };
            if (e == args.npos) {
                return false;
            }

            const std::string_view key { args.substr(s, e - s) };
            p_comm_->debug_printf<true>(PSTR("key=\"%.*s\"\r\n"), key.size(), key.data());

            const std::string_view val { args.substr(e + 1) };
            p_comm_->debug_printf<true>(PSTR("val=\"%.*s\"\r\n"), val.size(), val.data());

            const float value { std::strtof(val.data(), nullptr) };
            p_comm_->debug_printf<true>(PP_ARGS("value={}\r\n", value));

            p_parameter_->set<float>(key, value);
            p_parameter_->flush();
        } else if (args.find(PSTR("param")) == 0) {
            size_t s { args.find(' ') };
            if (s == args.npos) {
                return false;
            }
            ++s;
            const size_t e { args.find(' ', s) };
            if (e == args.npos) {
                return false;
            }

            const std::string_view key { args.substr(s, e - s) };
            p_comm_->debug_printf<true>(PSTR("key=\"%.*s\"\r\n"), key.size(), key.data());

            const std::string_view val { args.substr(e + 1) };
            p_comm_->debug_printf<true>(PSTR("val=\"%.*s\"\r\n"), val.size(), val.data());

            int32_t value {};
            std::from_chars(val.cbegin(), val.cend(), value);
            p_comm_->debug_printf<true>(PP_ARGS("value={}\r\n", value));

            p_parameter_->set<int32_t>(key, value);
            p_parameter_->flush();
        } else {
            return false;
        }
        return true;
    });

    if (CtBotConfig::AUDIO_AVAILABLE) {
        p_parser_->register_cmd(PSTR("audio"), 'a', [this](const std::string_view& args) {
            if (args.find(PSTR("play")) == 0) {
                const size_t s { args.find(' ') + 1 };
                const size_t e { args.find(' ', s) };
                return play_wav(args.substr(s, e - s));
            } else if (args.find(PSTR("on")) == 0) {
                get_ena()->on(EnaI2cTypes::AUDIO);
                using namespace std::chrono_literals;
                std::this_thread::sleep_for(1'500ms);
            } else if (args.find(PSTR("off")) == 0) {
                get_ena()->off(EnaI2cTypes::AUDIO);
                p_play_wav_->stop();
                if (CtBotConfig::AUDIO_TEST_AVAILABLE) {
                    p_audio_sine_->frequency(0.f);
                }
            } else if (args.find(PSTR("vol")) == 0) {
                const auto n { args.find(' ') };
                if (n == args.npos) {
                    return false;
                }
                const float volume { std::strtof(args.data() + n, nullptr) };
                for (auto& e : p_audio_mixer_) {
                    e->gain(0, volume);
                    e->gain(1, volume);
                    e->gain(2, volume);
                }

            } else if (args.find(PSTR("pitch")) == 0) {
                uint8_t pitch;
                CmdParser::split_args(args, pitch);
                p_tts_->set_pitch(pitch);
            } else if (args.find(PSTR("speak")) == 0) {
                if (p_tts_->is_playing()) {
                    return false;
                }

                const size_t s { args.find(' ') };
                if (s != args.npos) {
                    return p_tts_->speak(args.substr(s + 1), true);
                }
            } else if (CtBotConfig::AUDIO_TEST_AVAILABLE && args.find(PSTR("sine")) == 0) {
                const auto n { args.find(' ') };
                if (n == args.npos) {
                    return false;
                }
                const float freq { std::strtof(args.data() + n, nullptr) };
                p_audio_sine_->frequency(freq);
            } else {
                return false;
            }
            return true;
        });
    }

    p_parser_->register_cmd(PSTR("fs"), 'f', [this](const std::string_view& args) {
        if (args.find(PSTR("ls")) == 0) {
            const auto s { args.find(' ') };
            std::string dir;
            if (s == args.npos) {
                dir = "/";
            } else {
                dir = args.substr(s + 1);
            }

            auto root { SD.open(dir.c_str()) };
            if (!root.isDirectory()) {
                return false;
            }

            while (true) {
                auto entry { root.openNextFile() };
                if (!entry) {
                    break;
                }

                const auto len { p_comm_->debug_print(entry.name(), true) };
                if (!entry.isDirectory()) {
                    if (len < 14) {
                        p_comm_->debug_print('\t', true);
                    }
                    p_comm_->debug_printf<true>(PP_ARGS("\t{} KB\r\n", static_cast<uint32_t>(entry.size() / 1'024ULL)));
                } else {
                    p_comm_->debug_print(PSTR("\r\n"), true);
                }

                entry.close();
            }

            return true;
        }

        return false;
    });

    p_parser_->register_cmd(PSTR("sleep"), [this](const std::string_view& args) {
        uint32_t duration;
        CmdParser::split_args(args, duration);
        std::this_thread::sleep_for(std::chrono::milliseconds(duration));
        return true;
    });

    p_parser_->register_cmd(PSTR("crash"), [this](const std::string_view&) {
        get_serial_cmd()->write_direct('\r');
        get_serial_cmd()->write_direct('\n');
        volatile uint8_t* ptr { reinterpret_cast<uint8_t*>(1) };
        *ptr = 0;
        portINSTR_SYNC_BARRIER();
        return true;
    });

    if (CtBotConfig::PROG_AVAILABLE) {
        p_parser_->register_cmd(PSTR("prog"), 'p', [this](const std::string_view& args) {
            if (args.find(PSTR("run")) == 0) {
                const size_t s { args.find(' ') + 1 };
                const size_t e { args.find(' ', s) };
                const std::string_view filename { args.substr(s, e - s) };

                auto p_cmd_script { std::make_unique<CmdScript>(filename, *p_comm_, *p_parser_) };
                return p_cmd_script->exec_script();
            } else if (args.find(PSTR("view")) == 0) {
                const size_t s { args.find(' ') + 1 };
                const size_t e { args.find(' ', s) };
                const std::string_view filename { args.substr(s, e - s) };

                auto p_cmd_script { std::make_unique<CmdScript>(filename, *p_comm_, *p_parser_) };
                return p_cmd_script->print_script();
            } else if (args.find(PSTR("create")) == 0) {
                size_t num {};
                const std::string_view str { CmdParser::split_args(args, num) };
                const size_t s { str.find(' ') + 1 };
                const size_t e { str.find(' ', s) };
                const std::string_view filename { str.substr(s, e - s) };

                auto p_cmd_script { std::make_unique<CmdScript>(filename, *p_comm_, *p_parser_) };
                return p_cmd_script->create_script(num);
            }

            return false;
        });
    }

    if (CtBotConfig::I2C_TOOLS_AVAILABLE) {
        p_parser_->register_cmd(PSTR("i2c"), 'i', [this](const std::string_view& args) {
            static I2C_Service* p_i2c {};
            static uint8_t dev_addr {};
            if (args.find(PSTR("select")) == 0) {
                uint8_t bus;
                uint16_t freq;
                CmdParser::split_args(args, bus, freq);
                delete p_i2c;
                p_i2c = new I2C_Service { bus, static_cast<uint32_t>(freq * 1000UL), CtBotConfig::I2C0_PIN_SDA,
                    CtBotConfig::I2C0_PIN_SCL }; // FIXME: other bus pins
                return p_i2c != nullptr;
            } else if (args.find(PSTR("addr")) == 0) {
                CmdParser::split_args(args, dev_addr);
                return true;
            } else if (args.find(PSTR("read8")) == 0) {
                uint8_t addr;
                CmdParser::split_args(args, addr);
                uint8_t data {};
                if (p_i2c->read_reg(dev_addr, addr, data)) {
                    return false;
                } else {
                    p_comm_->debug_printf<true>(PP_ARGS("bus {} dev {#x} addr {#x} = {#x}\r\n", p_i2c->get_bus(), dev_addr, addr, data));
                    return true;
                }
            } else if (args.find(PSTR("read16")) == 0) {
                uint8_t addr;
                CmdParser::split_args(args, addr);
                uint16_t data {};
                if (p_i2c->read_reg(dev_addr, addr, data)) {
                    return false;
                } else {
                    p_comm_->debug_printf<true>(PP_ARGS("bus {} dev {#x} addr {#x} = {#x}\r\n", p_i2c->get_bus(), dev_addr, addr, data));
                    return true;
                }
            } else if (args.find(PSTR("read32")) == 0) {
                uint8_t addr;
                CmdParser::split_args(args, addr);
                uint32_t data {};
                if (p_i2c->read_reg(dev_addr, addr, data)) {
                    return false;
                } else {
                    p_comm_->debug_printf<true>(PP_ARGS("bus {} dev {#x} addr {#x} = {#x}\r\n", p_i2c->get_bus(), dev_addr, addr, data));
                    return true;
                }
            } else if (args.find(PSTR("write8")) == 0) {
                uint8_t addr;
                uint8_t data;
                CmdParser::split_args(args, addr, data);
                if (p_i2c->write_reg(dev_addr, addr, data)) {
                    return false;
                } else {
                    p_comm_->debug_printf<true>(PP_ARGS("bus {} dev {#x} addr {#x} = {#x}\r\n", p_i2c->get_bus(), dev_addr, addr, data));
                    return true;
                }
            } else if (args.find(PSTR("write16")) == 0) {
                uint8_t addr;
                uint16_t data;
                CmdParser::split_args(args, addr, data);
                if (p_i2c->write_reg(dev_addr, addr, data)) {
                    return false;
                } else {
                    p_comm_->debug_printf<true>(PP_ARGS("bus {} dev {#x} addr {#x} = {#x}\r\n", p_i2c->get_bus(), dev_addr, addr, data));
                    return true;
                }
            } else if (args.find(PSTR("write32")) == 0) {
                uint8_t addr;
                uint32_t data;
                CmdParser::split_args(args, addr, data);
                if (p_i2c->write_reg(dev_addr, addr, data)) {
                    return false;
                } else {
                    p_comm_->debug_printf<true>(PP_ARGS("bus {} dev {#x} addr {#x} = {#x}\r\n", p_i2c->get_bus(), dev_addr, addr, data));
                    return true;
                }
            } else if (args.find(PSTR("setbit")) == 0) {
                uint8_t addr;
                uint8_t bit;
                CmdParser::split_args(args, addr, bit);
                if (p_i2c->set_bit(dev_addr, addr, bit, true)) {
                    return false;
                } else {
                    uint8_t data;
                    p_i2c->read_reg(dev_addr, addr, data);
                    p_comm_->debug_printf<true>(PP_ARGS("bus {} dev {#x} addr {#x} = {#x}\r\n", p_i2c->get_bus(), dev_addr, addr, data));
                    return true;
                }
            } else if (args.find(PSTR("clearbit")) == 0) {
                uint8_t addr;
                uint8_t bit;
                CmdParser::split_args(args, addr, bit);
                if (p_i2c->set_bit(dev_addr, addr, bit, false)) {
                    return false;
                } else {
                    uint8_t data;
                    p_i2c->read_reg(dev_addr, addr, data);
                    p_comm_->debug_printf<true>(PP_ARGS("bus {} dev {#x} addr {#x} = {#x}\r\n", p_i2c->get_bus(), dev_addr, addr, data));
                    return true;
                }
            } else if (args.find(PSTR("scan")) == 0) {
                for (uint8_t i { 8 }; i < 120; ++i) {
                    // if (p_i2c->test(i)) { // FIXME: add scan
                    //     p_comm_->debug_printf<true>(PP_ARGS("bus {}: dev {#x} found.\r\n", p_i2c_->get_bus(), i));
                    // }
                }
            } else {
                return false;
            }

            return true;
        });
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
    //             p_comm_->debug_print(ret, true);
    //             p_comm_->debug_print("\r\n", true);
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
            p_comm_->debug_print(PSTR("CtBot::play_wav(): Still playing, abort.\r\n"), false);
            return false;
        }

        const auto str_begin { filename.find_first_not_of(' ') };
        if (str_begin == filename.npos) {
            p_comm_->debug_print(PSTR("CtBot::play_wav(): no file given, abort.\r\n"), false);
            return false;
        }

        const std::string file { filename.substr(str_begin) };
        const bool res { p_play_wav_->play(file.c_str()) };

        if (res) {
            p_comm_->debug_printf<false>(PP_ARGS("CtBot::play_wav(): Playing file \"{s}\"\r\n", file.c_str()));
        } else {
            p_comm_->debug_printf<false>(PP_ARGS("CtBot::play_wav(): File \"{s}\" not found\r\n", file.c_str()));
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

    p_comm_->debug_print(PSTR("System shutting down...\r\n"), false);
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

    delete p_parameter_;
    if (DEBUG_LEVEL_ > 1) {
        ::serialport_puts(PSTR("p_parameter_ deleted.\r\n"));
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

} // namespace ctbot
