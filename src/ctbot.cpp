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
#include "serial_connection_teensy.h"
#include "help_texts.h"
#include "cmd_parser.h"
#include "cmd_script.h"
#include "parameter_storage.h"
#include "i2c_wrapper.h"
#include "tests.h"

#include "timers.h"
#include "pprintpp.hpp"
#include "portable/teensy.h"
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
#include "SPI.h"
#include "arduino_freertos.h" // cleanup of ugly macro stuff etc.
#include "tts.h"


void software_isr();

extern "C" {
static int lua_wrapper_print(lua_State* L) {
    auto p_comm { ctbot::CtBot::get_instance().get_comm() };

    const int n = lua_gettop(L); /* number of arguments */
    lua_getglobal(L, "tostring");
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
}


namespace ctbot {
TaskHandle_t CtBot::audio_task_ {};

CtBot& CtBot::get_instance() {
    static CtBot* p_instance { CtBotConfig::BEHAVIOR_MODEL_AVAILABLE ? new CtBotBehavior : new CtBot };
    return *p_instance;
}

CtBot::CtBot()
    // initializes serial connection here for debug purpose
    : shutdown_ {}, ready_ {}, task_id_ {}, p_scheduler_ {}, p_sensors_ {}, p_motors_ { nullptr, nullptr },
      p_speedcontrols_ { nullptr, nullptr }, p_servos_ { nullptr, nullptr }, p_ena_ {}, p_ena_pwm_ {}, p_leds_ {}, p_lcd_ {}, p_tft_ {},
      p_serial_usb_ { new SerialConnectionTeensy { 0, CtBotConfig::UART0_BAUDRATE } }, p_serial_wifi_ {}, p_comm_ {}, p_parser_ {}, p_i2c_ {}, p_parameter_ {},
      p_audio_output_ {}, p_audio_sine_ {}, p_play_wav_ {}, p_tts_ {}, p_watch_timer_ {}, p_lua_ {} {
    std::atexit([]() {
        if (DEBUG) {
            ::serial_puts(PSTR("exit()"));
        }

        CtBot* ptr = &get_instance();
        delete ptr;

        if (DEBUG) {
            ::serial_puts(PSTR("exit(): stopping scheduler, bye."));
        }

        Scheduler::stop();
    });
}

CtBot::~CtBot() {
    if (DEBUG) {
        ::serial_puts(PSTR("CtBot::~CtBot(): destroying CtBot instance."));
    }
}

void CtBot::stop() {
    shutdown_ = true;
}

void CtBot::setup(const bool set_ready) {
    p_scheduler_ = new Scheduler;
    task_id_ = p_scheduler_->task_add("main", TASK_PERIOD_MS, TASK_PRIORITY, STACK_SIZE, [this]() { return run(); });
    p_scheduler_->task_register(PSTR("Tmr Svc"));
    p_scheduler_->task_register(PSTR("EVENT"));

    p_parser_ = new CmdParser;
    configASSERT(p_parser_);
    if (CtBotConfig::UART_FOR_CMD != 0) {
        p_serial_wifi_ = new SerialConnectionTeensy { CtBotConfig::UART_FOR_CMD, CtBotConfig::UART_WIFI_PIN_RX, CtBotConfig::UART_WIFI_PIN_TX,
            CtBotConfig::UART_WIFI_BAUDRATE };
        configASSERT(p_serial_wifi_);
        p_comm_ = new CommInterfaceCmdParser { *p_serial_wifi_, *p_parser_, true };
    } else {
        p_comm_ = new CommInterfaceCmdParser { *p_serial_usb_, *p_parser_, true };
    }
    configASSERT(p_comm_);

    p_ena_ = new EnaI2c { CtBotConfig::ENA_I2C_BUS, CtBotConfig::ENA_I2C_ADDR,
        CtBotConfig::ENA_I2C_BUS == 0 ?
            CtBotConfig::I2C0_FREQ :
            (CtBotConfig::ENA_I2C_BUS == 1 ? CtBotConfig::I2C1_FREQ : (CtBotConfig::ENA_I2C_BUS == 2 ? CtBotConfig::I2C2_FREQ : CtBotConfig::I2C3_FREQ)) };
    p_ena_pwm_ = new LedsI2cEna { CtBotConfig::ENA_PWM_I2C_BUS, CtBotConfig::ENA_PWM_I2C_ADDR,
        CtBotConfig::ENA_PWM_I2C_BUS == 0 ?
            CtBotConfig::I2C0_FREQ :
            (CtBotConfig::ENA_PWM_I2C_BUS == 1 ? CtBotConfig::I2C1_FREQ :
                                                 (CtBotConfig::ENA_PWM_I2C_BUS == 2 ? CtBotConfig::I2C2_FREQ : CtBotConfig::I2C3_FREQ)) };

    p_sensors_ = new Sensors { *this };
    configASSERT(p_sensors_);
    p_sensors_->enable_sensors();

    p_motors_[0] = new Motor { p_sensors_->get_enc_l(), CtBotConfig::MOT_L_PWM_PIN, CtBotConfig::MOT_L_DIR_PIN, CtBotConfig::MOT_L_DIR };
    p_motors_[1] = new Motor { p_sensors_->get_enc_r(), CtBotConfig::MOT_R_PWM_PIN, CtBotConfig::MOT_R_DIR_PIN, CtBotConfig::MOT_R_DIR };

    p_speedcontrols_[0] = new SpeedControl { p_sensors_->get_enc_l(), *p_motors_[0] };
    p_speedcontrols_[1] = new SpeedControl { p_sensors_->get_enc_r(), *p_motors_[1] };

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
    }

    init_parser();

    if (CtBotConfig::I2C_TOOLS_AVAILABLE) {
        p_i2c_ = new I2C_Wrapper { 0 };
        configASSERT(p_i2c_);
    }

#ifdef BUILTIN_SDCARD
    if (!(SD.begin(BUILTIN_SDCARD))) {
        p_comm_->debug_print(PSTR("SD.begin() failed.\r\n"), false);
    }
#endif

    p_parameter_ = new ParameterStorage { "ctbot.jsn" };
    configASSERT(p_parameter_);

    if (CtBotConfig::AUDIO_AVAILABLE) {
        Scheduler::enter_critical_section();
        p_play_wav_ = new AudioPlaySdWav;
        configASSERT(p_play_wav_);
        p_tts_ = new TTS;
        configASSERT(p_tts_);
        p_audio_output_ = new AudioOutputI2S;
        configASSERT(p_audio_output_);
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

        for (uint8_t i {}; i < CtBotConfig::AUDIO_CHANNELS; ++i) {
            p_audio_conn_.push_back(new AudioConnection { *p_play_wav_, i, *p_audio_mixer_[i], 0 });
            configASSERT(*p_audio_conn_.rbegin());
            p_audio_conn_.push_back(new AudioConnection { *p_audio_mixer_[i], 0, *p_audio_output_, i });
            configASSERT(*p_audio_conn_.rbegin());
        }
        p_audio_conn_.push_back(new AudioConnection { *p_tts_, 0, *p_audio_mixer_[0], 1 });
        if (CtBotConfig::AUDIO_TEST_AVAILABLE) {
            p_audio_conn_.push_back(new AudioConnection { *p_audio_sine_, 0, *p_audio_mixer_[0], 2 });
            configASSERT(*p_audio_conn_.rbegin());
        }

        AudioMemory(CtBotConfig::AUDIO_CHANNELS * 2 + 2);

        ::attachInterruptVector(IRQ_SOFTWARE, []() {
            if (ctbot::CtBotConfig::AUDIO_AVAILABLE && audio_task_) {
                ::xTaskNotifyFromISR(audio_task_, 0, eNoAction, nullptr);
                portYIELD_FROM_ISR(true);
            }
        });
        Scheduler::exit_critical_section();

        get_scheduler()->task_add("audio", 1, Scheduler::MAX_PRIORITY, 512, [this]() {
            while (get_ready()) {
                ::xTaskNotifyWait(0, 0, nullptr, portMAX_DELAY);
                ::software_isr(); // AudioStream::update_all()
            }
            if (shutdown_) {
                audio_task_ = nullptr;
            }
        });
        audio_task_ = ::xTaskGetHandle("audio");

        p_scheduler_->task_register("speak", true);
    }

    if (CtBotConfig::LUA_AVAILABLE) {
        p_lua_ = new LuaWrapper;
        configASSERT(p_lua_);
        lua_register(p_lua_->get_state(), "print", lua_wrapper_print);
    }

    add_post_hook(
        "task",
        [this]() {
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
                    p_comm_->debug_print("\r\n", true);
                }
            }
        },
        false);

    p_comm_->debug_print(PSTR("\r\n*** c't-Bot init done. ***\n\r\nType \"help\" (or \"h\") to print help message\n\r\n"), true);
    p_comm_->flush();

    ready_ = set_ready;

    // auto p_er = new EventResponder;
    // auto p_mt = new MillisTimer;
    // p_mt->beginRepeating(3'000, *p_er);
    // p_er->attach([](EventResponder&) {
    //     CtBot::get_instance().get_comm()->debug_print(PSTR("p_er triggered by yield.\n\rtask: \""), true);
    //     TaskStatus_t status;
    //     ::vTaskGetInfo(nullptr, &status, false, eInvalid);
    //     CtBot::get_instance().get_comm()->debug_print(status.pcTaskName, true);
    //     CtBot::get_instance().get_comm()->debug_print(PSTR("\"\n\r"), true);
    // });
    // p_er->attachImmediate([](EventResponder&) {
    //     CtBot::get_instance().get_comm()->debug_print(PSTR("p_er triggered imm.\n\rtask: \""), true);
    //     TaskStatus_t status;
    //     ::vTaskGetInfo(nullptr, &status, false, eInvalid);
    //     CtBot::get_instance().get_comm()->debug_print(status.pcTaskName, true);
    //     CtBot::get_instance().get_comm()->debug_print(PSTR("\"\n\r"), true);
    // });
    // p_er->attachInterrupt([](EventResponder&) {
    //     CtBot::get_instance().get_comm()->debug_print(PSTR("p_er triggered int.\n\rtask: \""), true);
    //     TaskStatus_t status;
    //     ::vTaskGetInfo(nullptr, &status, false, eInvalid);
    //     CtBot::get_instance().get_comm()->debug_print(status.pcTaskName, true);
    //     CtBot::get_instance().get_comm()->debug_print(PSTR("\"\n\r"), true);
    // });
}

void CtBot::init_parser() {
    CtBotHelpTexts::init();
    p_parser_->register_cmd(PSTR("help"), 'h', [this](const std::string_view&) {
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
            p_watch_timer_ = ::xTimerCreate(PSTR("watch_t"), pdMS_TO_TICKS(1'000UL), true, p_cmd, [](TimerHandle_t handle) {
                CtBot& ctbot { CtBot::get_instance() };
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
            uint8_t v {};
            CmdParser::split_args(args, v);
            p_comm_->set_echo(v);
        } else if (args.find(PSTR("task")) == 0) {
            const size_t s { args.find(' ') + 1 };
            const size_t e { args.find(' ', s) };
            const uint16_t task_id { get_scheduler()->task_get(args.substr(s, e - s)) };

            if (task_id < 0xffff) {
                uint8_t v {};
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
            uint8_t v {};
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
            uint8_t v {};
            CmdParser::split_args(args.substr(s), v);
            auto it { post_hooks_.find(hookname) };
            if (it != post_hooks_.end()) {
                std::get<1>(it->second) = v;
            } else {
                for (const auto& e : post_hooks_) {
                    p_comm_->debug_print(e.first, true);
                    p_comm_->debug_print("\r\n", true);
                }
                return false;
            }
        } else if (args.find(PSTR("kp")) == 0) {
            int16_t left {}, right {};
            CmdParser::split_args(args, left, right);
            p_speedcontrols_[0]->set_parameters(static_cast<float>(left), p_speedcontrols_[0]->get_ki(), p_speedcontrols_[0]->get_kd());
            p_speedcontrols_[1]->set_parameters(static_cast<float>(right), p_speedcontrols_[1]->get_ki(), p_speedcontrols_[1]->get_kd());
        } else if (args.find(PSTR("ki")) == 0) {
            int16_t left {}, right {};
            CmdParser::split_args(args, left, right);
            p_speedcontrols_[0]->set_parameters(p_speedcontrols_[0]->get_kp(), static_cast<float>(left), p_speedcontrols_[0]->get_kd());
            p_speedcontrols_[1]->set_parameters(p_speedcontrols_[1]->get_kp(), static_cast<float>(right), p_speedcontrols_[1]->get_kd());
        } else if (args.find(PSTR("kd")) == 0) {
            int16_t left {}, right {};
            CmdParser::split_args(args, left, right);
            p_speedcontrols_[0]->set_parameters(p_speedcontrols_[0]->get_kp(), p_speedcontrols_[0]->get_ki(), static_cast<float>(left));
            p_speedcontrols_[1]->set_parameters(p_speedcontrols_[1]->get_kp(), p_speedcontrols_[1]->get_ki(), static_cast<float>(right));
        } else if (CtBotConfig::LCD_AVAILABLE && args.find("lcdout") != args.npos) {
            const size_t s { args.find(' ') + 1 };
            const size_t e { args.find(' ', s) };
            p_lcd_->set_output(args.substr(s, e - s));
        } else if (args.find(PSTR("led")) == 0) {
            uint8_t mask {}, pwm {};
            CmdParser::split_args(args, mask, pwm);
            p_leds_->set_pwm(static_cast<LedTypes>(mask), pwm);
        } else if (args.find(PSTR("enapwm")) == 0) {
            uint8_t mask {}, pwm {};
            CmdParser::split_args(args, mask, pwm);
            p_ena_pwm_->set_pwm(static_cast<LedTypesEna>(mask), pwm);
        } else {
            return false;
        }
        return true;
    });

    p_parser_->register_cmd(PSTR("get"), 'g', [this](const std::string_view& args) {
        if (args.find("dist") == 0) {
            p_comm_->debug_printf<true>(PP_ARGS("{} {}", p_sensors_->get_distance_l(), p_sensors_->get_distance_r()));
        } else if (args.find("enc") == 0) {
            p_comm_->debug_printf<true>(PP_ARGS("{} {}", p_sensors_->get_enc_l().get(), p_sensors_->get_enc_r().get()));
            // } else if (args.find("mouse") == 0) {
            //     // mouse sensor not implemented
        } else if (args.find("border") == 0) {
            p_comm_->debug_printf<true>(PP_ARGS("{} {}", p_sensors_->get_border_l(), p_sensors_->get_border_r()));
        } else if (args.find("line") == 0) {
            p_comm_->debug_printf<true>(PP_ARGS("{} {}", p_sensors_->get_line_l(), p_sensors_->get_line_r()));
        } else if (args.find("ldr") == 0) {
            p_comm_->debug_printf<true>(PP_ARGS("{} {}", p_sensors_->get_ldr_l(), p_sensors_->get_ldr_r()));
        } else if (args.find("speed") == 0) {
            const auto l { static_cast<int16_t>(p_sensors_->get_enc_l().get_speed()) };
            const auto r { static_cast<int16_t>(p_sensors_->get_enc_r().get_speed()) };
            p_comm_->debug_printf<true>(PP_ARGS("{} {}", l, r));
        } else if (args.find("gyro") == 0) {
            p_comm_->debug_printf<true>(PP_ARGS("{6.2}\r\n", p_sensors_->get_mpu6050()->get_angle_gyro_z()));
        } else if (args.find("motor") == 0) {
            p_comm_->debug_printf<true>(PP_ARGS("{} {}", p_motors_[0]->get(), p_motors_[1]->get()));
        } else if (args.find("servo") == 0) {
            p_comm_->debug_printf<true>(PP_ARGS("{}[{s}] ", p_servos_[0]->get_position(), p_servos_[0]->get_active() ? "on " : "off"));
            if (p_servos_[1]) {
                p_comm_->debug_printf<true>(PP_ARGS("{}[{s}]", p_servos_[1]->get_position(), p_servos_[1]->get_active() ? "on" : "off"));
            }
        } else if (args.find("rc5") == 0) {
            p_comm_->debug_printf<true>(
                PP_ARGS("{} {} {}", p_sensors_->get_rc5().get_addr(), p_sensors_->get_rc5().get_cmd(), p_sensors_->get_rc5().get_toggle()));
        } else if (args.find("transmm") == 0) {
            p_comm_->debug_print(p_sensors_->get_transport_mm(), true);
        } else if (args.find("trans") == 0) {
            p_comm_->debug_print(p_sensors_->get_transport(), true);
            // } else if (args.find("door") == 0) {
            //     p_comm_->debug_print(p_sensors_->get_shutter(), true);
        } else if (args.find("led") == 0) {
            p_comm_->debug_printf<true>(PP_ARGS("{#x}", static_cast<uint8_t>(p_leds_->get())));
        } else if (args.find("volt") == 0) {
            p_comm_->debug_printf<true>(PP_ARGS("{.2} V", p_sensors_->get_bat_voltage()));
        } else if (args.find("tasks") == 0) {
            get_scheduler()->print_task_list(*p_comm_);
        } else if (args.find("free") == 0) {
            get_scheduler()->print_ram_usage(*p_comm_);
            const auto mem_use { AudioMemoryUsage() };
            const auto mem_use_max { AudioMemoryUsageMax() };
            p_comm_->debug_printf<true>(PP_ARGS("\r\nAudioMemoryUsage()={} blocks\tmax={} blocks\r\n", mem_use, mem_use_max));
            const float cpu_use { AudioProcessorUsage() };
            const float cpu_use_max { AudioProcessorUsageMax() };
            p_comm_->debug_printf<true>(PP_ARGS("AudioProcessorUsage()={.2} %%\tmax={.2} %%", cpu_use, cpu_use_max));
        } else if (args.find("params") == 0) {
            auto dump { p_parameter_->dump() };
            p_comm_->debug_printf<true>(PP_ARGS("dump=\"{s}\"", dump->c_str()));
        } else if (args.find("paramf") == 0) {
            size_t s { args.find(' ') };
            if (s == args.npos) {
                return false;
            }
            ++s;
            const size_t e { args.find(' ', s) };
            const std::string_view param { args.substr(s, e - s) };
            float x;
            if (p_parameter_->get(param, x)) {
                p_comm_->debug_printf<true>("paramf \"%.*s\"=%f", param.size(), param.data(), x);
            } else {
                return false;
            }
        } else if (args.find("param") == 0) {
            size_t s { args.find(' ') };
            if (s == args.npos) {
                return false;
            }
            ++s;
            const size_t e { args.find(' ', s) };
            const std::string_view param { args.substr(s, e - s) };
            int32_t x;
            if (p_parameter_->get(param, x)) {
                p_comm_->debug_printf<true>("param \"%.*s\"=%" PRId32, param.size(), param.data(), x);
            } else {
                return false;
            }
        } else {
            return false;
        }
        p_comm_->debug_print("\r\n", true);
        return true;
    });

    p_parser_->register_cmd(PSTR("set"), 's', [this](const std::string_view& args) {
        if (args.find("speed") == 0) {
            int16_t left {}, right {};
            CmdParser::split_args(args, left, right);
            p_speedcontrols_[0]->set_speed(static_cast<float>(left));
            p_speedcontrols_[1]->set_speed(static_cast<float>(right));
        } else if (args.find("motor") == 0) {
            int16_t left {}, right {};
            CmdParser::split_args(args, left, right);
            p_motors_[0]->set(left);
            p_motors_[1]->set(right);
        } else if (args.find("servo") == 0) {
            uint8_t s1 {}, s2 {};
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
        } else if (args.find("enapwm") == 0) {
            uint8_t pin {};
            bool value {};
            CmdParser::split_args(args, pin, value);

            if (value) {
                p_ena_pwm_->on(static_cast<LedTypesEna>(1 << pin));
            } else {
                p_ena_pwm_->off(static_cast<LedTypesEna>(1 << pin));
            }
        } else if (args.find("ena") == 0) {
            uint8_t pin {};
            bool value {};
            CmdParser::split_args(args, pin, value);

            if (value) {
                p_ena_->on(static_cast<EnaI2cTypes>(1 << pin));
            } else {
                p_ena_->off(static_cast<EnaI2cTypes>(1 << pin));
            }
        } else if (args.find("led") == 0) {
            uint8_t led {};
            CmdParser::split_args(args, led);
            p_leds_->set(static_cast<LedTypes>(led));
        } else if (CtBotConfig::LCD_AVAILABLE && args.find("lcdbl") == 0) {
            bool v {};
            CmdParser::split_args(args, v);
            p_lcd_->set_backlight(v);
        } else if (CtBotConfig::LCD_AVAILABLE && args.find("lcd") == 0) {
            uint8_t line {}, column {};
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
        } else if (CtBotConfig::TFT_AVAILABLE && args.find("tftbl") == 0) {
            const auto n { args.find(' ') };
            if (n == args.npos) {
                return false;
            }
            const float v { std::strtof(args.data() + n, nullptr) };
            p_tft_->set_backlight(v);
        } else if (args.find("paramf") == 0) {
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
            p_comm_->debug_printf<true>("key=\"%.*s\"\r\n", key.size(), key.data());

            const std::string_view val { args.substr(e + 1) };
            p_comm_->debug_printf<true>("val=\"%.*s\"\r\n", val.size(), val.data());

            const float value { std::strtof(val.data(), nullptr) };
            p_comm_->debug_printf<true>(PP_ARGS("value={}\r\n", value));

            p_parameter_->set<float>(key, value);
            p_parameter_->flush();
        } else if (args.find("param") == 0) {
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
            p_comm_->debug_printf<true>("key=\"%.*s\"\r\n", key.size(), key.data());

            const std::string_view val { args.substr(e + 1) };
            p_comm_->debug_printf<true>("val=\"%.*s\"\r\n", val.size(), val.data());

            int32_t value;
            std::from_chars(val.cbegin(), std::prev(val.cend()), value);
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
            if (args.find("play") == 0) {
                const size_t s { args.find(' ') + 1 };
                const size_t e { args.find(' ', s) };
                return play_wav(args.substr(s, e - s));
            } else if (args.find("stop") == 0) {
                p_play_wav_->stop();
                get_ena()->off(EnaI2cTypes::AUDIO);
            } else if (args.find("vol") == 0) {
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

            } else if (args.find("pitch") == 0) {
                uint8_t pitch {};
                CmdParser::split_args(args, pitch);
                p_tts_->set_pitch(pitch);
            } else if (args.find("speak") == 0) {
                if (p_tts_->is_playing()) {
                    return false;
                }

                const size_t s { args.find(' ') };
                if (s != args.npos) {
                    get_ena()->on(EnaI2cTypes::AUDIO);
                    return p_tts_->speak(args.substr(s + 1), true);
                }
            } else if (CtBotConfig::AUDIO_TEST_AVAILABLE && args.find("sine") == 0) {
                const auto n { args.find(' ') };
                if (n == args.npos) {
                    return false;
                }
                const float freq { std::strtof(args.data() + n, nullptr) };
                get_ena()->on(EnaI2cTypes::AUDIO);
                p_audio_sine_->frequency(freq);
                if (freq <= 0.f) {
                    get_ena()->off(EnaI2cTypes::AUDIO);
                }
            } else {
                return false;
            }
            return true;
        });
    }

    p_parser_->register_cmd(PSTR("fs"), 'f', [this](const std::string_view& args) {
        if (args.find("ls") == 0) {
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

                p_comm_->debug_print(entry.name(), true);
                if (!entry.isDirectory()) {
                    p_comm_->debug_printf<true>(PP_ARGS("\t\t{}\r\n", entry.size()));
                }

                entry.close();
            }

            return true;
        }

        return false;
    });

    p_parser_->register_cmd(PSTR("sleep"), [this](const std::string_view& args) {
        uint32_t duration {};
        CmdParser::split_args(args, duration);
        std::this_thread::sleep_for(std::chrono::milliseconds(duration));
        return true;
    });

    p_parser_->register_cmd(PSTR("crash"), [](const std::string_view&) {
        uint8_t* ptr = nullptr;
        *ptr = 42;
        return true;
    });

    if (CtBotConfig::PROG_AVAILABLE) {
        p_parser_->register_cmd(PSTR("prog"), 'p', [this](const std::string_view& args) {
            if (args.find("run") == 0) {
                const size_t s { args.find(' ') + 1 };
                const size_t e { args.find(' ', s) };
                const std::string_view filename { args.substr(s, e - s) };

                auto p_cmd_script { std::make_unique<CmdScript>(filename, *p_comm_, *p_parser_) };
                return p_cmd_script->exec_script();
            } else if (args.find("view") == 0) {
                const size_t s { args.find(' ') + 1 };
                const size_t e { args.find(' ', s) };
                const std::string_view filename { args.substr(s, e - s) };

                auto p_cmd_script { std::make_unique<CmdScript>(filename, *p_comm_, *p_parser_) };
                return p_cmd_script->print_script();
            } else if (args.find("create") == 0) {
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
            if (args.find("select") == 0) {
                uint8_t bus {};
                uint16_t freq {};
                CmdParser::split_args(args, bus, freq);
                return p_i2c_->init(bus, freq * 1000UL);
            } else if (args.find("addr") == 0) {
                uint8_t addr {};
                CmdParser::split_args(args, addr);
                p_i2c_->set_address(addr);
                return true;
            } else if (args.find("read8") == 0) {
                uint8_t addr {};
                CmdParser::split_args(args, addr);
                uint8_t data {};
                if (p_i2c_->read_reg8(addr, data)) {
                    return false;
                } else {
                    p_comm_->debug_printf<true>(PP_ARGS(
                        "bus {} @ {} kHz dev {#x} addr {#x} = {#x}\r\n", p_i2c_->get_bus(), p_i2c_->get_freq() / 1000UL, p_i2c_->get_address(), addr, data));
                    return true;
                }
            } else if (args.find("read16") == 0) {
                uint8_t addr {};
                CmdParser::split_args(args, addr);
                uint16_t data {};
                if (p_i2c_->read_reg16(addr, data)) {
                    return false;
                } else {
                    p_comm_->debug_printf<true>(PP_ARGS(
                        "bus {} @ {} kHz dev {#x} addr {#x} = {#x}\r\n", p_i2c_->get_bus(), p_i2c_->get_freq() / 1000UL, p_i2c_->get_address(), addr, data));
                    return true;
                }
            } else if (args.find("read32") == 0) {
                uint8_t addr {};
                CmdParser::split_args(args, addr);
                uint32_t data {};
                if (p_i2c_->read_reg32(addr, data)) {
                    return false;
                } else {
                    p_comm_->debug_printf<true>(PP_ARGS(
                        "bus {} @ {} kHz dev {#x} addr {#x} = {#x}\r\n", p_i2c_->get_bus(), p_i2c_->get_freq() / 1000UL, p_i2c_->get_address(), addr, data));
                    return true;
                }
            } else if (args.find("write8") == 0) {
                uint8_t addr {};
                uint8_t data {};
                CmdParser::split_args(args, addr, data);
                if (p_i2c_->write_reg8(addr, data)) {
                    return false;
                } else {
                    p_comm_->debug_printf<true>(PP_ARGS(
                        "bus {} @ {} kHz dev {#x} addr {#x} = {#x}\r\n", p_i2c_->get_bus(), p_i2c_->get_freq() / 1000UL, p_i2c_->get_address(), addr, data));
                    return true;
                }
            } else if (args.find("write16") == 0) {
                uint8_t addr {};
                uint16_t data {};
                CmdParser::split_args(args, addr, data);
                if (p_i2c_->write_reg16(addr, data)) {
                    return false;
                } else {
                    p_comm_->debug_printf<true>(PP_ARGS(
                        "bus {} @ {} kHz dev {#x} addr {#x} = {#x}\r\n", p_i2c_->get_bus(), p_i2c_->get_freq() / 1000UL, p_i2c_->get_address(), addr, data));
                    return true;
                }
            } else if (args.find("write32") == 0) {
                uint8_t addr {};
                uint32_t data {};
                CmdParser::split_args(args, addr, data);
                if (p_i2c_->read_reg32(addr, data)) {
                    return false;
                } else {
                    p_comm_->debug_printf<true>(PP_ARGS(
                        "bus {} @ {} kHz dev {#x} addr {#x} = {#x}\r\n", p_i2c_->get_bus(), p_i2c_->get_freq() / 1000UL, p_i2c_->get_address(), addr, data));
                    return true;
                }
            } else if (args.find("setbit") == 0) {
                uint8_t addr {};
                uint8_t bit {};
                CmdParser::split_args(args, addr, bit);
                if (p_i2c_->set_bit(addr, bit, true)) {
                    return false;
                } else {
                    uint8_t data;
                    p_i2c_->read_reg8(addr, data);
                    p_comm_->debug_printf<true>(PP_ARGS(
                        "bus {} @ {} kHz dev {#x} addr {#x} = {#x}\r\n", p_i2c_->get_bus(), p_i2c_->get_freq() / 1000UL, p_i2c_->get_address(), addr, data));
                    return true;
                }
            } else if (args.find("clearbit") == 0) {
                uint8_t addr {};
                uint8_t bit {};
                CmdParser::split_args(args, addr, bit);
                if (p_i2c_->set_bit(addr, bit, false)) {
                    return false;
                } else {
                    uint8_t data;
                    p_i2c_->read_reg8(addr, data);
                    p_comm_->debug_printf<true>(PP_ARGS(
                        "bus {} @ {} kHz dev {#x} addr {#x} = {#x}\r\n", p_i2c_->get_bus(), p_i2c_->get_freq() / 1000UL, p_i2c_->get_address(), addr, data));
                    return true;
                }
            } else if (args.find("scan") == 0) {
                for (uint8_t i { 8 }; i < 120; ++i) {
                    if (p_i2c_->test(i)) {
                        p_comm_->debug_printf<true>(
                            PP_ARGS("bus {} @ {} kHz: dev {#x} found.\r\n", p_i2c_->get_bus(), p_i2c_->get_freq() / 1000UL, p_i2c_->get_address()));
                    } else {
                        // p_comm_->debug_printf<true>(
                        //     PP_ARGS("bus {} @ {} kHz: dev {#x} NOT found.\r\n", p_i2c_->get_bus(), p_i2c_->get_freq() / 1000UL,
                        //     p_i2c_->get_address()));
                    }
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

bool CtBot::play_wav(const std::string_view& filename) {
    if (CtBotConfig::AUDIO_AVAILABLE) {
        if (p_play_wav_->isPlaying()) {
            p_comm_->debug_print("CtBot::play_wav(): Still playing, abort.\r\n", false);
            return false;
        }

        const auto str_begin { filename.find_first_not_of(' ') };
        if (str_begin == filename.npos) {
            p_comm_->debug_print("CtBot::play_wav(): no file given, abort.\r\n", false);
            return false;
        }

        get_ena()->on(EnaI2cTypes::AUDIO);
        using namespace std::chrono_literals;
        std::this_thread::sleep_for(200ms);
        const std::string file { filename.substr(str_begin) };
        const bool res { p_play_wav_->play(file.c_str()) };

        if (res) {
            p_comm_->debug_printf<false>(PP_ARGS("CtBot::play_wav(): Playing file \"{s}\"\r\n", file.c_str()));
        } else {
            get_ena()->off(EnaI2cTypes::AUDIO);
            p_comm_->debug_printf<false>(PP_ARGS("CtBot::play_wav(): File \"{s}\" not found\r\n", file.c_str()));
        }

        return res;
    } else {
        return false;
    }
}

void CtBot::shutdown() {
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
    p_motors_[0]->set(0);
    p_motors_[1]->set(0);
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

    if (CtBotConfig::AUDIO_AVAILABLE) {
        if (CtBotConfig::AUDIO_TEST_AVAILABLE) {
            delete p_audio_sine_;
            if (DEBUG) {
                ::serial_puts(PSTR("CtBot::shutdown(): p_audio_sine_ deleted."));
            }
        }
        p_scheduler_->task_remove(p_scheduler_->task_get("audio"));
        if (DEBUG) {
            ::serial_puts(PSTR("CtBot::shutdown(): audio task removed."));
        }
        for (auto& e : p_audio_conn_) {
            delete e;
        }
        if (DEBUG) {
            ::serial_puts(PSTR("CtBot::shutdown(): p_audio_conn_ deleted."));
        }

        for (auto& e : p_audio_mixer_) {
            delete e;
        }
        if (DEBUG) {
            ::serial_puts(PSTR("CtBot::shutdown(): p_audio_mixer_ deleted."));
        }

        delete p_audio_output_;
        if (DEBUG) {
            ::serial_puts(PSTR("CtBot::shutdown(): p_audio_output_ deleted."));
        }

        delete p_tts_;
        if (DEBUG) {
            ::serial_puts(PSTR("CtBot::shutdown(): p_tts_ deleted."));
        }

        delete p_play_wav_;
        if (DEBUG) {
            ::serial_puts(PSTR("CtBot::shutdown(): p_play_wav_ deleted."));
        }
    }

    delete p_parameter_;
    if (DEBUG) {
        ::serial_puts(PSTR("p_parameter_ deleted."));
    }
    if (CtBotConfig::TFT_AVAILABLE) {
        delete p_tft_;
        if (DEBUG) {
            ::serial_puts(PSTR("p_tft_ deleted."));
        }
    }
    if (CtBotConfig::LCD_AVAILABLE) {
        delete p_lcd_;
        if (DEBUG) {
            ::serial_puts(PSTR("p_lcd_ deleted."));
        }
    };
    delete p_leds_;
    if (DEBUG) {
        ::serial_puts(PSTR("p_leds_ deleted."));
    }
    delete p_servos_[0];
    delete p_servos_[1];
    if (DEBUG) {
        ::serial_puts(PSTR("p_servos_ deleted."));
    }
    delete p_speedcontrols_[0];
    delete p_speedcontrols_[1];
    if (DEBUG) {
        ::serial_puts(PSTR("p_speedcontrols_ deleted."));
    }
    delete p_motors_[0];
    delete p_motors_[1];
    if (DEBUG) {
        ::serial_puts(PSTR("p_motors_ deleted."));
    }
    delete p_sensors_;
    if (DEBUG) {
        ::serial_puts(PSTR("p_sensors_ deleted."));
    }
    delete p_comm_;
    if (DEBUG) {
        ::serial_puts(PSTR("p_comm_ deleted."));
    }
    delete p_serial_wifi_;
    if (DEBUG) {
        ::serial_puts(PSTR("p_serial_wifi_ deleted."));
    }
    delete p_parser_;
    if (DEBUG) {
        ::serial_puts(PSTR("p_parser_ deleted."));
    }
    delete p_scheduler_;
    if (DEBUG) {
        ::serial_puts(PSTR("p_scheduler_ deleted."));
    }
    delete p_serial_usb_;
    p_serial_usb_ = nullptr;
    if (DEBUG) {
        ::serial_puts(PSTR("p_serial_usb_ deleted."));
    }

    free_rtos_std::gthr_freertos::set_next_stacksize(384);
    auto p_exit_thread { std::make_unique<std::thread>([]() { std::exit(0); }) };
    free_rtos_std::gthr_freertos::set_name(p_exit_thread.get(), "EXIT");
    free_rtos_std::gthr_freertos::set_priority(p_exit_thread.get(), 9);

    while (true) {
    }
}

void CtBot::add_pre_hook(const std::string& name, std::function<void()>&& hook, bool active) {
    pre_hooks_[name] = std::make_tuple(hook, active);
}

void CtBot::add_post_hook(const std::string& name, std::function<void()>&& hook, bool active) {
    post_hooks_[name] = std::make_tuple(hook, active);
}

} // namespace ctbot
