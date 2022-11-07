/*
 * This file is part of the ct-Bot teensy framework.
 * Copyright (c) 2021 Timo Sandmann
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
 * @file    ctbot_cli.cpp
 * @brief   Command line interface
 * @author  Timo Sandmann
 * @date    20.11.2021
 */

#include "ctbot_cli.h"

#include "ctbot.h"
#include "ctbot_config.h"
#include "cmd_parser.h"
#include "cmd_script.h"
#include "comm_interface.h"
#include "i2c_service.h"
#include "logger.h"
#include "parameter_storage.h"
#include "scheduler.h"
#include "sensors.h"
#include "speed_control.h"

#include "driver/lc_display.h"
#include "driver/leds_i2c.h"
#include "driver/motor.h"
#include "driver/mpu_6050.h"
#include "driver/servo.h"
#include "driver/serial_io.h"
#include "driver/tft_display.h"

#ifndef sei
#define sei() __enable_irq() // for Audio.h
#endif
#ifndef cli
#define cli() __disable_irq() // for Audio.h
#endif

#include "pprintpp.hpp"
#include "Audio.h"
#include "arduino_freertos.h" // cleanup of ugly macro stuff etc.
#include "tts.h"
#include "freertos_time.h"

#include <chrono>
#include <cinttypes>
#include <cstring>
#include <thread>


namespace ctbot {
const char CtBotCli::general_[] { "command subcommand [param]           explanation\r\n"
                                  "----------------------------------------------------------------------------------\r\n"
                                  "help (h)                             print this help message\r\n"
                                  "halt                                 shutdown and put Teensy in sleep mode\r\n"
                                  "watch (w) COMMAND                    execute COMMAND every second\r\n"
                                  "sleep MS                             sleep for MS milliseconds\r\n"
                                  "stack TASK_ID                        print stack of task with id TASK_ID\r\n"
                                  "crash                                cause a crash intentionally\r\n" };

const char CtBotCli::config_[] { "config (c)\r\n"
                                 "\techo [0|1]                   set console echo on/off\r\n"
                                 "\tviewer [0|1]                 start/stop remote viewer connection\r\n"
                                 "\ttask TASKNAME [0|1]          start/stop a TASK\r\n"
                                 "\tprehook HOOK [0|1]           start/stop a pre-hook\r\n"
                                 "\tposthook HOOK [0|1]          start/stop a post-hook\r\n"
                                 "\tk{p,i,d} [0;65535]           set Kp/Ki/Kd parameter for speed controller\r\n"
                                 "\tled BITMASK [0;255]          set LED brightness for given leds\r\n"
                                 "\tenapwm BITMASK [0;255]       set duty cycle for given ena channels\r\n" };

const char CtBotCli::get_[] { "get (g)\r\n"
                              "\tdist                         print distance sensor's values\r\n"
                              "\tenc                          print encoder's values\r\n"
                              "\tborder                       print border sensor's values\r\n"
                              "\tline                         print line sensor's values\r\n"

                              "\tspeed                        print speed for left and right wheel\r\n"
                              "\tservo                        print setpoints for servos\r\n"
                              "\trc5                          print last received RC5 data\r\n"

                              "\ttransmm                      print transport pocket distance measured\r\n"
                              "\ttrans                        print transport pocket status\r\n"
                              "\tled                          print LED setting\r\n"

                              "\tvolt                         print input voltage\r\n"
                              "\tcurrents                     print currents for 5V, motors and servo\r\n"

                              "\ttasks                        print task list\r\n"
                              "\tfree                         print free RAM, etc.\r\n"
                              "\tparam PARAMETER              print value of given integer parameter\r\n"
                              "\tparamf PARAMETER             print value of given float parameter\r\n"
                              "\tparams                       print all saved parameters\r\n"
                              "\ttime                         print current date and time in UTC\r\n"
                              "\tsctrl-crc                    print count of CRC errors for speed controller\r\n" };

const char CtBotCli::set_[] { "set (s)\r\n"
                              "\tspeed [-100;100] [=]         set new speed in % for left and right motor\r\n"
                              "\tservo [0;180|255] [=]        set new position for servo 1 and 2, 255 to disable\r\n"
                              "\trc5 [0;31] [0;127]           set RC5 code with address and command\r\n"
                              "\tena [0;7] [0|1]              enable/disable single ENA channel\r\n"
                              "\tenapwm [0;7] [0|1]           enable/disable single ENA pwm channel\r\n"
                              "\tled [0;255]                  enable/disable LEDs of given mask\r\n"
                              "\tlcd [1;4] [1;20] TEXT        print TEXT on LCD at line and column\r\n"
                              "\tlcdbl [0;1]                  switch LCD backlight ON (1) or OFF (0)\r\n"
                              "\ttftbl [0;100]                set TFT backlight brightness to given percentage\r\n"
                              "\tparam PARAMETER VALUE        set value of given integer parameter\r\n"
                              "\tparamf PARAMETER VALUE       set value of given float parameter\r\n"
                              "\ttime UNIX_TIME               set current date and time\r\n" };

const char CtBotCli::audio_[] { "audio (a)\r\n"
                                "\ton                           enable audio amplifier\r\n"
                                "\toff                          shutdown amp, stop currently playing wavefile\r\n"
                                "\tvol [0;1]                    set volume\r\n"
                                "\tpitch [1;16]                 set pitch for speak\r\n"
                                "\tplay FILENAME                play wavefile FILENAME from SD card\r\n"
                                "\tpause                        toggle play/pause of currently playing wavefile\r\n"
                                "\tstop                         stop currently playing wavefile\r\n"
                                "\tspeak TEXT                   speak TEXT\r\n"
                                "\tsine [0;22050]               play given frequency (needs AUDIO_TEST_AVAILABLE)\r\n" };

const char CtBotCli::filesystem_[] { "fs (f)\r\n"
                                     "\tls DIR                       list files of directory DIR on SD card\r\n"
                                     "\tcat FILE                     print content of FILE on SD card\r\n" };

const char CtBotCli::prog_[] { "prog (p)\r\n"
                               "\trun FILENAME                 run script FILENAME from SD card\r\n"
                               "\tview FILENAME                print script FILENAME to terminal\r\n"
                               "\tcreate NUM FILENAME          create script FILENAME from last NUM commands in history\r\n" };

const char CtBotCli::i2c_[] { "i2c (i)\r\n"
                              "\tselect [1;3] FREQ            select I2C bus to use (1, 2 or 3) and set frequency to FREQ kHz\r\n"
                              "\taddr [0;127]                 set I2C-address of device to use\r\n"
                              "\tread [1, 2, 4] REG           read N bytes from register at address REG\r\n"
                              "\twrite [1, 2, 4] REG DATA     write N bytes (DATA) to register at address REG\r\n"
                              "\tsetbit REG BIT               set bit BIT in register at address REG\r\n"
                              "\tclearbit REG BIT             clear bit BIT in register at address REG\r\n"
                              "\tscan                         scan selected I2C bus and print found device addresses\r\n" };


static __attribute__((noinline)) void crash() {
    // configASSERT(false);
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Warray-bounds"
    volatile uint8_t* ptr { reinterpret_cast<uint8_t*>(1) };
    *ptr = 0;
#pragma GCC diagnostic pop
    portINSTR_SYNC_BARRIER();
}

void CtBotCli::add_helptext(const char* str) {
    texts_.emplace_back(str);
}

void CtBotCli::print() const {
    for (auto& e : texts_) {
        p_ctbot_->get_comm()->debug_print(e, true);
        using namespace std::chrono_literals;
        std::this_thread::sleep_for(10ms);
        p_ctbot_->get_comm()->debug_print("\r\n", true);
    }
}

CtBotCli::CtBotCli(CtBot* p_ctbot) : p_ctbot_ { p_ctbot } {
    configASSERT(p_ctbot_);

    add_helptext(general_);
    add_helptext(config_);
    add_helptext(get_);
    add_helptext(set_);
    if constexpr (CtBotConfig::AUDIO_AVAILABLE) {
        add_helptext(audio_);
    }
    add_helptext(filesystem_);
    if constexpr (CtBotConfig::PROG_AVAILABLE) {
        add_helptext(prog_);
    }
    if constexpr (CtBotConfig::I2C_TOOLS_AVAILABLE) {
        add_helptext(i2c_);
    }
}

void CtBotCli::init_commands() {
    auto p_parser { p_ctbot_->get_cmd_parser() };
    configASSERT(p_parser);

    p_parser->register_cmd(PSTR("help"), "h", [this](const std::string_view&) {
        print();
        return true;
    });

    p_parser->register_cmd(PSTR("halt"), [this](const std::string_view&) {
        p_ctbot_->stop();
        return true;
    });

    p_parser->register_cmd(PSTR("watch"), "w", [this](const std::string_view& args) {
        // TODO: parameter for time?
        if (args.size()) {
            auto p_cmd { new std::string { args } };
            if (p_ctbot_->p_watch_timer_) {
                auto ptr { static_cast<std::string*>(::pvTimerGetTimerID(p_ctbot_->p_watch_timer_)) };
                xTimerStop(p_ctbot_->p_watch_timer_, 0);
                delete ptr;
                xTimerDelete(p_ctbot_->p_watch_timer_, 0);
            }
            p_ctbot_->p_watch_timer_ = ::xTimerCreate(PSTR("watch_t"), pdMS_TO_TICKS(1'000UL), true, p_cmd, [](TimerHandle_t handle) {
                auto& ctbot { CtBot::get_instance() };
                auto ptr { static_cast<std::string*>(::pvTimerGetTimerID(handle)) };
                ctbot.get_cmd_parser()->execute_cmd(*ptr, *ctbot.get_comm());
            });
            if (!p_ctbot_->p_watch_timer_) {
                delete p_cmd;
                return false;
            }
            ::xTimerStart(p_ctbot_->p_watch_timer_, 0);
        } else {
            if (!p_ctbot_->p_watch_timer_) {
                return false;
            }
            auto ptr { static_cast<std::string*>(::pvTimerGetTimerID(p_ctbot_->p_watch_timer_)) };
            xTimerStop(p_ctbot_->p_watch_timer_, 0);
            delete ptr;
            xTimerDelete(p_ctbot_->p_watch_timer_, 0);
            p_ctbot_->p_watch_timer_ = nullptr;
        }
        return true;
    });

    p_parser->register_cmd(PSTR("config"), "c", [this](const std::string_view& args) {
        if (args.find(PSTR("echo")) == 0) {
            return eval_args<bool>(
                [this](bool value) {
                    p_ctbot_->p_comm_->set_echo(value);
                    return true;
                },
                args);
        } else if (args.find(PSTR("viewer")) == 0) {
            return eval_args<bool>(
                [this](bool value) {
                    p_ctbot_->p_comm_->enable_remoteviewer(value);
                    return true;
                },
                args);
        } else if (args.find(PSTR("task")) == 0) {
            const auto args2 { CmdParser::trim_to_first_arg(args) };
            const size_t end { args2.find(' ') };
            const std::string_view taskname { args2.substr(0, end) };
            const uint16_t task_id { p_ctbot_->get_scheduler()->task_get(taskname) };

            if (task_id < 0xffff) {
                return eval_args<bool>(
                    [this, &taskname, task_id](bool value) {
                        if (!value) {
                            p_ctbot_->get_scheduler()->task_suspend(task_id);
                        } else {
                            p_ctbot_->get_scheduler()->task_resume(task_id);
                        }
                        return true;
                    },
                    args2);
            }
        } else if (args.find(PSTR("prehook")) == 0) {
            const auto args2 { CmdParser::trim_to_first_arg(args) };
            const size_t end { args2.find(' ') };
            const std::string_view hookname { args2.substr(0, end) };
            return eval_args<bool>(
                [this, &hookname](bool value) {
                    auto it { p_ctbot_->pre_hooks_.find(hookname) };
                    if (it != p_ctbot_->pre_hooks_.end()) {
                        std::get<1>(it->second) = value;
                        return true;
                    }
                    return false;
                },
                args2);
        } else if (args.find(PSTR("posthook")) == 0) {
            const auto args2 { CmdParser::trim_to_first_arg(args) };
            const size_t end { args2.find(' ') };
            const std::string_view hookname { args2.substr(0, end) };
            return eval_args<bool>(
                [this, &hookname](bool value) {
                    auto it { p_ctbot_->post_hooks_.find(hookname) };
                    if (it != p_ctbot_->post_hooks_.end()) {
                        std::get<1>(it->second) = value;
                        return true;
                    }
                    return false;
                },
                args2);
        } else if (!CtBotConfig::EXTERNAL_SPEEDCTRL && args.find(PSTR("kp")) == 0) {
            return eval_args<int16_t, int16_t>(
                [this](int16_t left, int16_t right) {
                    p_ctbot_->p_speedcontrols_[0]->set_parameters(
                        static_cast<float>(left), p_ctbot_->p_speedcontrols_[0]->get_ki(), p_ctbot_->p_speedcontrols_[0]->get_kd());
                    p_ctbot_->p_speedcontrols_[1]->set_parameters(
                        static_cast<float>(right), p_ctbot_->p_speedcontrols_[1]->get_ki(), p_ctbot_->p_speedcontrols_[1]->get_kd());
                    return true;
                },
                args);
        } else if (!CtBotConfig::EXTERNAL_SPEEDCTRL && args.find(PSTR("ki")) == 0) {
            return eval_args<int16_t, int16_t>(
                [this](int16_t left, int16_t right) {
                    p_ctbot_->p_speedcontrols_[0]->set_parameters(
                        p_ctbot_->p_speedcontrols_[0]->get_kp(), static_cast<float>(left), p_ctbot_->p_speedcontrols_[0]->get_kd());
                    p_ctbot_->p_speedcontrols_[1]->set_parameters(
                        p_ctbot_->p_speedcontrols_[1]->get_kp(), static_cast<float>(right), p_ctbot_->p_speedcontrols_[1]->get_kd());
                    return true;
                },
                args);
        } else if (!CtBotConfig::EXTERNAL_SPEEDCTRL && args.find(PSTR("kd")) == 0) {
            return eval_args<int16_t, int16_t>(
                [this](int16_t left, int16_t right) {
                    p_ctbot_->p_speedcontrols_[0]->set_parameters(
                        p_ctbot_->p_speedcontrols_[0]->get_kp(), p_ctbot_->p_speedcontrols_[0]->get_ki(), static_cast<float>(left));
                    p_ctbot_->p_speedcontrols_[1]->set_parameters(
                        p_ctbot_->p_speedcontrols_[1]->get_kp(), p_ctbot_->p_speedcontrols_[1]->get_ki(), static_cast<float>(right));
                    return true;
                },
                args);
        } else if (CtBotConfig::LCD_AVAILABLE && args.find(PSTR("lcdout")) != args.npos) {
            const size_t s { args.find(' ') + 1 };
            const size_t e { args.find(' ', s) };
            p_ctbot_->p_lcd_->set_output(args.substr(s, e - s));
            return true;
        } else if (args.find(PSTR("led")) == 0) {
            return eval_args<uint8_t, uint8_t>(
                [this](uint8_t mask, uint8_t pwm) {
                    const auto led_mask { static_cast<LedTypes>(mask) };
                    p_ctbot_->p_leds_->set_pwm(led_mask, pwm);
                    if ((p_ctbot_->p_leds_->get() & led_mask) != LedTypes::NONE) {
                        p_ctbot_->p_leds_->off(led_mask);
                        p_ctbot_->p_leds_->on(led_mask);
                    }
                    return true;
                },
                args);
        } else if (args.find(PSTR("enapwm")) == 0) {
            return eval_args<uint8_t, uint8_t>(
                [this](uint8_t mask, uint8_t pwm) {
                    const auto ena_mask { static_cast<LedTypesEna<>>(mask) };
                    p_ctbot_->p_ena_pwm_->set_pwm(static_cast<LedTypesEna<>>(mask), pwm);
                    if ((p_ctbot_->p_ena_pwm_->get() & ena_mask) != LedTypesEna<>::NONE) {
                        p_ctbot_->p_ena_pwm_->off(ena_mask);
                        p_ctbot_->p_ena_pwm_->on(ena_mask);
                    }
                    return true;
                },
                args);
        }
        return false;
    });

    p_parser->register_cmd(PSTR("get"), "g", [this](const std::string_view& args) {
        if (args.find(PSTR("dist")) == 0) {
            p_ctbot_->p_comm_->debug_printf<true>(PP_ARGS("{} {}", p_ctbot_->p_sensors_->get_distance_l(), p_ctbot_->p_sensors_->get_distance_r()));
        } else if (args.find(PSTR("enc")) == 0) {
            if constexpr (CtBotConfig::EXTERNAL_SPEEDCTRL) {
                p_ctbot_->p_comm_->debug_printf<true>(
                    PP_ARGS("{} {}", p_ctbot_->p_speedcontrols_[0]->get_enc_counts(), p_ctbot_->p_speedcontrols_[1]->get_enc_counts()));
            } else {
                p_ctbot_->p_comm_->debug_printf<true>(PP_ARGS("{} {}", p_ctbot_->p_sensors_->get_enc_l().get(), p_ctbot_->p_sensors_->get_enc_r().get()));
            }
        } else if (args.find(PSTR("border")) == 0) {
            p_ctbot_->p_comm_->debug_printf<true>(PP_ARGS("{} {}", p_ctbot_->p_sensors_->get_border_l(), p_ctbot_->p_sensors_->get_border_r()));
        } else if (args.find(PSTR("line")) == 0) {
            p_ctbot_->p_comm_->debug_printf<true>(PP_ARGS("{} {}", p_ctbot_->p_sensors_->get_line_l(), p_ctbot_->p_sensors_->get_line_r()));
        } else if (args.find(PSTR("speed")) == 0) {
            const auto l { static_cast<int16_t>(p_ctbot_->p_speedcontrols_[0]->get_enc_speed()) };
            const auto r { static_cast<int16_t>(p_ctbot_->p_speedcontrols_[1]->get_enc_speed()) };
            p_ctbot_->p_comm_->debug_printf<true>(PP_ARGS("{} {}", l, r));
        } else if (CtBotConfig::MPU6050_AVAILABLE && args.find(PSTR("mpu6050")) == 0 && p_ctbot_->p_sensors_->get_mpu6050()) {
            auto [e1, e2, e3] = p_ctbot_->p_sensors_->get_mpu6050()->get_euler();
            auto [y, p, r] = p_ctbot_->p_sensors_->get_mpu6050()->get_ypr();
            p_ctbot_->p_comm_->debug_printf<true>(PP_ARGS("  {9.4}   {9.4}   {9.4}\r\n", e1, e2, e3));
            p_ctbot_->p_comm_->debug_printf<true>(PP_ARGS("y={9.4} p={9.4} r={9.4}\r\n", y, p, r));
        } else if (!CtBotConfig::EXTERNAL_SPEEDCTRL && args.find(PSTR("motor")) == 0) {
            if (p_ctbot_->p_motors_[0] && p_ctbot_->p_motors_[1]) {
                p_ctbot_->p_comm_->debug_printf<true>(PP_ARGS("{} {}", p_ctbot_->p_motors_[0]->get(), p_ctbot_->p_motors_[1]->get()));
            }
        } else if (args.find(PSTR("servo")) == 0) {
            p_ctbot_->p_comm_->debug_printf<true>(
                PP_ARGS("{}[{s}] ", p_ctbot_->p_servos_[0]->get_position(), p_ctbot_->p_servos_[0]->get_active() ? PSTR("on ") : PSTR("off")));
            if (p_ctbot_->p_servos_[1]) {
                p_ctbot_->p_comm_->debug_printf<true>(
                    PP_ARGS("{}[{s}]", p_ctbot_->p_servos_[1]->get_position(), p_ctbot_->p_servos_[1]->get_active() ? PSTR("on") : PSTR("off")));
            }
        } else if (args.find(PSTR("rc5")) == 0) {
            p_ctbot_->p_comm_->debug_printf<true>(PP_ARGS("{} {} {}", p_ctbot_->p_sensors_->get_rc5().get_addr(), p_ctbot_->p_sensors_->get_rc5().get_cmd(),
                p_ctbot_->p_sensors_->get_rc5().get_toggle()));
        } else if (args.find(PSTR("transmm")) == 0) {
            p_ctbot_->p_comm_->debug_print(p_ctbot_->p_sensors_->get_transport_mm(), true);
        } else if (args.find(PSTR("trans")) == 0) {
            p_ctbot_->p_comm_->debug_print(p_ctbot_->p_sensors_->get_transport(), true);
            // } else if (args.find(PSTR("door")) == 0) {
            //     p_ctbot_->p_comm_->debug_print(p_ctbot_->p_sensors_->get_shutter(), true);
        } else if (args.find(PSTR("led")) == 0) {
            p_ctbot_->p_comm_->debug_printf<true>(PP_ARGS("{#x}", static_cast<uint8_t>(p_ctbot_->p_leds_->get())));
        } else if (args.find(PSTR("volt")) == 0) {
            p_ctbot_->p_comm_->debug_printf<true>(
                PP_ARGS("{.2} V (probably {.2} V per cell)", p_ctbot_->p_sensors_->get_bat_voltage(), p_ctbot_->p_sensors_->get_bat_voltage() / 4.f));
        } else if (args.find(PSTR("currents")) == 0) {
            if constexpr (CtBotConfig::EXTERNAL_SPEEDCTRL) {
                p_ctbot_->p_comm_->debug_printf<true>(PP_ARGS("@5V: {} mA @Motor: {} mA @Servo: {} mA", p_ctbot_->p_sensors_->get_5v_current(),
                    SpeedControlExternal::get_motor_current(), p_ctbot_->p_sensors_->get_servo_current()));
            } else {
                p_ctbot_->p_comm_->debug_printf<true>(
                    PP_ARGS("@5V: {} mA @Servo: {} mA", p_ctbot_->p_sensors_->get_5v_current(), p_ctbot_->p_sensors_->get_servo_current()));
            }
        } else if (args.find(PSTR("tasks")) == 0) {
            p_ctbot_->get_scheduler()->print_task_list(*p_ctbot_->p_comm_);
        } else if (args.find(PSTR("free")) == 0) {
            p_ctbot_->get_scheduler()->print_ram_usage(*p_ctbot_->p_comm_);
            const auto mem_use { AudioMemoryUsage() };
            const auto mem_use_max { AudioMemoryUsageMax() };
            p_ctbot_->p_comm_->debug_printf<true>(PP_ARGS("\r\nAudioMemoryUsage()={} blocks\tmax={} blocks\r\n", mem_use, mem_use_max));
            const float cpu_use { AudioProcessorUsage() };
            const float cpu_use_max { AudioProcessorUsageMax() };
            p_ctbot_->p_comm_->debug_printf<true>(PP_ARGS("AudioProcessorUsage()={.2} %%\tmax={.2} %%", cpu_use, cpu_use_max));
        } else if (args.find(PSTR("params")) == 0) {
            auto dump { p_ctbot_->p_parameter_->dump() };
            p_ctbot_->p_comm_->debug_printf<true>(PP_ARGS("dump=\"{s}\"", dump->c_str()));
        } else if (args.find(PSTR("paramf")) == 0) {
            size_t s { args.find(' ') };
            if (s == args.npos) {
                return false;
            }
            ++s;
            const size_t e { args.find(' ', s) };
            const std::string_view param { args.substr(s, e - s) };
            float x;
            if (p_ctbot_->p_parameter_->get(param, x)) {
                p_ctbot_->p_comm_->debug_printf<true>(PSTR("paramf \"%.*s\"=%f"), param.size(), param.data(), x);
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
            if (p_ctbot_->p_parameter_->get(param, x)) {
                p_ctbot_->p_comm_->debug_printf<true>(PSTR("param \"%.*s\"=%" PRId32), param.size(), param.data(), x);
            } else {
                return false;
            }
        } else if (args.find(PSTR("time")) == 0) {
            std::string tmp;
            if (format_time(tmp, PSTR("%c UTC"))) {
                p_ctbot_->get_comm()->debug_print(tmp, true);
                p_ctbot_->get_comm()->debug_print(PSTR("\r\n"), true);
            }
        } else if (CtBotConfig::EXTERNAL_SPEEDCTRL && args.find(PSTR("sctrl-crc")) == 0) {
            p_ctbot_->p_comm_->debug_printf<true>(PP_ARGS("SpeedControl CRC errors: {}", SpeedControlExternal::get_crc_errors()));
        } else {
            return false;
        }
        p_ctbot_->p_comm_->debug_print(PSTR("\r\n"), true);
        return true;
    });

    p_parser->register_cmd(PSTR("set"), "s", [this](const std::string_view& args) {
        if (args.find(PSTR("speed")) == 0) {
            if (eval_args<float, float>(
                    [this](float left, float right) {
                        p_ctbot_->p_speedcontrols_[0]->set_speed(left);
                        p_ctbot_->p_speedcontrols_[1]->set_speed(right);
                        return true;
                    },
                    args)) {
                return true;
            } else {
                p_ctbot_->p_speedcontrols_[0]->set_speed(0.f);
                p_ctbot_->p_speedcontrols_[1]->set_speed(0.f);
            }
        } else if (!CtBotConfig::EXTERNAL_SPEEDCTRL && args.find(PSTR("motor")) == 0) {
            if (p_ctbot_->p_motors_[0] && p_ctbot_->p_motors_[1]) {
                return eval_args<int16_t, int16_t>(
                    [this](int16_t left, int16_t right) {
                        p_ctbot_->p_motors_[0]->set(left);
                        p_ctbot_->p_motors_[1]->set(right);
                        return true;
                    },
                    args);
            }
        } else if (args.find(PSTR("servo")) == 0) {
            uint8_t servo1;
            if (auto [args2, ec] = CmdParser::split_args(args, servo1); ec == std::errc {}) {
                if (servo1 <= 180) {
                    p_ctbot_->p_servos_[0]->set(servo1);
                } else {
                    p_ctbot_->p_servos_[0]->disable();
                }

                if (p_ctbot_->p_servos_[1]) {
                    eval_args<uint8_t>(
                        [this](uint8_t servo2) {
                            if (servo2 <= 180) {
                                p_ctbot_->p_servos_[1]->set(servo2);
                            } else {
                                p_ctbot_->p_servos_[1]->disable();
                            }
                            return true;
                        },
                        args2);
                    return true;
                }

                return true;
            }
        } else if (args.find(PSTR("rc5")) == 0) {
            return eval_args<uint8_t, uint8_t>(
                [this](uint8_t addr, uint8_t cmd) {
                    p_ctbot_->p_sensors_->get_rc5().set_rc5(addr, cmd, !p_ctbot_->p_sensors_->get_rc5().get_toggle());
                    return true;
                },
                args);
        } else if (args.find(PSTR("enapwm")) == 0) {
            return eval_args<uint8_t, bool>(
                [this](auto pin, auto value) {
                    if (value) {
                        p_ctbot_->p_ena_pwm_->on(static_cast<LedTypesEna<>>(1 << pin));
                    } else {
                        p_ctbot_->p_ena_pwm_->off(static_cast<LedTypesEna<>>(1 << pin));
                    }
                    return true;
                },
                args);
        } else if (args.find(PSTR("ena")) == 0) {
            return eval_args<uint8_t, bool>(
                [this](uint8_t pin, bool value) {
                    if (value) {
                        p_ctbot_->p_ena_->on(static_cast<EnaI2cTypes>(1 << pin));
                    } else {
                        p_ctbot_->p_ena_->off(static_cast<EnaI2cTypes>(1 << pin));
                    }
                    return true;
                },
                args);
        } else if (args.find(PSTR("led")) == 0) {
            return eval_args<uint8_t>(
                [this](uint8_t led) {
                    p_ctbot_->p_leds_->set(static_cast<LedTypes>(led));
                    return true;
                },
                args);
        } else if (CtBotConfig::LCD_AVAILABLE && args.find(PSTR("lcdbl")) == 0) {
            return eval_args<bool>(
                [this](bool value) {
                    p_ctbot_->p_lcd_->set_backlight(value);
                    return true;
                },
                args);
        } else if (CtBotConfig::LCD_AVAILABLE && args.find(PSTR("lcd")) == 0) {
            uint8_t line, column;
            if (auto [p_text, ec] = CmdParser::split_args(args, line, column); ec == std::errc {}) {
                if (!line && !column) {
                    p_ctbot_->p_lcd_->clear();
                    return true;
                }

                p_ctbot_->p_lcd_->set_cursor(line, column);
                p_ctbot_->p_lcd_->print(p_text + 1);
                return true;
            }
        } else if (CtBotConfig::TFT_AVAILABLE && args.find(PSTR("tftbl")) == 0) {
            return eval_args<float>(
                [this](float value) {
                    p_ctbot_->p_tft_->set_backlight(value);
                    return true;
                },
                args);
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
            p_ctbot_->p_comm_->debug_printf<true>(PSTR("key=\"%.*s\"\r\n"), key.size(), key.data());

            const std::string_view val { args.substr(e) };
            p_ctbot_->p_comm_->debug_printf<true>(PSTR("val=\"%.*s\"\r\n"), val.size(), val.data());

            return eval_args<float>(
                [this, key](float value) {
                    p_ctbot_->p_comm_->debug_printf<true>(PP_ARGS("value={}\r\n", value));

                    p_ctbot_->p_parameter_->set<float>(key, value);
                    p_ctbot_->p_parameter_->flush();
                    return true;
                },
                val);
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
            p_ctbot_->p_comm_->debug_printf<true>(PSTR("key=\"%.*s\"\r\n"), key.size(), key.data());

            const std::string_view val { args.substr(e) };
            p_ctbot_->p_comm_->debug_printf<true>(PSTR("val=\"%.*s\"\r\n"), val.size(), val.data());

            return eval_args<int32_t>(
                [this, key](int32_t value) {
                    p_ctbot_->p_comm_->debug_printf<true>(PP_ARGS("value={}\r\n", value));

                    p_ctbot_->p_parameter_->set<int32_t>(key, value);
                    p_ctbot_->p_parameter_->flush();
                    return true;
                },
                val);
        } else if (args.find(PSTR("time")) == 0) {
            if (eval_args<int32_t>(
                    [this](int32_t time) {
                        const auto now { std::chrono::system_clock::from_time_t(time) };
                        free_rtos_std::set_system_clock(now);
                        return true;
                    },
                    args)) {
                return true;
            } else {
                p_ctbot_->get_logger()->log(PSTR("invalid time received\r\n"), true);
            }
        }
        return false;
    });

    if constexpr (CtBotConfig::AUDIO_AVAILABLE) {
        p_ctbot_->p_parser_->register_cmd(PSTR("audio"), "a", [this](const std::string_view& args) {
            if (args.find(PSTR("play")) == 0) {
                return p_ctbot_->play_wav(CmdParser::trim_to_first_arg(args));
            } else if (args.find(PSTR("stop")) == 0) {
                p_ctbot_->p_play_wav_->stop();
                return true;
            } else if (args.find(PSTR("pause")) == 0) {
                p_ctbot_->p_play_wav_->togglePlayPause();
                return true;
            } else if (args.find(PSTR("on")) == 0) {
                p_ctbot_->get_ena()->on(EnaI2cTypes::AUDIO);
                return true;
            } else if (args.find(PSTR("off")) == 0) {
                p_ctbot_->get_ena()->off(EnaI2cTypes::AUDIO);
                p_ctbot_->p_play_wav_->stop();
                if constexpr (CtBotConfig::AUDIO_TEST_AVAILABLE) {
                    p_ctbot_->p_audio_sine_->frequency(0.f);
                }
                return true;
            } else if (args.find(PSTR("vol")) == 0) {
                return eval_args<float>(
                    [this](float volume) {
                        for (auto& e : p_ctbot_->p_audio_mixer_) {
                            e->gain(0, volume);
                            e->gain(1, volume);
                            e->gain(2, volume);
                            e->gain(3, volume);
                        }
                        return true;
                    },
                    args);
            } else if (args.find(PSTR("pitch")) == 0) {
                return eval_args<uint8_t>(
                    [this](uint8_t pitch) {
                        p_ctbot_->p_tts_->set_pitch(pitch);
                        return true;
                    },
                    args);
            } else if (args.find(PSTR("speak")) == 0) {
                if (p_ctbot_->p_tts_->is_playing()) {
                    return false;
                }

                const size_t s { args.find(' ') };
                if (s != args.npos) {
                    return p_ctbot_->p_tts_->speak(args.substr(s + 1), true);
                }
            } else if (CtBotConfig::AUDIO_TEST_AVAILABLE && args.find(PSTR("sine")) == 0) {
                return eval_args<float>(
                    [this](float freq) {
                        p_ctbot_->p_audio_sine_->frequency(freq);
                        return true;
                    },
                    args);
            }

            return false;
        });
    }

    p_parser->register_cmd(PSTR("fs"), "f", [this](const std::string_view& args) {
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

                const auto len { p_ctbot_->p_comm_->debug_print(entry.name(), true) };
                if (!entry.isDirectory()) {
                    if (len <= 15) {
                        p_ctbot_->p_comm_->debug_print('\t', true);
                    }
                    p_ctbot_->p_comm_->debug_printf<true>(PP_ARGS("\t{} KB\r\n", static_cast<uint32_t>(entry.size() / 1'024ULL)));
                } else {
                    p_ctbot_->p_comm_->debug_print(PSTR("\r\n"), true);
                }

                entry.close();
            }

            return true;
        } else if (args.find(PSTR("cat")) == 0) {
            const auto s { args.find(' ') };
            if (s == args.npos) {
                return false;
            }
            const std::string path { args.substr(s + 1) };
            if (!SD.exists(path.c_str())) {
                p_ctbot_->p_comm_->debug_print(PSTR("File not found\r\n"), true);
                return false;
            }
            File file { SD.open(path.c_str()) };
            while (file.available()) {
                const auto c { file.read() };
                if (c > 0) {
                    p_ctbot_->p_comm_->debug_print(static_cast<char>(c), true);
                    if (c == '\n') {
                        p_ctbot_->p_comm_->debug_print('\r', true);
                    }
                }
            }
            p_ctbot_->p_comm_->debug_print(PSTR("\r\n"), true);

            return true;
        }

        return false;
    });

    p_parser->register_cmd(PSTR("sleep"), [this](const std::string_view& args) {
        return eval_args<uint32_t>(
            [this](uint32_t duration) {
                std::this_thread::sleep_for(std::chrono::milliseconds(duration));
                return true;
            },
            args);
    });

    p_parser->register_cmd(PSTR("crash"), [this](const std::string_view&) {
        p_ctbot_->get_serial_cmd()->write_direct('\r');
        p_ctbot_->get_serial_cmd()->write_direct('\n');

        crash();
        return true;
    });

    p_parser->register_cmd(PSTR("stack"), [this](const std::string_view& args) {
        return eval_args<uint32_t>(
            [this](uint32_t task_id) {
                auto p_task { p_ctbot_->get_scheduler()->task_get(task_id) };
                if (!p_task) {
                    return false;
                }

                p_task->print(*p_ctbot_->p_comm_);
                freertos::print_stack_trace(p_task->get_handle());
                return true;
            },
            args);
    });

    if constexpr (CtBotConfig::PROG_AVAILABLE) {
        p_parser->register_cmd(PSTR("prog"), "p", [this](const std::string_view& args) {
            if (args.find(PSTR("run")) == 0) {
                const auto args2 { CmdParser::trim_to_first_arg(args) };
                const size_t end { args2.find(' ') };
                const std::string_view filename { args2.substr(0, end) };

                auto p_cmd_script { std::make_unique<CmdScript>(filename, *p_ctbot_->p_comm_, *p_ctbot_->p_parser_) };
                return p_cmd_script->exec_script();
            } else if (args.find(PSTR("view")) == 0) {
                const auto args2 { CmdParser::trim_to_first_arg(args) };
                const size_t end { args2.find(' ') };
                const std::string_view filename { args2.substr(0, end) };

                auto p_cmd_script { std::make_unique<CmdScript>(filename, *p_ctbot_->p_comm_, *p_ctbot_->p_parser_) };
                return p_cmd_script->print_script();
            } else if (args.find(PSTR("create")) == 0) {
                size_t num;
                if (auto [res, ec] = CmdParser::split_args(args, num); ec == std::errc {}) {
                    const std::string_view str { res, args.cend() };
                    const auto args2 { CmdParser::trim_to_first_arg(str) };
                    const size_t end { args2.find(' ') };
                    const std::string_view filename { args2.substr(0, end) };

                    auto p_cmd_script { std::make_unique<CmdScript>(filename, *p_ctbot_->p_comm_, *p_ctbot_->p_parser_) };
                    return p_cmd_script->create_script(num);
                }
            }

            return false;
        });
    }

    if constexpr (CtBotConfig::I2C_TOOLS_AVAILABLE) {
        p_parser->register_cmd(PSTR("i2c"), "i", [this](const std::string_view& args) {
            static I2C_Service* p_i2c {};
            static uint8_t dev_addr {};
            if (args.find(PSTR("select")) == 0) {
                return eval_args<uint8_t, uint16_t>(
                    [this](uint8_t bus, uint16_t freq) {
                        delete p_i2c;
                        p_i2c = nullptr;
                        if (bus >= 1 && bus <= 3) {
                            p_i2c = new I2C_Service { static_cast<uint8_t>(bus - 1U), static_cast<uint32_t>(freq * 1'000UL),
                                bus == 1 ? CtBotConfig::I2C1_PIN_SDA : (bus == 2 ? CtBotConfig::I2C2_PIN_SDA : CtBotConfig::I2C3_PIN_SDA),
                                bus == 1 ? CtBotConfig::I2C1_PIN_SCL : (bus == 2 ? CtBotConfig::I2C2_PIN_SCL : CtBotConfig::I2C3_PIN_SCL) };
                            configASSERT(p_i2c);
                            return true;
                        }
                        return false;
                    },
                    args);
            } else if (args.find(PSTR("addr")) == 0) {
                if (auto [_, ec] = CmdParser::split_args(args, dev_addr); ec == std::errc {}) {
                    return true;
                }
            } else if (args.find(PSTR("read")) == 0) {
                uint8_t width;
                uint32_t result;
                if (auto [args2, ec] = CmdParser::split_args(args, width); ec == std::errc {}) {
                    return eval_args<uint8_t>(
                        [this, width, &result](uint8_t addr) {
                            switch (width) {
                                case 1: {
                                    uint8_t data;
                                    if (p_i2c->read_reg(dev_addr, addr, data)) {
                                        return false;
                                    } else {
                                        result = data;
                                    }
                                    break;
                                }
                                case 2: {
                                    uint16_t data;
                                    if (p_i2c->read_reg(dev_addr, addr, data)) {
                                        return false;
                                    } else {
                                        result = data;
                                    }
                                    break;
                                }

                                case 4: {
                                    uint32_t data;
                                    if (p_i2c->read_reg(dev_addr, addr, data)) {
                                        return false;
                                    } else {
                                        result = data;
                                    }
                                    break;
                                }

                                default: return false;
                            }
                            p_ctbot_->p_comm_->debug_printf<true>(
                                PP_ARGS("read {}: bus {} dev {#x} addr {#x} = {#x}\r\n", width, p_i2c->get_bus(), dev_addr, addr, result));
                            return true;
                        },
                        args2);
                }
            } else if (args.find(PSTR("write")) == 0) {
                uint8_t width;
                if (auto [args2, ec] = CmdParser::split_args(args, width); ec == std::errc {}) {
                    return eval_args<uint8_t, uint32_t>(
                        [this, width](uint8_t addr, uint32_t data) {
                            switch (width) {
                                case 1:
                                    if (p_i2c->write_reg(dev_addr, addr, static_cast<uint8_t>(data))) {
                                        return false;
                                    }
                                    break;
                                case 2:
                                    if (p_i2c->write_reg(dev_addr, addr, static_cast<uint16_t>(data))) {
                                        return false;
                                    }
                                    break;
                                case 4:
                                    if (p_i2c->write_reg(dev_addr, addr, static_cast<uint32_t>(data))) {
                                        return false;
                                    }
                                    break;
                                default: return false;
                            }
                            p_ctbot_->p_comm_->debug_printf<true>(
                                PP_ARGS("write {}: bus {} dev {#x} addr {#x} = {#x}\r\n", width, p_i2c->get_bus(), dev_addr, addr, data));
                            return true;
                        },
                        args2);
                }
            } else if (args.find(PSTR("setbit")) == 0) {
                return eval_args<uint8_t, uint8_t>(
                    [this](uint8_t addr, uint8_t bit) {
                        if (p_i2c->set_bit(dev_addr, addr, bit, true)) {
                            return false;
                        } else {
                            uint8_t data;
                            p_i2c->read_reg(dev_addr, addr, data);
                            p_ctbot_->p_comm_->debug_printf<true>(PP_ARGS("bus {} dev {#x} addr {#x} = {#x}\r\n", p_i2c->get_bus(), dev_addr, addr, data));
                            return true;
                        }
                    },
                    args);
            } else if (args.find(PSTR("clearbit")) == 0) {
                return eval_args<uint8_t, uint8_t>(
                    [this](uint8_t addr, uint8_t bit) {
                        if (p_i2c->set_bit(dev_addr, addr, bit, false)) {
                            return false;
                        } else {
                            uint8_t data;
                            p_i2c->read_reg(dev_addr, addr, data);
                            p_ctbot_->p_comm_->debug_printf<true>(PP_ARGS("bus {} dev {#x} addr {#x} = {#x}\r\n", p_i2c->get_bus(), dev_addr, addr, data));
                            return true;
                        }
                    },
                    args);
            } else if (args.find(PSTR("scan")) == 0) {
                for (uint8_t i { 8 }; i < 120; ++i) {
                    if (p_i2c->test(i)) {
                        p_ctbot_->p_comm_->debug_printf<true>(PP_ARGS("bus {}: dev {#x} found.\r\n", p_i2c->get_bus(), i));
                    }
                }
                return true;
            }

            return false;
        });
    }
}

FLASHMEM bool CtBotCli::format_time(std::string& output, const char* format) const {
    output.resize(32, '\0');
    struct timeval tv;
    ::gettimeofday(&tv, nullptr);
    std::time_t t { tv.tv_sec };
    struct tm* info { std::localtime(&t) };
    return std::strftime(output.data(), output.size(), format, info) > 0 ? true : false;
}

} // namespace ctbot
