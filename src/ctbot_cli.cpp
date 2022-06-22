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
const char CtBotCli::general_[] { "command subcommand [param]     explanation\r\n"
                                  "----------------------------------------------------------------------------------\r\n"
                                  "help (h)                       print this help message\r\n"
                                  "halt                           shutdown and put Teensy in sleep mode\r\n"
                                  "sleep MS                       sleep for MS milliseconds\r\n" };

const char CtBotCli::config_[] { "config (c)\r\n"
                                 "\techo [0|1]                   set console echo on/off\r\n"
                                 "\ttask ledtest [0|1]           start/stop LED test\r\n"
                                 "\ttask sctrl [0|1]             start/stop speed controller task\r\n"
                                 "\ttask [taskname] [0|1]        start/stop a task\r\n"
                                 "\tk{p,i,d} [0;65535]           set Kp/Ki/Kd parameter for speed controller\r\n" };

const char CtBotCli::get_[] { "get (g)\r\n"
                              "\tdist                         print current distance sensor's values\r\n"
                              "\tenc                          print current encoder's values\r\n"
                              "\tborder                       print current border sensor's values\r\n"
                              "\tline                         print current line sensor's values\r\n"
                              "\tldr                          print current LDR sensor's values\r\n"

                              "\tspeed                        print current speed for left and right wheel\r\n"
                              "\tmotor                        print pwm for left and right motor\r\n"
                              "\tservo                        print setpoints for servos\r\n"
                              "\trc5                          print last received RC5 data\r\n"

                              "\ttrans                        print current transport pocket status\r\n"
                              "\tdoor                         print current door status\r\n"
                              "\tled                          print current LED setting\r\n"

                              "\ttasks                        print task list\r\n"
                              "\tfree                         print free RAM, etc.\r\n" };

const char CtBotCli::set_[] { "set (s)\r\n"
                              "\tspeed [-100;100] [=]         set new speed in % for left and right motor\r\n"
                              "\tmotor [-16k;16k] [=]         set new pwm for left and right motor\r\n"
                              "\tservo [0;180|255] [=]        set new position for servo 1 and 2, 255 to disable\r\n"
                              "\tled [0;255]                  set LED mask\r\n"
                              "\tlcd [1;4] [1;20] TEXT        print TEXT on LCD at line and column\r\n"
                              "\tlcdbl [0;1]                  switch LCD backlight ON (1) or OFF (0)\r\n" };

const char CtBotCli::audio_[] { "audio (a)\r\n"
                                "\ton                           enable audio amplifier\r\n"
                                "\toff                          shutdown amp, stop currently playing wavefile\r\n"
                                "\tvol VOLUME                   set volume to VOLUME (0.0;1.0]\r\n"
                                "\tpitch PITCH                  set pitch for speak to PITCH [1;16]\r\n"
                                "\tplay FILENAME                play wavefile FILENAME from SD card\r\n"
                                "\tspeak TEXT                   speak TEXT\r\n" };

const char CtBotCli::filesystem_[] { "fs (f)\r\n"
                                     "\tls DIR                       list files of directory DIR on SD card\r\n" };

const char CtBotCli::prog_[] { "prog (p)\r\n"
                               "\trun FILENAME                 run script FILENAME from SD card\r\n"
                               "\tview FILENAME                print script FILENAME to terminal\r\n"
                               "\tcreate NUM FILENAME          create script FILENAME from last NUM commands in history\r\n" };

const char CtBotCli::i2c_[] { "i2c (i)\r\n"
                              "\tselect [0;3] FREQ            select I2C bus to use (0, 1, 2 or 3) and set frequency to FREQ kHz\r\n"
                              "\taddr ADDRESS                 set I2C-address of device to use\r\n"
                              "\tread8 REG                    read 1 byte from register at address REG\r\n"
                              "\tread16 REG                   read 2 bytes from register at address REG\r\n"
                              "\tread32 REG                   read 4 bytes from register at address REG\r\n"
                              "\twrite8 REG DATA              write 1 byte (DATA) in register at address REG\r\n"
                              "\twrite16 REG DATA             write 2 bytes (DATA) in register at address REG\r\n"
                              "\twrite32 REG DATA             write 4 bytes (DATA) in register at address REG\r\n" };


static __attribute__((noinline)) void crash() {
    // configASSERT(false);
    volatile uint8_t* ptr { reinterpret_cast<uint8_t*>(1) };
    *ptr = 0;
    portINSTR_SYNC_BARRIER();
}

std::string_view CtBotCli::create_sv(const char* str) {
    return std::move(std::string_view { str, std::strlen(str) });
}

void CtBotCli::add_helptext(const char* str) {
    texts_.emplace_back(create_sv(str));
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
    if (CtBotConfig::AUDIO_AVAILABLE) {
        add_helptext(audio_);
    }
    add_helptext(filesystem_);
    if (CtBotConfig::PROG_AVAILABLE) {
        add_helptext(prog_);
    }
    if (CtBotConfig::I2C_TOOLS_AVAILABLE) {
        add_helptext(i2c_);
    }
}

void CtBotCli::init_commands() {
    auto p_parser { p_ctbot_->get_cmd_parser() };
    configASSERT(p_parser);

    p_parser->register_cmd(PSTR("help"), 'h', [this](const std::string_view&) FLASHMEM {
        print();
        return true;
    });

    p_parser->register_cmd(PSTR("halt"), [this](const std::string_view&) {
        p_ctbot_->stop();
        return true;
    });

    p_parser->register_cmd(PSTR("watch"), 'w', [this](const std::string_view& args) {
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

    p_parser->register_cmd(PSTR("config"), 'c', [this](const std::string_view& args) {
        if (args.find(PSTR("echo")) == 0) {
            uint8_t v;
            CmdParser::split_args(args, v);
            p_ctbot_->p_comm_->set_echo(v);
        } else if (args.find(PSTR("viewer")) == 0) {
            uint8_t v;
            CmdParser::split_args(args, v);
            p_ctbot_->p_comm_->enable_remoteviewer(v);
        } else if (args.find(PSTR("task")) == 0) {
            const size_t s { args.find(' ') + 1 };
            const size_t e { args.find(' ', s) };
            const uint16_t task_id { p_ctbot_->get_scheduler()->task_get(args.substr(s, e - s)) };

            if (task_id < 0xffff) {
                uint8_t v;
                CmdParser::split_args(args.substr(s), v);
                if (!v) {
                    p_ctbot_->get_scheduler()->task_suspend(task_id);
                } else {
                    p_ctbot_->get_scheduler()->task_resume(task_id);
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
            auto it { p_ctbot_->pre_hooks_.find(hookname) };
            if (it != p_ctbot_->pre_hooks_.end()) {
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
            auto it { p_ctbot_->post_hooks_.find(hookname) };
            if (it != p_ctbot_->post_hooks_.end()) {
                std::get<1>(it->second) = v;
            } else {
                return false;
            }
        } else if (args.find(PSTR("kp")) == 0) {
            int16_t left, right;
            CmdParser::split_args(args, left, right);
            p_ctbot_->p_speedcontrols_[0]->set_parameters(
                static_cast<float>(left), p_ctbot_->p_speedcontrols_[0]->get_ki(), p_ctbot_->p_speedcontrols_[0]->get_kd());
            p_ctbot_->p_speedcontrols_[1]->set_parameters(
                static_cast<float>(right), p_ctbot_->p_speedcontrols_[1]->get_ki(), p_ctbot_->p_speedcontrols_[1]->get_kd());
        } else if (args.find(PSTR("ki")) == 0) {
            int16_t left, right;
            CmdParser::split_args(args, left, right);
            p_ctbot_->p_speedcontrols_[0]->set_parameters(
                p_ctbot_->p_speedcontrols_[0]->get_kp(), static_cast<float>(left), p_ctbot_->p_speedcontrols_[0]->get_kd());
            p_ctbot_->p_speedcontrols_[1]->set_parameters(
                p_ctbot_->p_speedcontrols_[1]->get_kp(), static_cast<float>(right), p_ctbot_->p_speedcontrols_[1]->get_kd());
        } else if (args.find(PSTR("kd")) == 0) {
            int16_t left, right;
            CmdParser::split_args(args, left, right);
            p_ctbot_->p_speedcontrols_[0]->set_parameters(
                p_ctbot_->p_speedcontrols_[0]->get_kp(), p_ctbot_->p_speedcontrols_[0]->get_ki(), static_cast<float>(left));
            p_ctbot_->p_speedcontrols_[1]->set_parameters(
                p_ctbot_->p_speedcontrols_[1]->get_kp(), p_ctbot_->p_speedcontrols_[1]->get_ki(), static_cast<float>(right));
        } else if (CtBotConfig::LCD_AVAILABLE && args.find(PSTR("lcdout")) != args.npos) {
            const size_t s { args.find(' ') + 1 };
            const size_t e { args.find(' ', s) };
            p_ctbot_->p_lcd_->set_output(args.substr(s, e - s));
        } else if (args.find(PSTR("led")) == 0) {
            uint8_t mask, pwm;
            CmdParser::split_args(args, mask, pwm);
            const auto led_mask { static_cast<LedTypes>(mask) };
            p_ctbot_->p_leds_->set_pwm(led_mask, pwm);
            if ((p_ctbot_->p_leds_->get() & led_mask) != LedTypes::NONE) { // FIXME: check for individual LEDs
                p_ctbot_->p_leds_->off(led_mask);
                p_ctbot_->p_leds_->on(led_mask);
            }
        } else if (args.find(PSTR("enapwm")) == 0) {
            uint8_t mask, pwm;
            CmdParser::split_args(args, mask, pwm);
            p_ctbot_->p_ena_pwm_->set_pwm(static_cast<LedTypesEna>(mask), pwm);
        } else {
            return false;
        }
        return true;
    });

    p_parser->register_cmd(PSTR("get"), 'g', [this](const std::string_view& args) {
        if (args.find(PSTR("dist")) == 0) {
            p_ctbot_->p_comm_->debug_printf<true>(PP_ARGS("{} {}", p_ctbot_->p_sensors_->get_distance_l(), p_ctbot_->p_sensors_->get_distance_r()));
        } else if (args.find(PSTR("enc")) == 0) {
            p_ctbot_->p_comm_->debug_printf<true>(PP_ARGS("{} {}", p_ctbot_->p_sensors_->get_enc_l().get(), p_ctbot_->p_sensors_->get_enc_r().get()));
        } else if (args.find(PSTR("border")) == 0) {
            p_ctbot_->p_comm_->debug_printf<true>(PP_ARGS("{} {}", p_ctbot_->p_sensors_->get_border_l(), p_ctbot_->p_sensors_->get_border_r()));
        } else if (args.find(PSTR("line")) == 0) {
            p_ctbot_->p_comm_->debug_printf<true>(PP_ARGS("{} {}", p_ctbot_->p_sensors_->get_line_l(), p_ctbot_->p_sensors_->get_line_r()));
        } else if (args.find(PSTR("ldr")) == 0) {
            p_ctbot_->p_comm_->debug_printf<true>(PP_ARGS("{} {}", p_ctbot_->p_sensors_->get_ldr_l(), p_ctbot_->p_sensors_->get_ldr_r()));
        } else if (args.find(PSTR("speed")) == 0) {
            const auto l { static_cast<int16_t>(p_ctbot_->p_speedcontrols_[0]->get_enc_speed()) };
            const auto r { static_cast<int16_t>(p_ctbot_->p_speedcontrols_[1]->get_enc_speed()) };
            p_ctbot_->p_comm_->debug_printf<true>(PP_ARGS("{} {}", l, r));
        } else if (args.find(PSTR("mpu")) == 0 && p_ctbot_->p_sensors_->get_mpu6050()) {
            auto [e1, e2, e3] = p_ctbot_->p_sensors_->get_mpu6050()->get_euler();
            auto [y, p, r] = p_ctbot_->p_sensors_->get_mpu6050()->get_ypr();
            p_ctbot_->p_comm_->debug_printf<true>(PP_ARGS("  {9.4}   {9.4}   {9.4}\r\n", e1, e2, e3));
            p_ctbot_->p_comm_->debug_printf<true>(PP_ARGS("y={9.4} p={9.4} r={9.4}\r\n", y, p, r));
        } else if (args.find(PSTR("motor")) == 0) {
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
            // } else if (args.find("door") == 0) {
            //     p_ctbot_->p_comm_->debug_print(p_ctbot_->p_sensors_->get_shutter(), true);
        } else if (args.find(PSTR("led")) == 0) {
            p_ctbot_->p_comm_->debug_printf<true>(PP_ARGS("{#x}", static_cast<uint8_t>(p_ctbot_->p_leds_->get())));
        } else if (args.find(PSTR("volt")) == 0) {
            p_ctbot_->p_comm_->debug_printf<true>(
                PP_ARGS("{.2} V (probably {.2} V per cell)", p_ctbot_->p_sensors_->get_bat_voltage(), p_ctbot_->p_sensors_->get_bat_voltage() / 4.f));
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
        } else {
            return false;
        }
        p_ctbot_->p_comm_->debug_print(PSTR("\r\n"), true);
        return true;
    });

    p_parser->register_cmd(PSTR("set"), 's', [this](const std::string_view& args) {
        if (args.find(PSTR("speed")) == 0) {
            float left {}, right {};
            const auto res { CmdParser::split_args(args, left, right) };
            if (!res.size()) {
                p_ctbot_->p_speedcontrols_[0]->set_speed(0.f);
                p_ctbot_->p_speedcontrols_[1]->set_speed(0.f);
                return true;
            }

            p_ctbot_->p_speedcontrols_[0]->set_speed(left);
            p_ctbot_->p_speedcontrols_[1]->set_speed(right);
        } else if (args.find(PSTR("motor")) == 0) {
            if (p_ctbot_->p_motors_[0] && p_ctbot_->p_motors_[1]) {
                int16_t left, right;
                CmdParser::split_args(args, left, right);

                p_ctbot_->p_motors_[0]->set(left);
                p_ctbot_->p_motors_[1]->set(right);
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
                p_ctbot_->p_servos_[0]->set(s1);
            } else {
                p_ctbot_->p_servos_[0]->disable();
            }
            if (p_ctbot_->p_servos_[1]) {
                if (s2 <= 180) {
                    p_ctbot_->p_servos_[1]->set(s2);
                } else {
                    p_ctbot_->p_servos_[1]->disable();
                }
            }
        } else if (args.find(PSTR("rc5")) == 0) {
            uint8_t addr, cmd;
            CmdParser::split_args(args, addr, cmd);
            p_ctbot_->p_sensors_->get_rc5().set_rc5(addr, cmd, !p_ctbot_->p_sensors_->get_rc5().get_toggle());
        } else if (args.find(PSTR("enapwm")) == 0) {
            uint8_t pin;
            bool value;
            CmdParser::split_args(args, pin, value);

            if (value) {
                p_ctbot_->p_ena_pwm_->on(static_cast<LedTypesEna>(1 << pin));
            } else {
                p_ctbot_->p_ena_pwm_->off(static_cast<LedTypesEna>(1 << pin));
            }
        } else if (args.find(PSTR("ena")) == 0) {
            uint8_t pin;
            bool value;
            CmdParser::split_args(args, pin, value);

            if (value) {
                p_ctbot_->p_ena_->on(static_cast<EnaI2cTypes>(1 << pin));
            } else {
                p_ctbot_->p_ena_->off(static_cast<EnaI2cTypes>(1 << pin));
            }
        } else if (args.find(PSTR("led")) == 0) {
            uint8_t led;
            CmdParser::split_args(args, led);

            p_ctbot_->p_leds_->set(static_cast<LedTypes>(led));
        } else if (CtBotConfig::LCD_AVAILABLE && args.find(PSTR("lcdbl")) == 0) {
            bool v;
            CmdParser::split_args(args, v);

            p_ctbot_->p_lcd_->set_backlight(v);
        } else if (CtBotConfig::LCD_AVAILABLE && args.find(PSTR("lcd")) == 0) {
            uint8_t line, column;
            auto sv { CmdParser::split_args(args, line, column) };
            if (!line && !column) {
                p_ctbot_->p_lcd_->clear();
                return true;
            }
            p_ctbot_->p_lcd_->set_cursor(line, column);
            if (sv.empty()) {
                return false;
            }
            p_ctbot_->p_lcd_->print(sv.substr(1));
        } else if (CtBotConfig::TFT_AVAILABLE && args.find(PSTR("tftbl")) == 0) {
            float value {};
            CmdParser::split_args(args, value);

            p_ctbot_->p_tft_->set_backlight(value);
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

            const std::string_view val { args.substr(e + 1) };
            p_ctbot_->p_comm_->debug_printf<true>(PSTR("val=\"%.*s\"\r\n"), val.size(), val.data());

            float value {};
            auto res { CmdParser::split_args(val, value) };
            if (!res.size()) {
                return false;
            }

            p_ctbot_->p_comm_->debug_printf<true>(PP_ARGS("value={}\r\n", value));

            p_ctbot_->p_parameter_->set<float>(key, value);
            p_ctbot_->p_parameter_->flush();
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

            const std::string_view val { args.substr(e + 1) };
            p_ctbot_->p_comm_->debug_printf<true>(PSTR("val=\"%.*s\"\r\n"), val.size(), val.data());

            int32_t value {};
            auto res { CmdParser::split_args(val, value) };
            if (!res.size()) {
                return false;
            }

            p_ctbot_->p_comm_->debug_printf<true>(PP_ARGS("value={}\r\n", value));

            p_ctbot_->p_parameter_->set<int32_t>(key, value);
            p_ctbot_->p_parameter_->flush();
        } else if (args.find(PSTR("time")) == 0) {
            const auto n { args.find(' ') };
            if (n == args.npos) {
                return false;
            }
            std::time_t time;
            CmdParser::split_args(args, time);

            if (time) {
                const auto now { std::chrono::system_clock::from_time_t(time) };
                free_rtos_std::set_system_clock(now);
            } else {
                p_ctbot_->get_logger()->log(PSTR("invalid time received\r\n"), true);
                return false;
            }
        } else {
            return false;
        }
        return true;
    });

    if (CtBotConfig::AUDIO_AVAILABLE) {
        p_ctbot_->p_parser_->register_cmd(PSTR("audio"), 'a', [this](const std::string_view& args) {
            if (args.find(PSTR("play")) == 0) {
                const size_t s { args.find(' ') + 1 };
                const size_t e { args.find(' ', s) };
                return p_ctbot_->play_wav(args.substr(s, e - s));
            } else if (args.find(PSTR("on")) == 0) {
                p_ctbot_->get_ena()->on(EnaI2cTypes::AUDIO);
                using namespace std::chrono_literals;
                std::this_thread::sleep_for(1'500ms);
            } else if (args.find(PSTR("off")) == 0) {
                p_ctbot_->get_ena()->off(EnaI2cTypes::AUDIO);
                p_ctbot_->p_play_wav_->stop();
                if (CtBotConfig::AUDIO_TEST_AVAILABLE) {
                    p_ctbot_->p_audio_sine_->frequency(0.f);
                }
            } else if (args.find(PSTR("vol")) == 0) {
                float volume {};
                auto res { CmdParser::split_args(args, volume) };
                if (!res.size()) {
                    return false;
                }

                for (auto& e : p_ctbot_->p_audio_mixer_) {
                    e->gain(0, volume);
                    e->gain(1, volume);
                    e->gain(2, volume);
                }
            } else if (args.find(PSTR("pitch")) == 0) {
                uint8_t pitch;
                CmdParser::split_args(args, pitch);

                p_ctbot_->p_tts_->set_pitch(pitch);
            } else if (args.find(PSTR("speak")) == 0) {
                if (p_ctbot_->p_tts_->is_playing()) {
                    return false;
                }

                const size_t s { args.find(' ') };
                if (s != args.npos) {
                    return p_ctbot_->p_tts_->speak(args.substr(s + 1), true);
                }
            } else if (CtBotConfig::AUDIO_TEST_AVAILABLE && args.find(PSTR("sine")) == 0) {
                float freq {};
                auto res { CmdParser::split_args(args, freq) };
                if (!res.size()) {
                    return false;
                }

                p_ctbot_->p_audio_sine_->frequency(freq);
            } else {
                return false;
            }
            return true;
        });
    }

    p_parser->register_cmd(PSTR("fs"), 'f', [this](const std::string_view& args) {
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
                    if (len < 14) {
                        p_ctbot_->p_comm_->debug_print('\t', true);
                    }
                    p_ctbot_->p_comm_->debug_printf<true>(PP_ARGS("\t{} KB\r\n", static_cast<uint32_t>(entry.size() / 1'024ULL)));
                } else {
                    p_ctbot_->p_comm_->debug_print(PSTR("\r\n"), true);
                }

                entry.close();
            }
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
        } else {
            return false;
        }

        return true;
    });

    p_parser->register_cmd(PSTR("sleep"), [this](const std::string_view& args) {
        uint32_t duration;
        auto res { CmdParser::split_args(args, duration) };
        if (!res.size()) {
            return false;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(duration));
        return true;
    });

    p_parser->register_cmd(PSTR("crash"), [this](const std::string_view&) {
        p_ctbot_->get_serial_cmd()->write_direct('\r');
        p_ctbot_->get_serial_cmd()->write_direct('\n');

        crash();
        return true;
    });

    p_parser->register_cmd(PSTR("stack"), [this](const std::string_view& args) {
        uint32_t task_id { ~0U };
        CmdParser::split_args(args, task_id);

        auto p_task { p_ctbot_->get_scheduler()->task_get(task_id) };
        if (!p_task) {
            return false;
        }

        p_task->print(*p_ctbot_->p_comm_);
        freertos::print_stack_trace(p_task->get_handle());
        return true;
    });

    if (CtBotConfig::PROG_AVAILABLE) {
        p_parser->register_cmd(PSTR("prog"), 'p', [this](const std::string_view& args) {
            if (args.find(PSTR("run")) == 0) {
                const size_t s { args.find(' ') + 1 };
                const size_t e { args.find(' ', s) };
                const std::string_view filename { args.substr(s, e - s) };

                auto p_cmd_script { std::make_unique<CmdScript>(filename, *p_ctbot_->p_comm_, *p_ctbot_->p_parser_) };
                return p_cmd_script->exec_script();
            } else if (args.find(PSTR("view")) == 0) {
                const size_t s { args.find(' ') + 1 };
                const size_t e { args.find(' ', s) };
                const std::string_view filename { args.substr(s, e - s) };

                auto p_cmd_script { std::make_unique<CmdScript>(filename, *p_ctbot_->p_comm_, *p_ctbot_->p_parser_) };
                return p_cmd_script->print_script();
            } else if (args.find(PSTR("create")) == 0) {
                size_t num {};
                const std::string_view str { CmdParser::split_args(args, num) };
                const size_t s { str.find(' ') + 1 };
                const size_t e { str.find(' ', s) };
                const std::string_view filename { str.substr(s, e - s) };

                auto p_cmd_script { std::make_unique<CmdScript>(filename, *p_ctbot_->p_comm_, *p_ctbot_->p_parser_) };
                return p_cmd_script->create_script(num);
            }

            return false;
        });
    }

    if (CtBotConfig::I2C_TOOLS_AVAILABLE) {
        p_parser->register_cmd(PSTR("i2c"), 'i', [this](const std::string_view& args) {
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
                    p_ctbot_->p_comm_->debug_printf<true>(PP_ARGS("bus {} dev {#x} addr {#x} = {#x}\r\n", p_i2c->get_bus(), dev_addr, addr, data));
                    return true;
                }
            } else if (args.find(PSTR("read16")) == 0) {
                uint8_t addr;
                CmdParser::split_args(args, addr);

                uint16_t data {};
                if (p_i2c->read_reg(dev_addr, addr, data)) {
                    return false;
                } else {
                    p_ctbot_->p_comm_->debug_printf<true>(PP_ARGS("bus {} dev {#x} addr {#x} = {#x}\r\n", p_i2c->get_bus(), dev_addr, addr, data));
                    return true;
                }
            } else if (args.find(PSTR("read32")) == 0) {
                uint8_t addr;
                CmdParser::split_args(args, addr);

                uint32_t data {};
                if (p_i2c->read_reg(dev_addr, addr, data)) {
                    return false;
                } else {
                    p_ctbot_->p_comm_->debug_printf<true>(PP_ARGS("bus {} dev {#x} addr {#x} = {#x}\r\n", p_i2c->get_bus(), dev_addr, addr, data));
                    return true;
                }
            } else if (args.find(PSTR("write8")) == 0) {
                uint8_t addr;
                uint8_t data;
                CmdParser::split_args(args, addr, data);

                if (p_i2c->write_reg(dev_addr, addr, data)) {
                    return false;
                } else {
                    p_ctbot_->p_comm_->debug_printf<true>(PP_ARGS("bus {} dev {#x} addr {#x} = {#x}\r\n", p_i2c->get_bus(), dev_addr, addr, data));
                    return true;
                }
            } else if (args.find(PSTR("write16")) == 0) {
                uint8_t addr;
                uint16_t data;
                CmdParser::split_args(args, addr, data);

                if (p_i2c->write_reg(dev_addr, addr, data)) {
                    return false;
                } else {
                    p_ctbot_->p_comm_->debug_printf<true>(PP_ARGS("bus {} dev {#x} addr {#x} = {#x}\r\n", p_i2c->get_bus(), dev_addr, addr, data));
                    return true;
                }
            } else if (args.find(PSTR("write32")) == 0) {
                uint8_t addr;
                uint32_t data;
                CmdParser::split_args(args, addr, data);

                if (p_i2c->write_reg(dev_addr, addr, data)) {
                    return false;
                } else {
                    p_ctbot_->p_comm_->debug_printf<true>(PP_ARGS("bus {} dev {#x} addr {#x} = {#x}\r\n", p_i2c->get_bus(), dev_addr, addr, data));
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
                    p_ctbot_->p_comm_->debug_printf<true>(PP_ARGS("bus {} dev {#x} addr {#x} = {#x}\r\n", p_i2c->get_bus(), dev_addr, addr, data));
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
                    p_ctbot_->p_comm_->debug_printf<true>(PP_ARGS("bus {} dev {#x} addr {#x} = {#x}\r\n", p_i2c->get_bus(), dev_addr, addr, data));
                    return true;
                }
            } else if (args.find(PSTR("scan")) == 0) {
                for (uint8_t i { 8 }; i < 120; ++i) {
                    // if (p_i2c->test(i)) { // FIXME: add scan
                    //     p_ctbot_->p_comm_->debug_printf<true>(PP_ARGS("bus {}: dev {#x} found.\r\n", p_i2c_->get_bus(), i));
                    // }
                }
            } else {
                return false;
            }

            return true;
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
