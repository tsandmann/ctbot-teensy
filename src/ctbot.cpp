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
#include "ctbot_config.h"
#include "scheduler.h"
#include "leds.h"
#include "display.h"
#include "ena.h"
#include "sensors.h"
#include "motor.h"
#include "speed_control.h"
#include "servo.h"
#include "serial_connection_teensy.h"
#include "cmd_parser.h"
#include "parameter_storage.h"

#include "arm_kinetis_debug.h"
#include "pprintpp.hpp"
#include <cstdlib>
#include <string>
#include <array>
#include <tuple>

#include "SD.h"
#include "SPI.h"
#include "arduino_fixed.h" // cleanup of ugly macro stuff etc.


namespace ctbot {

const char CtBot::usage_text[] { "command\tsubcommand [param]\texplanation\r\n"
                                 "----------------------------------------------------------------------------------\r\n"
                                 "help (h)\t\t\tprint this help message\r\n"
                                 "\r\n"

                                 "halt\t\t\t\tshutdown and put Teensy in sleep mode\r\n"
                                 "\r\n"

                                 "config (c)\r\n"
                                 "\techo [0|1]\t\tset console echo on/off\r\n"
                                 "\ttask ledtest [0|1]\tstart/stop LED test\r\n"
                                 "\ttask sctrl [0|1]\tstart/stop speed controller task\r\n"
                                 "\ttask [taskname] [0|1]\tstart/stop a task\r\n"
                                 "\tk{p,i,d} [0;65535]\tset Kp/Ki/Kd parameter for speed controller\r\n"
                                 "\tswd [0|1]\t\tactivate (1) or deactivate (0) SWD debug port\r\n"
                                 "\r\n"

                                 "get (g)\r\n"
                                 "\tdist\t\t\tprint current distance sensor's values\r\n"
                                 "\tenc\t\t\tprint current encoder's values\r\n"
                                 "\tborder\t\t\tprint current border sensor's values\r\n"
                                 "\tline\t\t\tprint current line sensor's values\r\n"
                                 "\tldr\t\t\tprint current LDR sensor's values\r\n"

                                 "\tspeed\t\t\tprint current speed for left and right wheel\r\n"
                                 "\tmotor\t\t\tprint pwm for left and right motor\r\n"
                                 "\tservo\t\t\tprint setpoints for servos\r\n"
                                 "\trc5\t\t\tprint last received RC5 data\r\n"

                                 "\ttrans\t\t\tprint current transport pocket status\r\n"
                                 "\tdoor\t\t\tprint current door status\r\n"
                                 "\tled\t\t\tprint current LED setting\r\n"

                                 "\ttasks\t\t\tprint task list\r\n"
                                 "\tfree\t\t\tprint free RAM, etc.\r\n"
                                 "\r\n"

                                 "set (s)\r\n"
                                 "\tspeed [-100;100] [=]\tset new speed in % for left and right motor\r\n"
                                 "\tmotor [-16k;16k] [=]\tset new pwm for left and right motor\r\n"
                                 "\tservo [0;180|255] [=]\tset new position for servo 1 and 2, 255 to disable\r\n"
                                 "\tled [0;255]\t\tset new LED setting\r\n"
                                 "\tlcd [1;4] [1;20] TEXT\tprint TEXT on LCD at line and column\r\n"
                                 "\tlcdbl [0;1]\t\tswitch LCD backlight ON (1) or OFF (0)\r\n"
                                 "\r\n" };


CtBot& CtBot::get_instance() {
    static CtBot* p_instance { new CtBot() };
    return *p_instance;
}

CtBot::CtBot()
    : shutdown_ { false }, task_id_ { 0 }, p_serial_usb_ { new SerialConnectionTeensy(0, CtBotConfig::UART0_BAUDRATE) },
      p_swd_debugger_ {} { // initializes serial connection here for debug purpose

    std::atexit([]() {
        CtBot* ptr = &get_instance();
        delete ptr;
    });
}

CtBot::~CtBot() {
    // arduino::Serial.println("CtBot::~CtBot(): CtBot instance deleted.");
}

void CtBot::start() {
    p_scheduler_->print_task_list(*p_comm_);
    p_comm_->debug_print("\n\r", false);

    p_scheduler_->run();

    shutdown();
}

void CtBot::stop() {
    shutdown_ = true;
}

void CtBot::setup() {
    p_scheduler_ = new Scheduler();
    task_id_ = p_scheduler_->task_add("main", TASK_PERIOD_MS, [this]() { return run(); });

    p_sensors_ = new Sensors();

    p_motors_[0] = new Motor(p_sensors_->get_enc_l(), CtBotConfig::MOT_L_PWM_PIN, CtBotConfig::MOT_L_DIR_PIN, true);
    p_motors_[1] = new Motor(p_sensors_->get_enc_r(), CtBotConfig::MOT_R_PWM_PIN, CtBotConfig::MOT_R_DIR_PIN, true);

    p_speedcontrols_[0] = new SpeedControl(p_sensors_->get_enc_l(), *p_motors_[0]);
    p_speedcontrols_[1] = new SpeedControl(p_sensors_->get_enc_r(), *p_motors_[1]);

    p_servos_[0] = new Servo(CtBotConfig::SERVO_1_PIN);
    p_servos_[1] = new Servo(CtBotConfig::SERVO_2_PIN);

    p_leds_ = new Leds();
    p_lcd_ = new Display();

    p_parser_ = new CmdParser();
    p_serial_wifi_ = new SerialConnectionTeensy(5, CtBotConfig::UART5_PIN_RX, CtBotConfig::UART5_PIN_TX, CtBotConfig::UART5_BAUDRATE);
    p_comm_ = new CommInterfaceCmdParser(CtBotConfig::UART_FOR_CMD == 5 ? *p_serial_wifi_ : *p_serial_usb_, *p_parser_, true);

    init_parser();

    if (!(SD.begin(BUILTIN_SDCARD))) {
        p_comm_->debug_print("SD.begin() failed.\r\n", false);
    }

    p_parameter_ = new ParameterStorage("ctbot.jsn");

    if (CtBotConfig::SWD_DEBUGGER_AVAILABLE) {
        p_swd_debugger_ = new ARMKinetisDebug { CtBotConfig::SWD_CLOCK_PIN, CtBotConfig::SWD_DATA_PIN, ARMDebug::LOG_NORMAL };
        if (p_swd_debugger_->begin()) {
            p_comm_->debug_print("p_swd_debugger_->begin() successfull.\r\n", true);

            if (p_swd_debugger_->detect()) {
                p_comm_->debug_print("p_swd_debugger_->detect() successfull.\r\n", true);

                if (p_swd_debugger_->reset(false)) {
                    p_comm_->debug_print("p_swd_debugger_->reset() successfull.\r\n", true);

                    // if (p_swd_debugger_->sys_reset_request()) {
                    //     p_comm_->debug_print("p_swd_debugger_->sys_reset_request() successfull.\r\n", true);
                    // } else {
                    //     p_comm_->debug_print("p_swd_debugger_->sys_reset_request() failed.\r\n", true);
                    // }
                } else {
                    p_comm_->debug_print("p_swd_debugger_->reset() failed.\r\n", true);
                }
            } else {
                p_comm_->debug_print("p_swd_debugger_->detect() failed.\r\n", true);
            }
        } else {
            p_comm_->debug_print("p_swd_debugger_->begin() failed.\r\n", true);
        }
    }

    p_comm_->debug_print("\r\n*** c't-Bot init done. ***\n\r\nType \"help\" (or \"h\") to print help message\n\r\n", true);
}

void CtBot::init_parser() {
    p_parser_->register_cmd("help", 'h', [this](const std::string&) {
        p_comm_->debug_print(usage_text, true);
        return true;
    });

    p_parser_->register_cmd("halt", [this](const std::string&) {
        stop();
        return true;
    });

    p_parser_->register_cmd("config", 'c', [this](const std::string& args) {
        if (args.find("echo") != args.npos) {
            uint8_t v;
            CmdParser::split_args(args, v);
            p_comm_->set_echo(v);
        } else if (args.find("task") != args.npos) {
            const size_t s { args.find(" ") + 1 };
            const size_t e { args.find(" ", s) };
            const std::string taskname { args.substr(s, e - s) };
            const uint16_t task_id { get_scheduler()->task_get(taskname) };

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
        } else if (args.find("kp") != args.npos) {
            int16_t left, right;
            CmdParser::split_args(args, left, right);
            p_speedcontrols_[0]->set_parameters(static_cast<float>(left), p_speedcontrols_[0]->get_ki(), p_speedcontrols_[0]->get_kd());
            p_speedcontrols_[1]->set_parameters(static_cast<float>(right), p_speedcontrols_[1]->get_ki(), p_speedcontrols_[1]->get_kd());
        } else if (args.find("ki") != args.npos) {
            int16_t left, right;
            CmdParser::split_args(args, left, right);
            p_speedcontrols_[0]->set_parameters(p_speedcontrols_[0]->get_kp(), static_cast<float>(left), p_speedcontrols_[0]->get_kd());
            p_speedcontrols_[1]->set_parameters(p_speedcontrols_[1]->get_kp(), static_cast<float>(right), p_speedcontrols_[1]->get_kd());
        } else if (args.find("kd") != args.npos) {
            int16_t left, right;
            CmdParser::split_args(args, left, right);
            p_speedcontrols_[0]->set_parameters(p_speedcontrols_[0]->get_kp(), p_speedcontrols_[0]->get_ki(), static_cast<float>(left));
            p_speedcontrols_[1]->set_parameters(p_speedcontrols_[1]->get_kp(), p_speedcontrols_[1]->get_ki(), static_cast<float>(right));
        } else if (args.find("lcdout") != args.npos) {
            const size_t s { args.find(" ") + 1 };
            const size_t e { args.find(" ", s) };
            const std::string filename { args.substr(s, e - s) };
            p_lcd_->set_output(filename);
        } else if (CtBotConfig::SWD_DEBUGGER_AVAILABLE && args.find("swd") != args.npos) {
            uint8_t v;
            CmdParser::split_args(args, v);
            if (v) {
                if (!p_swd_debugger_->sys_reset_request()) {
                    return false;
                }
            } else {
                if (!p_swd_debugger_->reset(false)) {
                    return false;
                }
            }
        } else {
            return false;
        }
        return true;
    });

    p_parser_->register_cmd("get", 'g', [this](const std::string& args) {
        if (args == "dist") {
            p_comm_->debug_printf<true>(PP_ARGS("{} {}", p_sensors_->get_distance_l(), p_sensors_->get_distance_r()));
        } else if (args == "enc") {
            p_comm_->debug_printf<true>(PP_ARGS("{} {}", p_sensors_->get_enc_l().get(), p_sensors_->get_enc_r().get()));
        } else if (args == "mouse") {
            p_comm_->debug_print("0 0", true); // mouse sensor not implemented
        } else if (args == "border") {
            p_comm_->debug_printf<true>(PP_ARGS("{} {}", p_sensors_->get_border_l(), p_sensors_->get_border_r()));
        } else if (args == "line") {
            p_comm_->debug_printf<true>(PP_ARGS("{} {}", p_sensors_->get_line_l(), p_sensors_->get_line_r()));
        } else if (args == "ldr") {
            p_comm_->debug_printf<true>(PP_ARGS("{} {}", p_sensors_->get_ldr_l(), p_sensors_->get_ldr_r()));
        } else if (args == "speed") {
            const auto l { static_cast<int16_t>(p_sensors_->get_enc_l().get_speed()) };
            const auto r { static_cast<int16_t>(p_sensors_->get_enc_r().get_speed()) };
            p_comm_->debug_printf<true>(PP_ARGS("{} {}", l, r));
        } else if (args == "motor") {
            p_comm_->debug_printf<true>(PP_ARGS("{} {}", p_motors_[0]->get(), p_motors_[1]->get()));
        } else if (args == "servo") {
            p_comm_->debug_printf<true>(PP_ARGS("{}[{s}] ", p_servos_[0]->get_position(), p_servos_[0]->get_active() ? "on " : "off"));
            p_comm_->debug_printf<true>(PP_ARGS("{}[{s}]", p_servos_[1]->get_position(), p_servos_[1]->get_active() ? "on" : "off"));
        } else if (args == "rc5") {
            p_comm_->debug_printf<true>(
                PP_ARGS("{} {} {}", p_sensors_->get_rc5().get_addr(), p_sensors_->get_rc5().get_cmd(), p_sensors_->get_rc5().get_toggle()));
        } else if (args == "trans") {
            p_comm_->debug_print(p_sensors_->get_transport(), true);
        } else if (args == "door") {
            p_comm_->debug_print(p_sensors_->get_shutter(), true);
        } else if (args == "led") {
            p_comm_->debug_printf<true>(PP_ARGS("{#x}", static_cast<uint8_t>(p_leds_->get())));
        } else if (args == "volt") {
            p_comm_->debug_printf<true>(PP_ARGS("{.2} V", p_sensors_->get_bat_voltage()));
        } else if (args == "tasks") {
            get_scheduler()->print_task_list(*p_comm_);
        } else if (args == "free") {
            get_scheduler()->print_ram_usage(*p_comm_);
        } else if (args == "params") {
            auto dump { p_parameter_->dump() };
            p_comm_->debug_printf<true>(PP_ARGS("dump=\"{s}\"", dump->c_str()));
        } else if (args.find("paramf") != args.npos) {
            const size_t s { args.find(" ") };
            if (s == args.npos) {
                return false;
            }
            const std::string param { args.substr(s + 1) };
            float x;
            if (p_parameter_->get(param, x)) {
                p_comm_->debug_printf<true>(PP_ARGS("param \"{s}\"={}", param.c_str(), x));
            } else {
                return false;
            }
        } else if (args.find("param") != args.npos) {
            const size_t s { args.find(" ") };
            if (s == args.npos) {
                p_comm_->debug_printf<true>(PP_ARGS("args=\"{s}\"\r\n", args.c_str()));
                return false;
            }
            const std::string param { args.substr(s + 1) };
            int32_t x;
            if (p_parameter_->get(param, x)) {
                p_comm_->debug_printf<true>(PP_ARGS("param \"{s}\"={}", param.c_str(), x));
            } else {
                return false;
            }
        } else {
            return false;
        }
        p_comm_->debug_print("\r\n", true);
        return true;
    });

    p_parser_->register_cmd("set", 's', [this](const std::string& args) {
        if (args.find("speed") != args.npos) {
            int16_t left, right;
            CmdParser::split_args(args, left, right);
            p_speedcontrols_[0]->set_speed(static_cast<float>(left));
            p_speedcontrols_[1]->set_speed(static_cast<float>(right));
        } else if (args.find("motor") != args.npos) {
            int16_t left, right;
            CmdParser::split_args(args, left, right);
            p_motors_[0]->set(left);
            p_motors_[1]->set(right);
        } else if (args.find("servo") != args.npos) {
            uint8_t s1, s2;
            CmdParser::split_args(args, s1, s2);
            if (!s2) {
                if (args.find(" ", 7) == args.npos) {
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
            if (s2 <= 180) {
                p_servos_[1]->set(s2);
            } else {
                p_servos_[1]->disable();
            }
        } else if (args.find("led") != args.npos) {
            uint8_t led;
            CmdParser::split_args(args, led);
            p_leds_->set(static_cast<ctbot::LedTypes>(led));
        } else if (args.find("lcdbl") != args.npos) {
            bool v;
            CmdParser::split_args(args, v);
            p_lcd_->set_backlight(v);
        } else if (args.find("lcd") != args.npos) {
            uint8_t line, column;
            char* ptr { CmdParser::split_args(args, line, column) };
            if (!line && !column) {
                p_lcd_->clear();
                return true;
            }
            p_lcd_->set_cursor(line, column);
            if (args.length() <= static_cast<size_t>(ptr - args.c_str())) {
                return false;
            }
            p_lcd_->print(++ptr);
        } else if (args.find("paramf") != args.npos) {
            const size_t s { args.find(" ") };
            if (s == args.npos) {
                return false;
            }
            const size_t e { args.find(" ", s + 1) };
            if (e == args.npos) {
                return false;
            }
            const std::string key { args.substr(s + 1, e - (s + 1)) };
            p_comm_->debug_printf<true>(PP_ARGS("key=\"{s}\"\r\n", key.c_str()));
            const std::string val { args.substr(e + 1) };
            p_comm_->debug_printf<true>(PP_ARGS("val=\"{s}\"\r\n", val.c_str()));
            const float value { std::stof(val) };
            p_comm_->debug_printf<true>(PP_ARGS("\"{s}\"={}\r\n", key.c_str(), value));
            p_parameter_->set<float>(key, value);
            p_parameter_->flush();
        } else if (args.find("param") != args.npos) {
            const size_t s { args.find(" ") };
            if (s == args.npos) {
                return false;
            }
            const size_t e { args.find(" ", s + 1) };
            if (e == args.npos) {
                return false;
            }
            const std::string key { args.substr(s + 1, e - (s + 1)) };
            p_comm_->debug_printf<true>(PP_ARGS("key=\"{s}\"\r\n", key.c_str()));
            const std::string val { args.substr(e + 1) };
            p_comm_->debug_printf<true>(PP_ARGS("val=\"{s}\"\r\n", val.c_str()));
            const int32_t value { std::stol(val) };
            p_comm_->debug_printf<true>(PP_ARGS("\"{s}\"={}\r\n", key.c_str(), value));
            p_parameter_->set<int32_t>(key, value);
            p_parameter_->flush();
        } else {
            return false;
        }
        return true;
    });
}

void CtBot::run() {
    p_sensors_->update();

    if (shutdown_) {
        shutdown();
    }
}

void CtBot::shutdown() {
    // FIXME: -> destructor?
    p_comm_->debug_print("System shutting down...\r\n", false);
    p_comm_->flush();
    p_serial_wifi_->flush();
    p_serial_usb_->flush();

    Scheduler::enter_critical_section();
    p_speedcontrols_[0]->set_speed(0.f);
    p_speedcontrols_[1]->set_speed(0.f);
    p_motors_[0]->set(0);
    p_motors_[1]->set(0);
    p_servos_[0]->disable();
    p_servos_[1]->disable();
    p_lcd_->clear();
    p_lcd_->set_backlight(false);
    p_leds_->set(LedTypes::NONE);
    p_sensors_->disable_all();

    if (CtBotConfig::SWD_DEBUGGER_AVAILABLE) {
        delete p_swd_debugger_;
    }

    delete p_parameter_;
    // arduino::Serial.println("p_parameter_ deleted.");
    delete p_comm_;
    // arduino::Serial.println("p_comm_ deleted.");
    delete p_serial_wifi_;
    // arduino::Serial.println("p_serial_wifi_ deleted.");
    delete p_parser_;
    // arduino::Serial.println("p_parser_ deleted.");
    delete p_lcd_;
    // arduino::Serial.println("p_lcd_ deleted.");
    delete p_leds_;
    // arduino::Serial.println("p_leds_ deleted.");
    delete p_servos_[0];
    delete p_servos_[1];
    // arduino::Serial.println("p_servos_ deleted.");
    delete p_speedcontrols_[0];
    delete p_speedcontrols_[1];
    // arduino::Serial.println("p_speedcontrols_ deleted.");
    delete p_motors_[0];
    delete p_motors_[1];
    // arduino::Serial.println("p_motors_ deleted.");
    delete p_sensors_;
    // arduino::Serial.println("p_sensors_ deleted.");
    delete p_scheduler_;
    // arduino::Serial.println("p_scheduler_ deleted.");
    delete p_serial_usb_;

    Scheduler::exit_critical_section();
    arduino::Serial.println("CtBot::shutdown(): stopping scheduler...");
    Scheduler::stop();
}

} // namespace ctbot
