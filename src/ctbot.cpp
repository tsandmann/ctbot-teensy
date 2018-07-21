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


namespace ctbot {

const char CtBot::usage_text[] { "command\tsubcommand [param]\texplanation\n"
                                 "----------------------------------------------------------------------------------\n"
                                 "help (h)\t\t\tprint this help message\n"
                                 "halt\t\t\t\tshutdown and put Teensy in sleep mode\n"
                                 "\n"

                                 "config (c)\n"
                                 "\techo [0|1]\t\tset console echo on/off\n"
                                 "\ttask ledtest [0|1]\tstart/stop LED test\n"
                                 "\ttask sctrl [0|1]\tstart/stop speed controller task\n"
                                 "\ttask [taskname] [0|1]\tstart/stop a task\n"
                                 "\tk{p,i,d} [0;65535]\tset Kp/Ki/Kd parameter for speed controller\n"
                                 "\n"

                                 "get (g)\n"
                                 "\tdist\t\t\tprint current distance sensor's values\n"
                                 "\tenc\t\t\tprint current encoder's values\n"
                                 "\tborder\t\t\tprint current border sensor's values\n"
                                 "\tline\t\t\tprint current line sensor's values\n"
                                 "\tldr\t\t\tprint current LDR sensor's values\n"

                                 "\tspeed\t\t\tprint current speed for left and right wheel\n"
                                 "\tmotor\t\t\tprint pwm for left and right motor\n"
                                 "\tservo\t\t\tprint setpoints for servos\n"
                                 "\trc5\t\t\tprint last received RC5 data\n"

                                 "\ttrans\t\t\tprint current transport pocket status\n"
                                 "\tdoor\t\t\tprint current door status\n"
                                 "\tled\t\t\tprint current LED setting\n"

                                 "\ttasks\t\t\tprint task list\n"
                                 "\tfree\t\t\tprint free RAM\n"
                                 "\n"

                                 "set (s)\n"
                                 "\tspeed [-100;100] [=]\tset new speed in % for left and right motor\n"
                                 "\tmotor [-16k;16000] [=]\tset new pwm for left and right motor\n"
                                 "\tservo [0;180|255] [=]\tset new position for servo 1 and 2, 255 to disable\n"
                                 "\tled [0;255]\t\tset new LED setting\n"
                                 "\tlcd [1;4] [1;20] TEXT\tprint TEXT on LCD at line and column\n"
                                 "\tlcdbl [0;1]\t\tswitch LCD backlight ON (1) or OFF (0)\n" };

CtBot& CtBot::get_instance() {
    static CtBot instance;
    return instance;
}

CtBot::CtBot() : shutdown_ { false }, p_serial_usb_ { new SerialConnectionTeensy(0, CtBotConfig::UART0_BAUDRATE) } {
    // initializes serial connection for debug purpose
}

void CtBot::stop() {
    shutdown_ = true;
}

void CtBot::setup() {
    p_scheduler_ = new Scheduler();
    p_scheduler_->task_add("main", TASK_PERIOD_MS, 512UL,
        [](void* p_data) {
            CtBot* p_this { reinterpret_cast<CtBot*>(p_data) };
            return p_this->run();
        },
        this);

    p_sensors_ = new Sensors();

    p_motors_[0] = new Motor(p_sensors_->get_enc_l(), CtBotConfig::MOT_L_PWM_PIN, CtBotConfig::MOT_L_DIR_PIN, false);
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

    p_comm_->debug_print("\n*** c't-Bot init done. ***\n\nType \"help\" to print help message\n\n");
}

void CtBot::init_parser() {
    p_parser_->register_cmd("help", 'h', [](const std::string&) {
        CtBot::get_instance().p_comm_->debug_print(usage_text);
        return true;
    });

    p_parser_->register_cmd("halt", [](const std::string&) {
        CtBot::get_instance().stop();
        return true;
    });

    p_parser_->register_cmd("config", 'c', [](const std::string& args) {
        CtBot* const p_this { &CtBot::get_instance() };

        if (args.find("echo") != args.npos) {
            uint8_t v;
            CmdParser::split_args(args, v);
            p_this->p_comm_->set_echo(v);
        } else if (args.find("task") != args.npos) {
            const size_t s { args.find(" ") + 1 };
            const size_t e { args.find(" ", s) };
            const std::string taskname { args.substr(s, e - s) };
            const uint16_t task_id { p_this->get_scheduler()->task_get(taskname) };

            if (task_id < 0xffff) {
                uint8_t v;
                CmdParser::split_args(args.substr(s), v);
                if (!v) {
                    p_this->get_scheduler()->task_suspend(task_id);
                } else {
                    p_this->get_scheduler()->task_resume(task_id);
                }
            } else {
                return false;
            }
        } else if (args.find("kp") != args.npos) {
            int16_t left, right;
            CmdParser::split_args(args, left, right);
            p_this->p_speedcontrols_[0]->set_parameters(static_cast<float>(left), p_this->p_speedcontrols_[0]->get_ki(), p_this->p_speedcontrols_[0]->get_kd());
            p_this->p_speedcontrols_[1]->set_parameters(
                static_cast<float>(right), p_this->p_speedcontrols_[1]->get_ki(), p_this->p_speedcontrols_[1]->get_kd());
        } else if (args.find("ki") != args.npos) {
            int16_t left, right;
            CmdParser::split_args(args, left, right);
            p_this->p_speedcontrols_[0]->set_parameters(p_this->p_speedcontrols_[0]->get_kp(), static_cast<float>(left), p_this->p_speedcontrols_[0]->get_kd());
            p_this->p_speedcontrols_[1]->set_parameters(
                p_this->p_speedcontrols_[1]->get_kp(), static_cast<float>(right), p_this->p_speedcontrols_[1]->get_kd());
        } else if (args.find("kd") != args.npos) {
            int16_t left, right;
            CmdParser::split_args(args, left, right);
            p_this->p_speedcontrols_[0]->set_parameters(p_this->p_speedcontrols_[0]->get_kp(), p_this->p_speedcontrols_[0]->get_ki(), static_cast<float>(left));
            p_this->p_speedcontrols_[1]->set_parameters(
                p_this->p_speedcontrols_[1]->get_kp(), p_this->p_speedcontrols_[1]->get_ki(), static_cast<float>(right));
        } else if (args.find("lcdout") != args.npos) {
            const size_t s { args.find(" ") + 1 };
            const size_t e { args.find(" ", s) };
            const std::string filename { args.substr(s, e - s) };
            p_this->p_lcd_->set_output(filename);
        } else {
            return false;
        }
        return true;
    });

    p_parser_->register_cmd("get", 'g', [](const std::string& args) {
        CtBot* const p_this { &CtBot::get_instance() };

        if (args == "dist") {
            p_this->serial_print(p_this->p_sensors_->get_distance_l(), p_this->p_sensors_->get_distance_r());
        } else if (args == "enc") {
            p_this->serial_print(p_this->p_sensors_->get_enc_l().get(), p_this->p_sensors_->get_enc_r().get());
        } else if (args == "mouse") {
            p_this->serial_print(0, 0); // mouse sensor not implemented
        } else if (args == "border") {
            p_this->serial_print(p_this->p_sensors_->get_border_l(), p_this->p_sensors_->get_border_r());
        } else if (args == "line") {
            p_this->serial_print(p_this->p_sensors_->get_line_l(), p_this->p_sensors_->get_line_r());
        } else if (args == "ldr") {
            p_this->serial_print(p_this->p_sensors_->get_ldr_l(), p_this->p_sensors_->get_ldr_r());
        } else if (args == "speed") {
            const auto l { static_cast<int16_t>(p_this->p_sensors_->get_enc_l().get_speed()) };
            const auto r { static_cast<int16_t>(p_this->p_sensors_->get_enc_r().get_speed()) };
            p_this->serial_print(l, r);
        } else if (args == "motor") {
            p_this->serial_print(p_this->p_motors_[0]->get(), p_this->p_motors_[1]->get());
        } else if (args == "servo") {
            p_this->get_comm()->debug_print(p_this->p_servos_[0]->get_position(), PrintBase::DEC);
            p_this->get_comm()->debug_print(p_this->p_servos_[0]->get_active() ? "[on] " : "[off] ");
            p_this->get_comm()->debug_print(p_this->p_servos_[1]->get_position(), PrintBase::DEC);
            p_this->get_comm()->debug_print(p_this->p_servos_[1]->get_active() ? "[on] " : "[off] ");
        } else if (args == "rc5") {
            p_this->serial_print(p_this->p_sensors_->get_rc5().get_addr(), p_this->p_sensors_->get_rc5().get_cmd(), p_this->p_sensors_->get_rc5().get_toggle());
        } else if (args == "trans") {
            p_this->serial_print(p_this->p_sensors_->get_transport());
        } else if (args == "door") {
            p_this->serial_print(p_this->p_sensors_->get_shutter());
        } else if (args == "led") {
            p_this->p_comm_->debug_print("0x");
            p_this->serial_print_base(static_cast<uint8_t>(p_this->p_leds_->get()), PrintBase::HEX);
        } else if (args == "tasks") {
            p_this->get_scheduler()->print_task_list(*p_this->p_comm_);
        } else if (args == "free") {
            p_this->get_scheduler()->print_free_ram(*p_this->p_comm_);
        } else {
            return false;
        }
        p_this->p_comm_->debug_print('\n');
        return true;
    });

    p_parser_->register_cmd("set", 's', [](const std::string& args) {
        CtBot* const p_this { &CtBot::get_instance() };

        if (args.find("speed") != args.npos) {
            int16_t left, right;
            CmdParser::split_args(args, left, right);
            p_this->p_speedcontrols_[0]->set_speed(static_cast<float>(left));
            p_this->p_speedcontrols_[1]->set_speed(static_cast<float>(right));
        } else if (args.find("motor") != args.npos) {
            int16_t left, right;
            CmdParser::split_args(args, left, right);
            p_this->p_motors_[0]->set(left);
            p_this->p_motors_[1]->set(right);
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
                p_this->p_servos_[0]->set(s1);
            } else {
                p_this->p_servos_[0]->disable();
            }
            if (s2 <= 180) {
                p_this->p_servos_[1]->set(s2);
            } else {
                p_this->p_servos_[1]->disable();
            }
        } else if (args.find("led") != args.npos) {
            uint8_t led;
            CmdParser::split_args(args, led);
            p_this->p_leds_->set(static_cast<ctbot::LedTypes>(led));
        } else if (args.find("lcdbl") != args.npos) {
            bool v;
            CmdParser::split_args(args, v);
            p_this->p_lcd_->set_backlight(v);
        } else if (args.find("lcd") != args.npos) {
            uint8_t line, column;
            char* ptr { CmdParser::split_args(args, line, column) };
            if (!line && !column) {
                p_this->p_lcd_->clear();
                return true;
            }
            p_this->p_lcd_->set_cursor(line, column);
            if (args.length() <= static_cast<size_t>(ptr - args.c_str())) {
                return false;
            }
            p_this->p_lcd_->print(++ptr);
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
    p_comm_->debug_print("System shutting down...\n");
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

    delete p_comm_;
    delete p_serial_wifi_;
    delete p_parser_;
    delete p_lcd_;
    delete p_leds_;
    delete p_servos_[0];
    delete p_servos_[1];
    delete p_speedcontrols_[0];
    delete p_speedcontrols_[1];
    delete p_motors_[0];
    delete p_motors_[1];
    delete p_sensors_;
    delete p_scheduler_;

    Scheduler::exit_critical_section();
    Scheduler::stop();
}

} // namespace ctbot
