/*
 * This file is part of the c't-Bot teensy framework.
 * Copyright (c) 2019 Timo Sandmann
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
 * @file    help_texts.cpp
 * @brief   Texts for help message of console interface
 * @author  Timo Sandmann
 * @date    31.08.2019
 */

#include "help_texts.h"
#include "ctbot_config.h"
#include "comm_interface.h"

#include <cstring>


namespace ctbot {
std::vector<std::string_view> CtBotHelpTexts::texts_ {};

const char CtBotHelpTexts::general_[] { "command\tsubcommand [param]\texplanation\r\n"
                                        "----------------------------------------------------------------------------------\r\n"
                                        "help (h)\t\t\tprint this help message\r\n"
                                        "halt\t\t\t\tshutdown and put Teensy in sleep mode\r\n"
                                        "sleep\tMS\t\t\tsleep for MS milliseconds\r\n" };

const char CtBotHelpTexts::config_[] { "config (c)\r\n"
                                       "\techo [0|1]\t\tset console echo on/off\r\n"
                                       "\ttask ledtest [0|1]\tstart/stop LED test\r\n"
                                       "\ttask sctrl [0|1]\tstart/stop speed controller task\r\n"
                                       "\ttask [taskname] [0|1]\tstart/stop a task\r\n"
                                       "\tk{p,i,d} [0;65535]\tset Kp/Ki/Kd parameter for speed controller\r\n"
                                       "\tswd [0|1]\t\tactivate (1) or deactivate (0) SWD debug port\r\n" };

const char CtBotHelpTexts::get_[] { "get (g)\r\n"
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
                                    "\tfree\t\t\tprint free RAM, etc.\r\n" };

const char CtBotHelpTexts::set_[] { "set (s)\r\n"
                                    "\tspeed [-100;100] [=]\tset new speed in % for left and right motor\r\n"
                                    "\tmotor [-16k;16k] [=]\tset new pwm for left and right motor\r\n"
                                    "\tservo [0;180|255] [=]\tset new position for servo 1 and 2, 255 to disable\r\n"
                                    "\tled [0;255]\t\tset LED mask\r\n"
                                    "\tlcd [1;4] [1;20] TEXT\tprint TEXT on LCD at line and column\r\n"
                                    "\tlcdbl [0;1]\t\tswitch LCD backlight ON (1) or OFF (0)\r\n" };

const char CtBotHelpTexts::audio_[] { "audio (a)\r\n"
                                      "\tplay FILENAME\t\tplay wavefile FILENAME from SD card\r\n"
                                      "\tstop\t\t\tstop currently playing wavefile\r\n" };

const char CtBotHelpTexts::filesystem_[] { "fs (f)\r\n"
                                           "\tls DIR\t\t\tlist files of directory DIR on SD card\r\n" };

const char CtBotHelpTexts::prog_[] { "prog (p)\r\n"
                                     "\trun FILENAME\t\trun script FILENAME from SD card\r\n"
                                     "\tview FILENAME\t\tprint script FILENAME to terminal\r\n"
                                     "\tcreate NUM FILENAME\tcreate script FILENAME from last NUM commands in history\r\n" };

const char CtBotHelpTexts::i2c_[] { "i2c (i)\r\n"
                                    "\tselect [0;3] FREQ\tselect I2C bus to use (0, 1, 2 or 3) and set frequency to FREQ kHz\r\n"
                                    "\taddr ADDRESS\t\tset I2C-address of device to use\r\n"
                                    "\tread8 REG\t\tread 1 byte from register at address REG\r\n"
                                    "\tread16 REG\t\tread 2 bytes from register at address REG\r\n"
                                    "\tread32 REG\t\tread 4 bytes from register at address REG\r\n"
                                    "\twrite8 REG DATA\t\twrite 1 byte (DATA) in register at address REG\r\n"
                                    "\twrite16 REG DATA\twrite 2 bytes (DATA) in register at address REG\r\n"
                                    "\twrite32 REG DATA\twrite 4 bytes (DATA) in register at address REG\r\n" };

std::string_view CtBotHelpTexts::create_sv(const char* str) {
    return std::move(std::string_view(str, std::strlen(str)));
}

void CtBotHelpTexts::init() {
    texts_.emplace_back(create_sv(general_));
    texts_.emplace_back(create_sv(config_));
    texts_.emplace_back(create_sv(get_));
    texts_.emplace_back(create_sv(set_));
    if (CtBotConfig::AUDIO_AVAILABLE) {
        texts_.emplace_back(create_sv(audio_));
    }
    texts_.emplace_back(create_sv(filesystem_));
    if (CtBotConfig::PROG_AVAILABLE) {
        texts_.emplace_back(create_sv(prog_));
    }
    if (CtBotConfig::I2C_TOOLS_AVAILABLE) {
        texts_.emplace_back(create_sv(i2c_));
    }
}

void CtBotHelpTexts::print(CommInterface& comm) {
    for (auto& e : texts_) {
        comm.debug_print(e, true);
        comm.debug_print("\r\n", true);
    }
}
} // namespace ctbot
