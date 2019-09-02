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
 * @file    help_texts.h
 * @brief   Texts for help message of console interface
 * @author  Timo Sandmann
 * @date    31.08.2019
 */

#pragma once

#include <string_view>
#include <vector>


namespace ctbot {

class CommInterface;

class CtBotHelpTexts {
    static std::vector<std::string_view> texts_;

    static const char general_[];
    static const char config_[];
    static const char get_[];
    static const char set_[];
    static const char audio_[];
    static const char filesystem_[];
    static const char prog_[];
    static const char i2c_[];

    static std::string_view create_sv(const char* str);

public:
    static void init();

    static void print(CommInterface& comm);
};
} // namespace ctbot
