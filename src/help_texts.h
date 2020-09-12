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

#include "avr/pgmspace.h"

#include <string_view>
#include <vector>


namespace ctbot {

class CommInterface;

class CtBotHelpTexts {
    static std::vector<std::string_view> texts_;

    PROGMEM static const char general_[];
    PROGMEM static const char config_[];
    PROGMEM static const char get_[];
    PROGMEM static const char set_[];
    PROGMEM static const char audio_[];
    PROGMEM static const char filesystem_[];
    PROGMEM static const char prog_[];
    PROGMEM static const char i2c_[];

    FLASHMEM static std::string_view create_sv(const char* str);

public:
    FLASHMEM static void init();

    FLASHMEM static void print(CommInterface& comm);
};
} // namespace ctbot
