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
 * @file    ctbot_cli.h
 * @brief   Command line interface
 * @author  Timo Sandmann
 * @date    20.11.2021
 */

#pragma once

#include "cmd_parser.h"

#include "avr/pgmspace.h"

#include <string_view>
#include <vector>
#include <functional>
#include <tuple>


namespace ctbot {

class CtBot;
class CommInterface;


class CtBotCli {
protected:
    PROGMEM static const char general_[];
    PROGMEM static const char config_[];
    PROGMEM static const char get_[];
    PROGMEM static const char set_[];
    PROGMEM static const char audio_[];
    PROGMEM static const char filesystem_[];
    PROGMEM static const char prog_[];
    PROGMEM static const char i2c_[];

    template <typename... Ts>
    struct ArgsHelper {
        using func = std::function<bool(Ts...)>;
        using types = std::tuple<Ts...>;
    };

    CtBot* p_ctbot_;
    std::vector<std::string_view> texts_;

    template <typename... Ts>
    bool eval_args(ArgsHelper<Ts...>::func&& func, const std::string_view& args) {
        typename ArgsHelper<Ts...>::types values;
        if (auto [_, ec] = CmdParser::split_args(args, values); ec == std::errc {}) {
            return std::apply(func, values);
        }

        return false;
    }

public:
    FLASHMEM CtBotCli(CtBot* p_ctbot);

    FLASHMEM void add_helptext(const char* str);

    FLASHMEM void init_commands();

    FLASHMEM void print() const;

    FLASHMEM bool format_time(std::string& output, const char* format = "%c") const;
};
} // namespace ctbot
