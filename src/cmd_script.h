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
 * @file    cmd_script.h
 * @brief   Command script execution
 * @author  Timo Sandmann
 * @date    25.08.2019
 */

#pragma once

#include <cstdint>
#include <string>
#include <string_view>
#include <functional>


namespace ctbot {

class CommInterface;
class CmdParser;

// FIXME: add documentation
class CmdScript {
    static constexpr bool DEBUG_ { false };
    static constexpr bool DEBUG_VERBOSE_ { false };
    static constexpr size_t MAX_LINE_LENGTH_ { 32 };

    CommInterface& comm_interface_;
    CmdParser& cmd_parser_;
    std::string filename_;

    FLASHMEM bool process_script(std::function<bool(const std::string_view&)> func);

public:
    FLASHMEM CmdScript(const std::string_view& filename, CommInterface& comm_interface, CmdParser& parser);

    FLASHMEM bool exec_script();

    FLASHMEM bool print_script();

    FLASHMEM bool create_script(const size_t history_depth);
};

} // namespace ctbot
