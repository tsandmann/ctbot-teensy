/*
 * This file is part of the ct-Bot teensy framework.
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
 * @file    cmd_script.cpp
 * @brief   Command script execution
 * @author  Timo Sandmann
 * @date    25.08.2019
 */

#include "cmd_script.h"
#include "ctbot.h"
#include "cmd_parser.h"
#include "comm_interface.h"
#include "fs_service.h"

#include "pprintpp.hpp"
#include "SPI.h"
#include "arduino_freertos.h" // cleanup of ugly macro stuff etc.


namespace ctbot {

CmdScript::CmdScript(const std::string_view& filename, CommInterface& comm_interface, CmdParser& parser)
    : comm_interface_ { comm_interface }, cmd_parser_ { parser }, filename_ { filename } {}

bool CmdScript::exec_script() {
    if constexpr (DEBUG_) {
        comm_interface_.set_color(CommInterface::Color::YELLOW, CommInterface::Color::BLACK);
        comm_interface_.debug_printf<true>(PP_ARGS("CmdScript::exec_script(): executing script \"{s}\"...\r\n", filename_.c_str()));
        comm_interface_.set_color(CommInterface::Color::WHITE, CommInterface::Color::BLACK);
    }

    const auto res { process_script([this](const std::string_view& str) {
        if (cmd_parser_.get_echo()) {
            comm_interface_.debug_print("> ", true);
            comm_interface_.debug_print(str, true);
            comm_interface_.debug_print("\r\n", true);
        }

        const auto res { cmd_parser_.execute_cmd(str, comm_interface_) };

        if constexpr (DEBUG_) {
            comm_interface_.set_color(CommInterface::Color::YELLOW, CommInterface::Color::BLACK);
            comm_interface_.debug_print(PSTR("command \""), true);
            comm_interface_.debug_print(str, true);
            comm_interface_.debug_printf<true>(PP_ARGS("\" finished with result {}.\r\n", res));
            comm_interface_.set_color(CommInterface::Color::WHITE, CommInterface::Color::BLACK);
        }

        if (!res) {
            comm_interface_.debug_print(PSTR("CmdScript::exec_script(): aborted.\r\n"), true);
            return false;
        }

        return true;
    }) };

    if constexpr (DEBUG_) {
        comm_interface_.set_color(CommInterface::Color::YELLOW, CommInterface::Color::BLACK);
        comm_interface_.debug_print(PSTR("CmdScript::exec_script(): done.\r\n"), true);
        comm_interface_.set_color(CommInterface::Color::WHITE, CommInterface::Color::BLACK);
    }

    return res;
}

bool CmdScript::print_script() {
    return process_script([this](const std::string_view& str) {
        comm_interface_.set_color(CommInterface::Color::YELLOW, CommInterface::Color::BLACK);
        comm_interface_.debug_print(str, true);
        comm_interface_.set_color(CommInterface::Color::WHITE, CommInterface::Color::BLACK);
        comm_interface_.debug_print(PSTR("\r\n"), true);
        return true;
    });
}

bool CmdScript::process_script(std::function<bool(const std::string_view&)> func) {
    if (!CtBot::get_instance().get_fs()) {
        return false;
    }

    if (!CtBot::get_instance().get_fs()->exists(filename_.c_str())) {
        comm_interface_.debug_printf<true>(PP_ARGS("CmdScript::process_script(): file \"{s}\" not found.\r\n", filename_.c_str()));
        return false;
    }

    File file { CtBot::get_instance().get_fs()->open(filename_.c_str()) };
    if constexpr (DEBUG_VERBOSE_) {
        comm_interface_.debug_print(PSTR("CmdScript::process_script(): get_fs()->open() done.\r\n"), true);
    }

    auto p_buffer { std::make_unique<uint32_t[]>(MAX_LINE_LENGTH_ / 4U + 1U) };
    if constexpr (DEBUG_VERBOSE_) {
        comm_interface_.debug_printf<true>(PP_ARGS("CmdScript::process_script(): p_buffer={#x} ", p_buffer.get()));
    }
    auto p_str { reinterpret_cast<char*>(p_buffer.get()) };
    configASSERT(reinterpret_cast<uintptr_t>(p_str) % 4 == 0);

    bool res { true };

    while (file.available()) {
        const auto n { file.readBytesUntil('\n', p_str, (MAX_LINE_LENGTH_ / 4U + 1U) * 4U) };
        if constexpr (DEBUG_VERBOSE_) {
            comm_interface_.debug_printf<true>(PP_ARGS("CmdScript::process_script(): n={}\r\n", n));
        }
        std::string_view str { p_str, n };
        if constexpr (DEBUG_VERBOSE_) {
            comm_interface_.debug_print(PSTR("CmdScript::process_script(): string_view created.\r\n"), true);
            comm_interface_.debug_printf<true>(PP_ARGS("CmdScript::process_script(): str.size={}\r\n", str.size()));
            for (size_t i {}; i < str.size(); ++i) {
                comm_interface_.debug_printf<true>(PP_ARGS("{#x} ", static_cast<uint16_t>(str[i])));
            }
            comm_interface_.debug_print(PSTR("\r\n"), true);
        }

        const auto pos { str.find_first_of('\r') };
        if (pos != str.npos) {
            if constexpr (DEBUG_VERBOSE_) {
                comm_interface_.debug_printf<true>(PP_ARGS("CmdScript::process_script(): pos={}\r\n", pos));
            }
            str.remove_suffix(str.size() - pos);
        }

        if constexpr (DEBUG_VERBOSE_) {
            comm_interface_.debug_printf<true>(PP_ARGS("CmdScript::process_script(): str.size={}\r\n", str.size()));
            for (size_t i {}; i < str.size(); ++i) {
                comm_interface_.debug_printf<true>(PP_ARGS("{#x} ", static_cast<uint16_t>(str[i])));
            }
            comm_interface_.debug_print(PSTR("\r\n"), true);
        }

        if (!func(str)) {
            res = false;
            break;
        }
    }

    file.close();

    return res;
}

bool CmdScript::create_script(const size_t history_depth) {
    if (!CtBot::get_instance().get_fs()) {
        return false;
    }

    if (filename_.size() > 8U + 1U + 3U) {
        return false;
    }
    const auto dot { filename_.find_first_of(".") };
    if (dot != filename_.npos && dot > 8U) {
        return false;
    }
    if (dot == filename_.npos && filename_.size() > 8U) {
        return false;
    }

    File file { CtBot::get_instance().get_fs()->open(filename_.c_str(), static_cast<uint8_t>(FILE_WRITE_BEGIN)) };
    if (!file) {
        return false;
    }

    for (size_t i { history_depth + 1U }; i > 1; --i) {
        const auto history { cmd_parser_.get_history(i) };
        if (history) {
            const auto str { history.value() };
            if constexpr (DEBUG_VERBOSE_) {
                comm_interface_.debug_printf<true>(PP_ARGS("str.size={}\r\n", str.size()));
                for (size_t i {}; i < str.size(); ++i) {
                    comm_interface_.debug_printf<true>(PP_ARGS("{#x} ", static_cast<uint16_t>(str.at(i))));
                }
                comm_interface_.debug_print(PSTR("\r\n"), true);
            }

            auto pos { str.find_first_of('\r') };
            if (pos == str.npos) {
                pos = str.size();
            }

            if constexpr (DEBUG_VERBOSE_) {
                comm_interface_.debug_printf<true>(PP_ARGS("pos={}\r\n", pos));
            }

            file.write(str.data(), pos);
            file.write(PSTR("\r\n"));
        }
    }

    file.close();

    return true;
}

} // namespace ctbot
