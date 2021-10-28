/*
 * This file is part of the ct-Bot teensy framework.
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
 * @file    cmd_parser.h
 * @brief   Command interface parser
 * @author  Timo Sandmann
 * @date    13.05.2018
 */

#include "cmd_parser.h"
#include "comm_interface.h"

#include "pprintpp.hpp"

#include <cstdlib>


namespace ctbot {

CmdParser::CmdParser() : echo_ {} {}

void CmdParser::register_cmd(const std::string& cmd, func_t&& func) {
    commands_[cmd] = func;
}

void CmdParser::register_cmd(const std::string& cmd, const char cmd_short, func_t&& func) {
    register_cmd(cmd, std::move(func));

    commands_[std::string(&cmd_short, 1)] = func;
}

bool CmdParser::execute_cmd(const std::string_view& cmd, CommInterface& comm) {
    if (!cmd.size()) {
        return false;
    }

    if (cmd[0] == '#') {
        // just a comment
        return true;
    }

    const auto arg_pos { cmd.find(" ") };
    const auto cmd_str { cmd.substr(0, arg_pos) };

    const auto it { commands_.find(cmd_str) };
    if (it == commands_.end()) {
        if (echo_) {
            comm.set_color(CommInterface::Color::RED, CommInterface::Color::BLACK);
            comm.set_attribute(CommInterface::Attribute::BOLD);
            comm.debug_print("ERROR", true);
            comm.set_color(CommInterface::Color::WHITE, CommInterface::Color::BLACK);
            comm.debug_print(": command \"", true);
            comm.debug_print(cmd, true);
            comm.debug_print("\" not found.\r\n", true);
            comm.set_attribute(CommInterface::Attribute::NORMAL);
            if (DEBUG_) {
                comm.set_color(CommInterface::Color::WHITE, CommInterface::Color::BLACK);
                comm.debug_printf<true>(PP_ARGS("cmd.size={}\r\n", cmd.size()));
                for (size_t i {}; i < cmd.size(); ++i) {
                    comm.debug_printf<true>(PP_ARGS("{#x} ", static_cast<uint16_t>(cmd[i])));
                }
            }
            comm.debug_print("\r\n", true);
        }
        return false;
    }

    std::string args; // FIXME: string_view
    if (arg_pos != std::string::npos && cmd.size() > arg_pos + 1) {
        args = cmd.substr(arg_pos + 1);
    }

    const bool res { it->second(args) };

    if (res && echo_) {
        comm.set_color(CommInterface::Color::GREEN, CommInterface::Color::BLACK);
        comm.set_attribute(CommInterface::Attribute::BOLD);
        comm.debug_print("OK\r\n", true);
        comm.set_attribute(CommInterface::Attribute::NORMAL);
        comm.set_color(CommInterface::Color::WHITE, CommInterface::Color::BLACK);
    } else if (echo_) {
        comm.set_color(CommInterface::Color::RED, CommInterface::Color::BLACK);
        comm.debug_print("ERROR\r\n", true);
        comm.set_attribute(CommInterface::Attribute::NORMAL);
        comm.set_color(CommInterface::Color::WHITE, CommInterface::Color::BLACK);
    }

    return res;
}

bool CmdParser::parse(const std::string_view& in, CommInterface& comm) {
    if (!in.empty()) {
        history_.emplace_front(in);
        if (history_.size() > HISTORY_SIZE_) {
            history_.pop_back();
        }
        return execute_cmd(history_.front(), comm);
    }

    return false;
}

} // namespace ctbot
