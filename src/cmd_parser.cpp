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

CmdParser::CmdParser() : echo_ { false } {}

void CmdParser::register_cmd(const std::string& cmd, const func_t& func) {
    commands_[cmd] = func;
}

void CmdParser::register_cmd(const std::string& cmd, const char cmd_short, const func_t& func) {
    register_cmd(cmd, func);

    commands_[std::string(&cmd_short, 1)] = func;
}

bool CmdParser::execute_cmd(const std::string& cmd, CommInterface& comm) {
    if (!cmd.length()) {
        return false;
    }

    const auto arg_pos(cmd.find(" "));
    const auto cmd_str(cmd.substr(0, arg_pos));

    const auto it(commands_.find(cmd_str));
    if (it == commands_.end()) {
        if (echo_) {
            comm.set_color(CommInterface::Color::RED, CommInterface::Color::BLACK);
            comm.set_attribute(CommInterface::Attribute::BOLD);
            comm.debug_print("ERROR", true);
            comm.set_color(CommInterface::Color::WHITE, CommInterface::Color::BLACK);
            comm.debug_printf<true>(PP_ARGS(": command \"{s}\" not found: ", cmd.c_str()));
            comm.set_attribute(CommInterface::Attribute::NORMAL);
            comm.set_color(CommInterface::Color::WHITE, CommInterface::Color::BLACK);
            // comm.debug_printf<false>(PP_ARGS("cmd.size={}\r\n", cmd.size()));
            for (size_t i { 0 }; i < cmd.size(); ++i) {
                comm.debug_printf<false>(PP_ARGS("{#x} ", static_cast<uint16_t>(cmd[i])));
            }
            comm.debug_print("\r\n", false);
        }
        return false;
    }

    std::string args;
    if (arg_pos != std::string::npos && cmd.length() > arg_pos + 1) {
        args = cmd.substr(arg_pos + 1);
    }

    bool res { it->second(args) };

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

bool CmdParser::parse(const char* in, CommInterface& comm) {
    if (*in) {
        history_.emplace_front(in);
        if (history_.size() > HISTORY_SIZE) {
            history_.pop_back();
        }
        return execute_cmd(history_.front(), comm);
    }

    return false;
}

} // namespace ctbot
