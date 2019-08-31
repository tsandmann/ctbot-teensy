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

#pragma once

#include <cstdint>
#include <cstdlib>
#include <map>
#include <string>
#include <string_view>
#include <functional>
#include <deque>


namespace ctbot {

class CommInterface;

/**
 * @brief Simple parser for console commands
 *
 * @startuml{CmdParser.png}
 *  !include cmd_parser.puml
 *  set namespaceSeparator ::
 *  skinparam classAttributeIconSize 0
 * @enduml
 */
class CmdParser {
protected:
    using func_t = std::function<bool(const std::string&)>;
    static constexpr bool DEBUG_ { true };
    static constexpr size_t HISTORY_SIZE_ { 16 };

    bool echo_;
    std::map<std::string /*cmd*/, func_t /*function*/, std::less<>> commands_;
    std::deque<std::string> history_;

public:
    /**
     * @brief Construct a new CmdParser object
     */
    CmdParser();

    /**
     * @brief Register a new command given as a string
     * @param[in] cmd: Pointer to of command
     * @param[in] func: Functor representing the action to execute for the command (may be a lambda)
     */
    void register_cmd(const std::string& cmd, const func_t& func);

    /**
     * @brief Register a new command given as a string
     * @param[in] cmd: Pointer to string of command
     * @param[in] cmd_short: Shortcut for command as a single character
     * @param[in] func: Functor representing the action to execute for the command (may be a lambda)
     */
    void register_cmd(const std::string& cmd, const char cmd_short, const func_t& func);

    /**
     * @brief Parse the input data and execute the corresponding command, if registered
     * @param[in] in: Pointer to input data as string_view
     * @param[in] comm: Reference to CommInterface (for debugging output only)
     * @return true on success
     */
    bool parse(const std::string_view& in, CommInterface& comm);

    bool execute_cmd(const std::string_view& cmd, CommInterface& comm); // FIXME: add documentation

    auto get_echo() const {
        return echo_;
    }

    /**
     * @brief Set the character echo mode for console
     * @param[in] value: true to activate character echo on console, false otherwise
     */
    void set_echo(bool value) {
        echo_ = value;
    }

    const std::string* get_history(const size_t num) const {
        if (num && num <= history_.size()) {
            return &history_[num - 1];
        }
        return nullptr;
    }

    /**
     * @brief Split a string into space seperated tokens and return the first as integer argument
     * @tparam T: Type of argument to get out
     * @param[in] args: Reference to input string
     * @param[out] x1: Reference to first output argument
     * @return Pointer to the character past the last character interpreted
     */
    template <typename T>
    static char* split_args(const std::string& args, T& x1) {
        const auto l { args.find(" ") + 1 };
        char* p_end;
        x1 = static_cast<T>(std::strtol(args.c_str() + l, &p_end, 0));
        return p_end;
    }

    /**
     * @brief Split a string into space seperated tokens and return them as integer arguments
     * @tparam T: Type of argument to get out
     * @param[in] args: Reference to input string
     * @param[out] x1: Reference to first output argument
     * @param[out] xn: Parameter pack of references to next arguments
     * @return Pointer to the character past the last character interpreted
     */
    template <typename T, typename... Args>
    static char* split_args(const std::string& args, T& x1, Args&... xn) {
        char* p_end { split_args(args, x1) };
        const std::string next_args { p_end };
        return split_args(next_args, xn...);
    }
};

} // namespace ctbot
