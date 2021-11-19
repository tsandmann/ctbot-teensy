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

#pragma once

#include <cstdint>
#include <cstdlib>
#include <map>
#include <string>
#include <string_view>
#include <charconv>
#include <functional>
#include <deque>
#include <concepts>


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
    using func_t = std::function<bool(const std::string_view&)>;
    static constexpr bool DEBUG_ { false };
    static constexpr size_t HISTORY_SIZE_ { 16 };

    bool echo_;
    std::map<std::string /*cmd*/, func_t /*function*/, std::less<>> commands_;
    std::deque<std::string> history_;

public:
    /**
     * @brief Construct a new CmdParser object
     */
    FLASHMEM CmdParser();

    /**
     * @brief Register a new command given as a string_view
     * @param[in] cmd: Command
     * @param[in] func: Functor representing the action to execute for the command (may be a lambda)
     */
    FLASHMEM void register_cmd(const std::string_view& cmd, func_t&& func);

    /**
     * @brief Register a new command given as a string_view
     * @param[in] cmd: Command
     * @param[in] cmd_short: Shortcut for command as a single character
     * @param[in] func: Functor representing the action to execute for the command (may be a lambda)
     */
    FLASHMEM void register_cmd(const std::string_view& cmd, const char cmd_short, func_t&& func);

    /**
     * @brief Parse the input data and execute the corresponding command, if registered
     * @param[in] in: Pointer to input data as string_view
     * @param[in] comm: Reference to CommInterface (for debugging output only)
     * @return true on success
     */
    FLASHMEM bool parse(const std::string_view& in, CommInterface& comm);

    FLASHMEM bool execute_cmd(const std::string_view& cmd, CommInterface& comm); // FIXME: add documentation

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

    const std::string_view get_history(const size_t num) const {
        if (num && num <= history_.size()) {
            return std::string_view { history_[num - 1] };
        }
        return std::string_view {};
    }

    /**
     * @brief Split a string into space seperated tokens and return the first as integer argument
     * @param[in] args: Reference to input string
     * @param[out] b: Reference to first output argument of type bool
     * @return Pointer to the character past the last character interpreted
     */
    static std::string_view split_args(const std::string_view& args, bool& b) {
        uint_fast8_t x1 {};
        b = false;
        auto l { args.find(' ') };
        if (l == args.npos) {
            return std::string_view {};
        }
        ++l;
        auto ptr { std::from_chars(args.cbegin() + l, args.cend(), x1) };
        b = x1;
        return args.substr(ptr.ptr - args.cbegin());
    }

    /**
     * @brief Split a string into space seperated tokens and return the first as integer argument
     * @param[in] args: Reference to input string as string_view
     * @param[out] x1: Reference to first output argument
     * @return string_view to the last character interpreted
     */
    static std::string_view split_args(const std::string_view& args, std::integral auto& x1) {
        x1 = {};
        auto l { args.find(' ') };
        if (l == args.npos) {
            return std::string_view {};
        }
        ++l;
        auto ptr { std::from_chars(args.cbegin() + l, args.cend(), x1) };
        return args.substr(ptr.ptr - args.cbegin());
    }

    /**
     * @brief Split a string into space seperated tokens and return them as integer arguments
     * @param[in] args: Reference to input string as string_view
     * @param[out] x1: Reference to first output argument
     * @param[out] xn: Parameter pack of references to next arguments
     * @return string_view to the last character interpreted
     */
    static std::string_view split_args(const std::string_view& args, std::integral auto& x1, std::integral auto&... xn) {
        auto next_args { split_args(args, x1) };
        return split_args(next_args, xn...);
    }
};

} // namespace ctbot
