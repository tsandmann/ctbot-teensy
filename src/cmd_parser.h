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

#include "fs_service.h"

#include "avr/pgmspace.h"

#include <charconv>
#include <concepts>
#include <cstdint>
#include <cstdlib>
#include <deque>
#include <functional>
#include <map>
#include <string>
#include <string_view>
#include <tuple>
#include <optional>


class FS_Service;

namespace ctbot {

class CommInterface;

namespace detail {
template <typename T>
concept Number = std::integral<T> || std::floating_point<T>;

template <typename T>
concept NumberArg =
#if __GNUC__ >= 12
    Number<T>;
#else //__GNUC__ < 12
    std::integral<T>;
#endif //__GNUC__
}

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
    std::map<const std::string_view /*cmd*/, func_t /*function*/, std::less<>> commands_;
    std::deque<std::string> history_;
    FS_Service* p_fs_svc_;
    File* p_file_;
    FS_Service::FileWrapper* p_file_wrapper_;

public:
    /**
     * @brief Construct a new CmdParser object
     */
    FLASHMEM CmdParser();

    FLASHMEM ~CmdParser();

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
    FLASHMEM void register_cmd(const std::string_view& cmd, const char* cmd_short, func_t&& func);

    /**
     * @brief Parse the input data and execute the corresponding command, if registered
     * @param[in] in: Pointer to input data as string_view
     * @param[in] comm: Reference to CommInterface (for debugging output only)
     * @return true on success
     */
    FLASHMEM bool parse(const std::string_view& in, CommInterface& comm);

    FLASHMEM bool execute_cmd(const std::string_view& cmd, CommInterface& comm); // TODO: add documentation

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

    FLASHMEM bool enable_nv_history(FS_Service& fs_svc, const std::string_view& filename);

    std::optional<std::string> get_history(size_t num);

    static std::string_view trim_to_first_arg(const std::string_view& str);

    /**
     * @brief Split a string into space seperated tokens and return the first as bool argument
     * @param[in] args: Reference to input string
     * @param[out] b: Reference to first output argument of type bool
     * @return Value of type std::from_chars_result such that ptr points at the first character not matching the pattern
     */
    static auto split_args(const std::string_view& args, bool trim, bool& b) {
        auto str { trim ? trim_to_first_arg(args) : args };
        uint8_t x1;
        auto res { std::from_chars(str.cbegin(), str.cend(), x1) };
        if (res.ec == std::errc {}) {
            b = static_cast<bool>(x1);
        }

        return res;
    }

    /**
     * @brief Split a string into space seperated tokens and return the first as number (integral or float)  argument
     * @param[in] args: Reference to input string as string_view
     * @param[out] x1: Reference to first output argument
     * @return Value of type std::from_chars_result such that ptr points at the first character not matching the pattern
     */
    static auto split_args(const std::string_view& args, bool trim, detail::NumberArg auto& x1) {
        auto str { trim ? trim_to_first_arg(args) : args };
        return std::from_chars(str.cbegin(), str.cend(), x1);
    }

#if __GNUC__ < 12
    /**
     * @brief Split a string into space seperated tokens and return the first as float argument
     * @param[in] args: Reference to input string as string_view
     * @param[out] x1: Reference to first output argument
     * @return Value of type std::from_chars_result such that ptr points at the first character not matching the pattern
     */
    static auto split_args(const std::string_view& args, bool trim, std::floating_point auto& x1) {
        auto str { trim ? trim_to_first_arg(args) : args };

        char* p_end;
        const auto tmp { std::strtof(str.cbegin(), &p_end) };
        std::from_chars_result res { nullptr, std::errc {} };
        if (p_end != str.cbegin()) {
            x1 = tmp;
            res.ptr = p_end;
        } else {
            res.ec = std::errc::invalid_argument;
        }

        return res;
    }
#endif // __GNUC__

    /**
     * @brief Split a string into space seperated tokens and return them as number (integral or float) arguments
     * @param[in] args: Reference to input string as string_view
     * @param[out] x1: Reference to first output argument
     * @param[out] xn: Parameter pack of references to next arguments
     * @return Value of type std::from_chars_result such that ptr points at the first character not matching the pattern
     */
    static auto split_args(const std::string_view& args, bool trim, detail::Number auto& x1, detail::Number auto&... xn) {
        auto res { split_args(args, trim, x1) };

        if (res.ec != std::errc {}) {
            return res;
        }

        return split_args(std::string_view(res.ptr, args.cend()), true, xn...);
    }

    template <typename... Ts>
    static auto split_args(const std::string_view& args, bool trim, std::tuple<Ts...>& x) {
        return std::apply([&args, trim](Ts&... values) { return split_args(args, trim, values...); }, x);
    }
};

} // namespace ctbot
