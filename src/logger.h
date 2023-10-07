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
 * @file    logger.h
 * @brief   Logger for ct-Bot teensy framework
 * @author  Timo Sandmann
 * @date    14.12.2021
 */

#pragma once

#include "fs_service.h"

#include "avr/pgmspace.h"

#include <concepts>
#include <cstdint>
#include <string>
#include <string_view>
#include <type_traits>
#include <vector>
#include <expected>


namespace ctbot {
class CtBot;

namespace logger {
template <typename T>
concept Number = std::integral<T> || std::floating_point<T>;

template <typename T>
concept PrintfArg = std::integral<T> || std::floating_point<T> || std::is_pointer<T>::value;
} // namespace logger


class LoggerTarget {
public:
    FLASHMEM LoggerTarget();
    FLASHMEM virtual ~LoggerTarget();

    FLASHMEM static int get_format_size(const char* format, ...) __attribute__((format(printf, 1, 2)));

    FLASHMEM static std::expected<std::string, int> create_formatted_string(size_t size, const char* format, ...);

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wformat-security"
    FLASHMEM static std::expected<std::string, int> string_format(const char* format, logger::PrintfArg auto... args) {
        const auto size { get_format_size(format, args...) + 1 };
        if (size > 0) [[likely]] {
            return create_formatted_string(size, format, args...);
        } else {
            return std::unexpected { size - 1 };
        }
    }
#pragma GCC diagnostic pop

    virtual void begin(const std::string_view& prefix) = 0;
    virtual size_t log(char c, bool block) = 0;
    virtual size_t log(const std::string_view& str, bool block) = 0;
    virtual void flush() = 0;

    template <bool BLOCK = false>
    FLASHMEM_T size_t log(const char* format, logger::PrintfArg auto... args) {
        return log(string_format(format, args...), BLOCK);
    }
};


class LoggerTargetFile : public LoggerTarget {
protected:
    File* p_file_;
    FS_Service::FileWrapper* p_file_wrapper_;
    CtBot* p_ctbot_;

public:
    FLASHMEM LoggerTargetFile(FS_Service& fs_svc, const std::string_view& filename, CtBot* p_ctbot);
    FLASHMEM virtual ~LoggerTargetFile() override;

    virtual void begin(const std::string_view& prefix) override;
    virtual size_t log(char c, bool block) override;
    virtual size_t log(const std::string_view& str, bool block) override;
    virtual void flush() override;
};


class Logger {
protected:
    std::vector<LoggerTarget*> targets_;

public:
    FLASHMEM Logger();

    FLASHMEM size_t add_target(LoggerTarget* p_target);

    void begin(const std::string_view& prefix = "") const;

    size_t log(char c, bool block);
    size_t log(const char* str, bool block);
    size_t log(const std::string* p_str, bool block);
    size_t log(const std::string& str, bool block);
    size_t log(std::string&& str, bool block);
    size_t log(const std::string_view& str, bool block);

    FLASHMEM size_t log(logger::Number auto v, bool block) {
        return log(std::to_string(v), block);
    }

    template <bool BLOCK = false>
    FLASHMEM_T size_t log(const char* format, logger::PrintfArg auto... args) {
        if (const auto str { LoggerTarget::string_format(format, args...) }; str.has_value()) {
            return log(*str, BLOCK);
        } else {
            return 0;
        }
    }

    void flush();
};
} // namespace ctbot
