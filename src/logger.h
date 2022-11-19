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

#include "avr/pgmspace.h"

#include <concepts>
#include <cstdint>
#include <string>
#include <string_view>
#include <type_traits>
#include <vector>


class FS_Service;
class File;

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

    FLASHMEM static size_t get_format_size(const char* format, ...) __attribute__((format(printf, 1, 2)));

    FLASHMEM static std::string create_formatted_string(const size_t size, const char* format, ...);

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wformat-security"
    FLASHMEM static std::string string_format(const char* format, logger::PrintfArg auto const... args) {
        const auto size { get_format_size(format, args...) + 1 };
        if (size > 0) {
            return create_formatted_string(size, format, args...);
        } else {
            return nullptr;
        }
    }
#pragma GCC diagnostic pop

    virtual void begin(const std::string_view& prefix) const = 0;
    virtual size_t log(const char c, const bool block) = 0;
    virtual size_t log(const std::string_view& str, const bool block) = 0;
    virtual void flush() = 0;

    template <bool BLOCK = false>
    FLASHMEM_T size_t log(const char* format, logger::PrintfArg auto const... args) {
        return log(string_format(format, args...), BLOCK);
    }
};


class LoggerTargetFile : public LoggerTarget {
protected:
    File* p_file_;
    CtBot* p_ctbot_;

public:
    FLASHMEM LoggerTargetFile(FS_Service& fs_svc, const std::string_view& filename, CtBot* p_ctbot);
    FLASHMEM virtual ~LoggerTargetFile() override;

    virtual void begin(const std::string_view& prefix) const override;
    virtual size_t log(const char c, const bool block) override;
    virtual size_t log(const std::string_view& str, const bool block) override;
    virtual void flush() override;
};


class Logger {
protected:
    std::vector<LoggerTarget*> targets_;

public:
    FLASHMEM Logger();

    FLASHMEM size_t add_target(LoggerTarget* p_target);

    void begin(const std::string_view& prefix = "") const;

    size_t log(const char c, const bool block);
    size_t log(const char* str, const bool block);
    size_t log(const std::string* p_str, const bool block);
    size_t log(const std::string& str, const bool block);
    size_t log(std::string&& str, const bool block);
    size_t log(const std::string_view& str, const bool block);

    FLASHMEM size_t log(logger::Number auto const v, const bool block) {
        return log(std::to_string(v), block);
    }

    template <bool BLOCK = false>
    FLASHMEM_T size_t log(const char* format, logger::PrintfArg auto const... args) {
        return log(LoggerTarget::string_format(format, args...), BLOCK);
    }

    void flush();
};
} // namespace ctbot
