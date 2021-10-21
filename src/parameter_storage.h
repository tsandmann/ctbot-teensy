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
 * @file    parameter_storage.h
 * @brief   Parameter storage abstraction layer
 * @author  Timo Sandmann
 * @date    27.12.2018
 */

#pragma once

#define ARDUINOJSON_ENABLE_STRING_VIEW 1
#define ARDUINOJSON_ENABLE_STD_STRING 1
#define ARDUINOJSON_ENABLE_STD_STREAM 0
#define ARDUINOJSON_ENABLE_ARDUINO_STRING 0
#define ARDUINOJSON_ENABLE_ARDUINO_STREAM 0
#define ARDUINOJSON_ENABLE_ARDUINO_PRINT 0
#include "arduino_freertos.h"
#include "ArduinoJson.h"

#include <string>
#include <string_view>
#include <memory>
#include <type_traits>
#include <concepts>


class FS;

namespace ctbot {

/**
 * @brief Parameter storage abstraction layer
 *
 * @startuml{ParameterStorage.png}
 *  !include parameter_storage.puml
 *  set namespaceSeparator ::
 *  skinparam classAttributeIconSize 0
 * @enduml
 */
class ParameterStorage { // FIXME: add documentation
    static constexpr bool DEBUG_ { false };

protected:
    FS& fs_;
    const std::string config_file_;
    DynamicJsonDocument* p_parameter_doc_;

    bool get_parameter(const std::string_view& key, uint32_t& value) const noexcept;
    bool get_parameter(const std::string_view& key, int32_t& value) const noexcept;
    bool get_parameter(const std::string_view& key, float& value) const noexcept;
    bool get_parameter(const std::string_view& key, const size_t index, uint32_t& value) const noexcept;
    bool get_parameter(const std::string_view& key, const size_t index, int32_t& value) const noexcept;
    bool get_parameter(const std::string_view& key, const size_t index, float& value) const noexcept;

    void set_parameter(const std::string_view& key, const uint32_t value) noexcept;
    void set_parameter(const std::string_view& key, const int32_t value) noexcept;
    void set_parameter(const std::string_view& key, const float value) noexcept;
    void set_parameter(const std::string_view& key, const size_t index, const uint32_t value) noexcept;
    void set_parameter(const std::string_view& key, const size_t index, const int32_t value) noexcept;
    void set_parameter(const std::string_view& key, const size_t index, const float value) noexcept;

public:
    FLASHMEM ParameterStorage(FS& fs, const std::string_view& config_file, const size_t buffer_size = 512);
    FLASHMEM ~ParameterStorage();

    std::unique_ptr<std::string> dump() const;

    bool flush() const;

    bool get(const std::string_view& key, std::unsigned_integral auto& value) const noexcept {
        uint32_t v;
        const bool res { get_parameter(key, v) };
        if (res) {
            value = static_cast<std::remove_reference<decltype(value)>::type>(v);
        }
        return res;
    }

    bool get(const std::string_view& key, std::signed_integral auto& value) const noexcept {
        int32_t v;
        const bool res { get_parameter(key, v) };
        if (res) {
            value = static_cast<std::remove_reference<decltype(value)>::type>(v);
        }
        return res;
    }

    bool get(const std::string_view& key, std::floating_point auto& value) const noexcept {
        float v;
        const bool res { get_parameter(key, v) };
        if (res) {
            value = static_cast<float>(v);
        }
        return res;
    }

    void set(const std::string_view& key, std::unsigned_integral auto const value) noexcept {
        set_parameter(key, static_cast<uint32_t>(value));
    }

    void set(const std::string_view& key, std::signed_integral auto const value) noexcept {
        set_parameter(key, static_cast<int32_t>(value));
    }

    void set(const std::string_view& key, std::floating_point auto const value) noexcept {
        set_parameter(key, static_cast<float>(value));
    }
};
} // namespace ctbot
