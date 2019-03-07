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

#define ARDUINOJSON_ENABLE_STD_STRING 1
#include "ArduinoJson.h"
#include <string>
#include <memory>
#include <type_traits>


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
class ParameterStorage {
    // FIXME: add documentation
protected:
    const std::string config_file_;
    DynamicJsonDocument* p_parameter_doc_;

    bool get_parameter(const std::string& key, uint32_t& value) const noexcept;
    bool get_parameter(const std::string& key, int32_t& value) const noexcept;
    bool get_parameter(const std::string& key, float& value) const noexcept;
    bool get_parameter(const std::string& key, const size_t index, uint32_t& value) const noexcept;
    bool get_parameter(const std::string& key, const size_t index, int32_t& value) const noexcept;
    bool get_parameter(const std::string& key, const size_t index, float& value) const noexcept;

    void set_parameter(const std::string& key, const uint32_t value) noexcept;
    void set_parameter(const std::string& key, const int32_t value) noexcept;
    void set_parameter(const std::string& key, const float value) noexcept;
    void set_parameter(const std::string& key, const size_t index, const uint32_t value) noexcept;
    void set_parameter(const std::string& key, const size_t index, const int32_t value) noexcept;
    void set_parameter(const std::string& key, const size_t index, const float value) noexcept;

public:
    ParameterStorage(const std::string& config_file, const size_t buffer_size = 512);
    ~ParameterStorage();

    std::unique_ptr<std::string> dump() const;

    bool flush() const;

    template <typename T>
    typename std::enable_if<std::is_integral<T>::value && std::is_unsigned<T>::value, bool>::type get(const std::string& key, T& value) const noexcept {
        uint32_t v;
        const bool res { get_parameter(key, v) };
        if (res) {
            value = static_cast<T>(v);
        }
        return res;
    }

    template <typename T>
    typename std::enable_if<std::is_integral<T>::value && std::is_signed<T>::value, bool>::type get(const std::string& key, T& value) const noexcept {
        int32_t v;
        const bool res { get_parameter(key, v) };
        if (res) {
            value = static_cast<T>(v);
        }
        return res;
    }

    template <typename T>
    typename std::enable_if<std::is_floating_point<T>::value, bool>::type get(const std::string& key, T& value) const noexcept {
        float v;
        const bool res { get_parameter(key, v) };
        if (res) {
            value = static_cast<T>(v);
        }
        return res;
    }

    template <typename T>
    typename std::enable_if<std::is_fundamental<T>::value, bool>::type get(const std::string& key, const size_t index, T& value) const noexcept {
        return get_parameter(key, index, value);
    }

    template <typename T>
    typename std::enable_if<std::is_integral<T>::value && std::is_unsigned<T>::value, void>::type set(const std::string& key, const T value) noexcept {
        set_parameter(key, static_cast<uint32_t>(value));
    }

    template <typename T>
    typename std::enable_if<std::is_integral<T>::value && std::is_signed<T>::value, void>::type set(const std::string& key, const T value) noexcept {
        set_parameter(key, static_cast<int32_t>(value));
    }

    template <typename T>
    typename std::enable_if<std::is_floating_point<T>::value, void>::type set(const std::string& key, const T value) noexcept {
        set_parameter(key, static_cast<float>(value));
    }

    template <typename T>
    typename std::enable_if<std::is_fundamental<T>::value, void>::type set(const std::string& key, const size_t index, const T& value) noexcept {
        set_parameter(key, index, value);
    }
};
} // namespace ctbot
