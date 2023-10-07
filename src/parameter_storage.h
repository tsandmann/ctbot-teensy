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

#include <concepts>
#include <memory>
#include <string>
#include <string_view>
#include <type_traits>
#include <expected>


class FS_Service;

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

public:
    enum class Error : uint8_t {
        SUCCESS = 0,
        NOT_AVAILABLE,
        INVALID_TYPE,
    };

protected:
    FS_Service& fs_svc_;
    const std::string config_file_;
    DynamicJsonDocument* p_parameter_doc_;

public:
    FLASHMEM ParameterStorage(FS_Service& fs_svc, const std::string_view& config_file, size_t buffer_size = 512);
    FLASHMEM ~ParameterStorage();

    std::unique_ptr<std::string> dump() const;

    bool flush() const;

    template <typename T>
    std::expected<T, Error> get(const std::string_view& key) const noexcept;

    template <typename T>
    std::expected<T, Error> get(const std::string_view& key, size_t index) const noexcept;

    template <typename T>
    void set(const std::string_view& key, T value) noexcept;

    template <typename T>
    void set(const std::string_view& key, size_t index, T value) noexcept;
};
} // namespace ctbot
