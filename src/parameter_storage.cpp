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
 * @file    parameter_storage.cpp
 * @brief   Parameter storage abstraction layer
 * @author  Timo Sandmann
 * @date    27.12.2018
 */

#include "parameter_storage.h"

#include "ctbot.h"
#include "fs_service.h"


namespace ctbot {
ParameterStorage::ParameterStorage(FS_Service& fs_svc, const std::string_view& config_file, size_t buffer_size)
    : fs_svc_ { fs_svc }, config_file_ { config_file }, p_parameter_doc_ {} {
    if (!fs_svc_.exists(config_file_.c_str())) {
        auto f { fs_svc_.open(config_file_.c_str(), static_cast<uint8_t>(FILE_WRITE)) };
        if (f) {
            f.close();
        } else {
            CtBot::get_instance().get_comm()->debug_print(PSTR("PS::ParameterStorage(): file create failed.\r\n"), true);
            return;
        }
    }
    auto f { fs_svc_.open(config_file_.c_str(), FILE_READ) };
    if (f) {
        const size_t n { static_cast<size_t>(f.size()) };
        p_parameter_doc_ = new DynamicJsonDocument { n + buffer_size };
        if (n) {
            auto error { deserializeJson(*p_parameter_doc_, f) };
            if (error) {
                CtBot::get_instance().get_comm()->debug_print(PSTR("PS::ParameterStorage(): deserializeJson() failed.\r\n"), true);
            } else if constexpr (DEBUG_) {
                CtBot::get_instance().get_comm()->debug_printf<true>(PSTR("PS::ParameterStorage(): parameter_=\"%s\"\r\n"), dump()->c_str());
            }
        }
        f.close();
    } else {
        CtBot::get_instance().get_comm()->debug_print(PSTR("ParameterStorage::ParameterStorage(): file open failed.\r\n"), true);
    }
}

ParameterStorage::~ParameterStorage() {
    flush();
    delete p_parameter_doc_;
}

template <typename T>
std::expected<T, ParameterStorage::Error> ParameterStorage::get(const std::string_view& key) const noexcept {
    static_assert(std::is_integral_v<T> || std::is_floating_point_v<T>, "invalid type");

    const auto data { p_parameter_doc_->getMember(key) };

    if (data.isNull()) [[unlikely]] {
        return std::unexpected { Error::NOT_AVAILABLE };
    }
    if (!data.is<T>()) [[unlikely]] {
        return std::unexpected { Error::INVALID_TYPE };
    }

    return data.as<T>();
}

template std::expected<uint32_t, ParameterStorage::Error> ParameterStorage::get(const std::string_view& key) const noexcept;
template std::expected<int32_t, ParameterStorage::Error> ParameterStorage::get(const std::string_view& key) const noexcept;
template std::expected<float, ParameterStorage::Error> ParameterStorage::get(const std::string_view& key) const noexcept;

template <typename T>
std::expected<T, ParameterStorage::Error> ParameterStorage::get(const std::string_view& key, size_t index) const noexcept {
    static_assert(std::is_integral_v<T> || std::is_floating_point_v<T>, "invalid type");

    const auto array { p_parameter_doc_->getMember(key) };

    if (array.isNull()) [[unlikely]] {
        if constexpr (DEBUG_) {
            CtBot::get_instance().get_comm()->debug_printf<true>(PSTR("PS::get_parameter(): key \"%s\" not found.\r\n"), key.data());
        }
        return std::unexpected { Error::NOT_AVAILABLE };
    }

    const auto data { array.getElement(index) };
    if (data.is<T>()) [[likely]] {
        if constexpr (DEBUG_) {
            CtBot::get_instance().get_comm()->debug_printf<true>(PSTR("PS::get_parameter(): key \"%s\" found, val=%u\r\n"), key.data(), data.as<T>());
        }
        return data.as<T>();
    }
    if constexpr (DEBUG_) {
        CtBot::get_instance().get_comm()->debug_printf<true>(PSTR("PS::get_parameter(): key \"%s\" found, index or type invalid.\r\n"), key.data());
    }
    return std::unexpected { Error::INVALID_TYPE };
}

template std::expected<uint32_t, ParameterStorage::Error> ParameterStorage::get(const std::string_view& key, size_t index) const noexcept;
template std::expected<int32_t, ParameterStorage::Error> ParameterStorage::get(const std::string_view& key, size_t index) const noexcept;
template std::expected<float, ParameterStorage::Error> ParameterStorage::get(const std::string_view& key, size_t index) const noexcept;

template <typename T>
void ParameterStorage::set(const std::string_view& key, T value) noexcept {
    static_assert(std::is_integral_v<T> || std::is_floating_point_v<T>, "invalid type");

    (*p_parameter_doc_)[key] = value;
}

template void ParameterStorage::set(const std::string_view& key, uint32_t value) noexcept;
template void ParameterStorage::set(const std::string_view& key, int32_t value) noexcept;
template void ParameterStorage::set(const std::string_view& key, float value) noexcept;

template <typename T>
void ParameterStorage::set(const std::string_view& key, size_t index, T value) noexcept {
    static_assert(std::is_integral_v<T> || std::is_floating_point_v<T>, "invalid type");

    const auto array { p_parameter_doc_->getMember(key) };

    if (array.isNull()) [[unlikely]] {
        p_parameter_doc_->createNestedArray(key);
        if constexpr (DEBUG_) {
            CtBot::get_instance().get_comm()->debug_print(PSTR("PS::set_parameter(): array created.\r\n"), true);
        }
    }
    // TODO: add zero-padding?
    array[index] = value;
}

template void ParameterStorage::set(const std::string_view& key, size_t index, uint32_t value) noexcept;
template void ParameterStorage::set(const std::string_view& key, size_t index, int32_t value) noexcept;
template void ParameterStorage::set(const std::string_view& key, size_t index, float value) noexcept;

std::unique_ptr<std::string> ParameterStorage::dump() const {
    auto str { std::make_unique<std::string>() };
    serializeJson(*p_parameter_doc_, *str);
    return str;
}

bool ParameterStorage::flush() const {
    fs_svc_.remove(config_file_.c_str());
    auto f { fs_svc_.open(config_file_.c_str(), static_cast<uint8_t>(FILE_WRITE)) };
    if (f) {
        serializeJson(*p_parameter_doc_, f);
        f.close();
    } else {
        CtBot::get_instance().get_comm()->debug_print(PSTR("PS::flush(): file create failed.\r\n"), true);
        return false;
    }
    return true;
}
} // namespace ctbot
