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
ParameterStorage::ParameterStorage(FS_Service& fs_svc, const std::string_view& config_file, const size_t buffer_size)
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

bool ParameterStorage::get_parameter(const std::string_view& key, uint32_t& value) const noexcept {
    const auto data { p_parameter_doc_->getMember(key) };

    if (data.isNull()) {
        return false;
    }
    if (!data.is<uint32_t>()) {
        return false;
    }

    value = data.as<uint32_t>();
    return true;
}

bool ParameterStorage::get_parameter(const std::string_view& key, int32_t& value) const noexcept {
    const auto data { p_parameter_doc_->getMember(key) };

    if (data.isNull()) {
        return false;
    }
    if (!data.is<int32_t>()) {
        return false;
    }

    value = data.as<int32_t>();
    return true;
}

bool ParameterStorage::get_parameter(const std::string_view& key, float& value) const noexcept {
    const auto data { p_parameter_doc_->getMember(key) };

    if (data.isNull()) {
        return false;
    }
    if (!data.is<float>()) {
        return false;
    }

    value = data.as<float>();
    return true;
}

bool ParameterStorage::get_parameter(const std::string_view& key, const size_t index, uint32_t& value) const noexcept {
    const auto array { p_parameter_doc_->getMember(key) };

    if (array.isNull()) {
        if constexpr (DEBUG_) {
            CtBot::get_instance().get_comm()->debug_printf<true>(PSTR("PS::get_parameter(): key \"%s\" not found.\r\n"), key.data());
        }
        return false;
    }

    const auto data { array.getElement(index) };
    if (data.is<uint32_t>()) {
        value = data.as<uint32_t>();
        if constexpr (DEBUG_) {
            CtBot::get_instance().get_comm()->debug_printf<true>(PSTR("PS::get_parameter(): key \"%s\" found, val=%u\r\n"), key.data(), value);
        }
        return true;
    }
    if constexpr (DEBUG_) {
        CtBot::get_instance().get_comm()->debug_printf<true>(PSTR("PS::get_parameter(): key \"%s\" found, index or type invalid.\r\n"), key.data());
    }
    return false;
}

bool ParameterStorage::get_parameter(const std::string_view& key, const size_t index, int32_t& value) const noexcept {
    const auto array { p_parameter_doc_->getMember(key) };

    if (array.isNull()) {
        if constexpr (DEBUG_) {
            CtBot::get_instance().get_comm()->debug_printf<true>(PSTR("PS::get_parameter(): key \"%s\" not found.\r\n"), key.data());
        }
        return false;
    }

    const auto data { array.getElement(index) };
    if (data.is<int32_t>()) {
        value = data.as<int32_t>();
        if constexpr (DEBUG_) {
            CtBot::get_instance().get_comm()->debug_printf<true>(PSTR("PS::get_parameter(): key \"%s\" found, val=%d\r\n"), value);
        }
        return true;
    }
    if constexpr (DEBUG_) {
        CtBot::get_instance().get_comm()->debug_printf<true>(PSTR("PS::get_parameter(): key \"%s\" found, index or type invalid.\r\n"), key.data());
    }
    return false;
}

bool ParameterStorage::get_parameter(const std::string_view& key, const size_t index, float& value) const noexcept {
    const auto array { p_parameter_doc_->getMember(key) };

    if (array.isNull()) {
        if constexpr (DEBUG_) {
            CtBot::get_instance().get_comm()->debug_printf<true>(PSTR("PS::get_parameter(): key \"%s\" not found.\r\n"), key.data());
        }
        return false;
    }

    const auto data { array.getElement(index) };
    if (data.is<float>()) {
        value = data.as<float>();
        if constexpr (DEBUG_) {
            CtBot::get_instance().get_comm()->debug_printf<true>(PSTR("PS::get_parameter(): key \"%s\" found, val=%f\r\n"), value);
        }
        return true;
    }
    if constexpr (DEBUG_) {
        CtBot::get_instance().get_comm()->debug_printf<true>(PSTR("PS::get_parameter(): key \"%s\" found, index or type invalid.\r\n"), key.data());
    }
    return false;
}

void ParameterStorage::set_parameter(const std::string_view& key, const uint32_t value) noexcept {
    (*p_parameter_doc_)[key] = value;
}

void ParameterStorage::set_parameter(const std::string_view& key, const int32_t value) noexcept {
    (*p_parameter_doc_)[key] = value;
}

void ParameterStorage::set_parameter(const std::string_view& key, const float value) noexcept {
    (*p_parameter_doc_)[key] = value;
}

void ParameterStorage::set_parameter(const std::string_view& key, const size_t index, const uint32_t value) noexcept {
    const auto array { p_parameter_doc_->getMember(key) };

    if (array.isNull()) {
        p_parameter_doc_->createNestedArray(key);
        if constexpr (DEBUG_) {
            CtBot::get_instance().get_comm()->debug_print(PSTR("PS::set_parameter(): array created.\r\n"), true);
        }
    }
    // TODO: add zero-padding?
    array[index] = value;
}

void ParameterStorage::set_parameter(const std::string_view& key, const size_t index, const int32_t value) noexcept {
    const auto array { p_parameter_doc_->getMember(key) };

    if (array.isNull()) {
        p_parameter_doc_->createNestedArray(key);
        if constexpr (DEBUG_) {
            CtBot::get_instance().get_comm()->debug_print(PSTR("PS::set_parameter(): array created.\r\n"), true);
        }
    }
    // TODO: add zero-padding?
    array[index] = value;
}

void ParameterStorage::set_parameter(const std::string_view& key, const size_t index, const float value) noexcept {
    const auto array { p_parameter_doc_->getMember(key) };

    if (array.isNull()) {
        p_parameter_doc_->createNestedArray(key);
        if constexpr (DEBUG_) {
            CtBot::get_instance().get_comm()->debug_print(PSTR("PS::set_parameter(): array created.\r\n"), true);
        }
    }
    // TODO: add zero-padding?
    array[index] = value;
}

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
