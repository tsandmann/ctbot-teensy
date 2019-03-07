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
 * @file    parameter_storage.cpp
 * @brief   Parameter storage abstraction layer
 * @author  Timo Sandmann
 * @date    27.12.2018
 */

#include "parameter_storage.h"

#include "SD.h"


namespace ctbot {
ParameterStorage::ParameterStorage(const std::string& config_file, const size_t buffer_size) : config_file_ { config_file }, p_parameter_doc_ {} {
    if (!SD.exists(config_file_.c_str())) {
        File f { SD.open(config_file_.c_str(), O_WRITE | O_CREAT) };
        if (f) {
            f.close();
        } else {
            // Serial.println("PS::ParameterStorage(): file create failed.");
        }
    }
    File f { SD.open(config_file_.c_str(), O_READ) };
    if (f) {
        const size_t n { f.size() };
        p_parameter_doc_ = new DynamicJsonDocument(n + buffer_size);
        if (n) {
            auto error { deserializeJson(*p_parameter_doc_, f) };
            if (error) {
                Serial.print("ParameterStorage::ParameterStorage(): deserializeJson() failed.");
            } else {
                // Serial.print("PS::ParameterStorage(): parameter_=\"");
                // Serial.print(dump()->c_str());
                // Serial.println("\"");
            }
        }
        f.close();
    } else {
        Serial.println("ParameterStorage::ParameterStorage(): file open failed.");
    }
}

ParameterStorage::~ParameterStorage() {
    flush();
    delete p_parameter_doc_;
}

bool ParameterStorage::get_parameter(const std::string& key, uint32_t& value) const noexcept {
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

bool ParameterStorage::get_parameter(const std::string& key, int32_t& value) const noexcept {
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

bool ParameterStorage::get_parameter(const std::string& key, float& value) const noexcept {
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

bool ParameterStorage::get_parameter(const std::string& key, const size_t index, uint32_t& value) const noexcept {
    const auto array { p_parameter_doc_->getMember(key) };

    if (array.isNull()) {
        // Serial.println("PS::get_parameter(): key not found.");
        return false;
    }

    const auto data { array.getElement(index) };
    if (data.is<uint32_t>()) {
        value = data.as<uint32_t>();
        // Serial.print("PS::get_parameter(): key found, val=");
        // Serial.println(value, 10);
        return true;
    }
    // Serial.println("PS::get_parameter(): key found, index or type invalid.");
    return false;
}

bool ParameterStorage::get_parameter(const std::string& key, const size_t index, int32_t& value) const noexcept {
    const auto array { p_parameter_doc_->getMember(key) };

    if (array.isNull()) {
        // Serial.println("PS::get_parameter(): key not found.");
        return false;
    }

    const auto data { array.getElement(index) };
    if (data.is<int32_t>()) {
        value = data.as<int32_t>();
        // Serial.print("PS::get_parameter(): key found, val=");
        // Serial.println(value, 10);
        return true;
    }
    // Serial.println("PS::get_parameter(): key found, index or type invalid.");
    return false;
}

bool ParameterStorage::get_parameter(const std::string& key, const size_t index, float& value) const noexcept {
    const auto array { p_parameter_doc_->getMember(key) };

    if (array.isNull()) {
        // Serial.println("PS::get_parameter(): key not found.");
        return false;
    }

    const auto data { array.getElement(index) };
    if (data.is<float>()) {
        value = data.as<float>();
        // Serial.print("PS::get_parameter(): key found, val=");
        // Serial.println(value, 10);
        return true;
    }
    // Serial.println("PS::get_parameter(): key found, index or type invalid.");
    return false;
}

void ParameterStorage::set_parameter(const std::string& key, const uint32_t value) noexcept {
    (*p_parameter_doc_)[key] = value;
}

void ParameterStorage::set_parameter(const std::string& key, const int32_t value) noexcept {
    (*p_parameter_doc_)[key] = value;
}

void ParameterStorage::set_parameter(const std::string& key, const float value) noexcept {
    (*p_parameter_doc_)[key] = value;
}

void ParameterStorage::set_parameter(const std::string& key, const size_t index, const uint32_t value) noexcept {
    const auto array { p_parameter_doc_->getMember(key) };

    if (array.isNull()) {
        p_parameter_doc_->createNestedArray(key);
        // Serial.println("PS::set_parameter(): array created.");
    }
    array[index] = value;
}

void ParameterStorage::set_parameter(const std::string& key, const size_t index, const int32_t value) noexcept {
    const auto array { p_parameter_doc_->getMember(key) };

    if (array.isNull()) {
        p_parameter_doc_->createNestedArray(key);
        // Serial.println("PS::set_parameter(): array created.");
    }
    array[index] = value;
}

void ParameterStorage::set_parameter(const std::string& key, const size_t index, const float value) noexcept {
    const auto array { p_parameter_doc_->getMember(key) };

    if (array.isNull()) {
        p_parameter_doc_->createNestedArray(key);
        // Serial.println("PS::set_parameter(): array created.");
    }
    array[index] = value;
}

std::unique_ptr<std::string> ParameterStorage::dump() const {
    auto str { std::make_unique<std::string>() };
    serializeJson(*p_parameter_doc_, *str);
    return str;
}

bool ParameterStorage::flush() const {
    SD.remove(config_file_.c_str());
    File f { SD.open(config_file_.c_str(), O_WRITE | O_CREAT) };
    if (f) {
        serializeJson(*p_parameter_doc_, f);
        f.close();
    } else {
        // Serial.println("PS::flush(): file create failed.");
        return false;
    }
    return true;
}
} // namespace ctbot
