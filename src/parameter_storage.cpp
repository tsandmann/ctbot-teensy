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
#include "scheduler.h"

#include "SD.h"


namespace ctbot {
ParameterStorage::ParameterStorage(const std::string& config_file) : config_file_ { config_file } {
    if (!SD.exists(config_file_.c_str())) {
        File f { SD.open(config_file_.c_str(), O_WRITE | O_CREAT) };
        if (f) {
            f.close();
        } else {
            // Serial.println("ParameterStorage::ParameterStorage(): file create failed.");
        }
    }
    File f { SD.open(config_file_.c_str(), O_READ) };
    if (f) {
        const size_t n { f.size() };
        if (n) {
            p_parameter_root_ = &json_buffer_.parseObject(f);
            if (!p_parameter_root_->success()) {
                Serial.print("ParameterStorage::ParameterStorage(): parseObject() failed.");
            } else {
                // Serial.print("ParameterStorage::ParameterStorage(): parameter_=\"");
                // Serial.print(dump()->c_str());
                // Serial.println("\"");
            }
        } else {
            p_parameter_root_ = &json_buffer_.createObject();
        }
        f.close();
    } else {
        Serial.println("ParameterStorage::ParameterStorage(): file open failed.");
    }
}

ParameterStorage::~ParameterStorage() {
    flush();
}

bool ParameterStorage::get_parameter(const std::string& key, uint32_t& value) const noexcept {
    if (!p_parameter_root_->containsKey(key)) {
        return false;
    }
    if (!p_parameter_root_->is<uint32_t>(key)) {
        return false;
    }

    value = p_parameter_root_->get<uint32_t>(key);
    return true;
}

bool ParameterStorage::get_parameter(const std::string& key, int32_t& value) const noexcept {
    if (!p_parameter_root_->containsKey(key)) {
        return false;
    }
    if (!p_parameter_root_->is<int32_t>(key)) {
        return false;
    }

    value = p_parameter_root_->get<int32_t>(key);
    return true;
}

bool ParameterStorage::get_parameter(const std::string& key, float& value) const noexcept {
    if (!p_parameter_root_->containsKey(key)) {
        return false;
    }
    if (!p_parameter_root_->is<float>(key)) {
        return false;
    }

    value = p_parameter_root_->get<float>(key);
    return true;
}

bool ParameterStorage::get_parameter(const std::string& key, const size_t index, uint32_t& value) const noexcept {
    if (!p_parameter_root_->containsKey(key)) {
        // Serial.println("PS::get_parameter(): key not found.");
        return false;
    }

    auto& array { p_parameter_root_->get<JsonArray>(key) };
    if (array.is<uint32_t>(index)) {
        value = array.get<uint32_t>(index);
        // Serial.print("PS::get_parameter(): key found, val=");
        // Serial.println(value, 10);
        return true;
    }
    // Serial.println("PS::get_parameter(): key found, index or type invalid.");
    return false;
}

bool ParameterStorage::get_parameter(const std::string& key, const size_t index, int32_t& value) const noexcept {
    if (!p_parameter_root_->containsKey(key)) {
        // Serial.println("PS::get_parameter(): key not found.");
        return false;
    }

    auto& array { p_parameter_root_->get<JsonArray>(key) };
    if (array.is<int32_t>(index)) {
        value = array.get<int32_t>(index);
        // Serial.print("PS::get_parameter(): key found, val=");
        // Serial.println(value, 10);
        return true;
    }
    // Serial.println("PS::get_parameter(): key found, index or type invalid.");
    return false;
}

bool ParameterStorage::get_parameter(const std::string& key, const size_t index, float& value) const noexcept {
    if (!p_parameter_root_->containsKey(key)) {
        // Serial.println("PS::get_parameter(): key not found.");
        return false;
    }

    auto& array { p_parameter_root_->get<JsonArray>(key) };
    if (array.is<float>(index)) {
        value = array.get<float>(index);
        // Serial.print("PS::get_parameter(): key found, val=");
        Serial.println(value, 2);
        return true;
    }
    // Serial.println("PS::get_parameter(): key found, index or type invalid.");
    return false;
}

void ParameterStorage::set_parameter(const std::string& key, const uint32_t value) noexcept {
    p_parameter_root_->set(key, value);
}

void ParameterStorage::set_parameter(const std::string& key, const int32_t value) noexcept {
    p_parameter_root_->set(key, value);
}

void ParameterStorage::set_parameter(const std::string& key, const float value) noexcept {
    p_parameter_root_->set(key, value);
}

void ParameterStorage::set_parameter(const std::string& key, const size_t index, const uint32_t value) noexcept {
    JsonArray* p_array;
    if (!p_parameter_root_->containsKey(key)) {
        p_array = &p_parameter_root_->createNestedArray(key);
        // Serial.println("PS::set_parameter(): array created.");
    } else {
        p_array = &p_parameter_root_->get<JsonArray>(key);
        // Serial.println("PS::set_parameter(): array found.");
    }

    for (size_t i { 0 }; i < index; ++i) {
        if (!p_array->is<uint32_t>(i)) {
            p_array->add(0U);
        }
    }

    if (!p_array->set(index, value)) {
        p_array->add(value);
    }
}

void ParameterStorage::set_parameter(const std::string& key, const size_t index, const int32_t value) noexcept {
    JsonArray* p_array;
    if (!p_parameter_root_->containsKey(key)) {
        p_array = &p_parameter_root_->createNestedArray(key);
        // Serial.println("PS::set_parameter(): array created.");
    } else {
        p_array = &p_parameter_root_->get<JsonArray>(key);
        // Serial.println("PS::set_parameter(): array found.");
    }

    for (size_t i { 0 }; i < index; ++i) {
        if (!p_array->is<int32_t>(i)) {
            p_array->add(0);
        }
    }

    if (!p_array->set(index, value)) {
        p_array->add(value);
    }
}

void ParameterStorage::set_parameter(const std::string& key, const size_t index, const float value) noexcept {
    JsonArray* p_array;
    if (!p_parameter_root_->containsKey(key)) {
        p_array = &p_parameter_root_->createNestedArray(key);
        // Serial.println("PS::set_parameter(): array created.");
    } else {
        p_array = &p_parameter_root_->get<JsonArray>(key);
        // Serial.println("PS::set_parameter(): array found.");
    }

    for (size_t i { 0 }; i < index; ++i) {
        if (!p_array->is<float>(i)) {
            p_array->add(0.f);
        }
    }

    if (!p_array->set(index, value)) {
        p_array->add(value);
    }
}

std::unique_ptr<std::string> ParameterStorage::dump() const {
    String str; // FIXME: improve buffer?
    p_parameter_root_->printTo(str);
    auto ret { std::make_unique<std::string>(str.c_str()) };
    return ret;
}

bool ParameterStorage::flush() const {
    SD.remove(config_file_.c_str());
    File f { SD.open(config_file_.c_str(), O_WRITE | O_CREAT) };
    if (f) {
        p_parameter_root_->printTo(f);
        f.close();
    } else {
        // Serial.println("ParameterStorage::flush(): file create failed.");
        return false;
    }
    // Serial.println("ParameterStorage::flush(): done.");
    return true;
}
} // namespace ctbot
