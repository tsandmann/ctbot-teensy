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

#include "FreeRTOS.h"
#include "SD.h"


// FIXME: use debug output of CommInterface
namespace ctbot {
ParameterStorage::ParameterStorage(const std::string& config_file) : config_file_ { config_file } {
    // arduino::Serial.print("PS::PS(): stack free before: ");
    // arduino::Serial.println(Scheduler::get_free_stack());

    if (!SD.exists(config_file_.c_str())) {
        File f { SD.open(config_file_.c_str(), O_WRITE | O_CREAT) };
        if (f) {
            f.close();
        } else {
            // arduino::Serial.println("ParameterStorage::ParameterStorage(): file create failed.");
        }
    }
    File f { SD.open(config_file_.c_str(), O_READ) };
    if (f) {
        const size_t n { f.size() };
        if (n) {
            p_parameter_root_ = &json_buffer_.parseObject(f);
            if (!p_parameter_root_->success()) {
                arduino::Serial.print("ParameterStorage::ParameterStorage(): parseObject() failed.");
            } else {
                // arduino::Serial.print("ParameterStorage::ParameterStorage(): parameter_=\"");
                // arduino::Serial.print(dump()->c_str());
                // arduino::Serial.println("\"");
            }
        } else {
            p_parameter_root_ = &json_buffer_.createObject();
        }
        f.close();
    } else {
        arduino::Serial.println("ParameterStorage::ParameterStorage(): file open failed.");
    }

    // arduino::Serial.print("PS::PS(): stack free after: ");
    // arduino::Serial.println(Scheduler::get_free_stack());
}

ParameterStorage::~ParameterStorage() {
    flush();
}

bool ParameterStorage::get_parameter(const std::string& key, uint32_t& value) const noexcept {
    // arduino::Serial.print("PS::get(): stack free before: ");
    // arduino::Serial.println(Scheduler::get_free_stack());

    if (!p_parameter_root_->containsKey(key)) {
        return false;
    }
    if (!p_parameter_root_->is<uint32_t>(key)) {
        return false;
    }

    value = p_parameter_root_->get<uint32_t>(key);

    // arduino::Serial.print("PS::get(): stack free after: ");
    // arduino::Serial.println(Scheduler::get_free_stack());
    return true;
}

bool ParameterStorage::get_parameter(const std::string& key, int32_t& value) const noexcept {
    // arduino::Serial.print("PS::get(): stack free before: ");
    // arduino::Serial.println(Scheduler::get_free_stack());

    if (!p_parameter_root_->containsKey(key)) {
        return false;
    }
    if (!p_parameter_root_->is<int32_t>(key)) {
        return false;
    }

    value = p_parameter_root_->get<int32_t>(key);

    // arduino::Serial.print("PS::get(): stack free after: ");
    // arduino::Serial.println(Scheduler::get_free_stack());
    return true;
}

bool ParameterStorage::get_parameter(const std::string& key, float& value) const noexcept {
    // arduino::Serial.print("PS::get(): stack free before: ");
    // arduino::Serial.println(Scheduler::get_free_stack());

    if (!p_parameter_root_->containsKey(key)) {
        return false;
    }
    if (!p_parameter_root_->is<float>(key)) {
        return false;
    }

    value = p_parameter_root_->get<float>(key);

    // arduino::Serial.print("PS::get(): stack free after: ");
    // arduino::Serial.println(Scheduler::get_free_stack());
    return true;
}

bool ParameterStorage::get_parameter(const std::string& key, const size_t index, uint32_t& value) const noexcept {
    if (!p_parameter_root_->containsKey(key)) {
        // arduino::Serial.println("PS::get_parameter(): key not found.");
        return false;
    }

    const auto& array { p_parameter_root_->get<JsonArray>(key) };
    if (array.is<uint32_t>(index)) {
        value = array.get<uint32_t>(index);
        // arduino::Serial.print("PS::get_parameter(): key found, val=");
        // arduino::Serial.println(value, 10);
        return true;
    }
    // arduino::Serial.println("PS::get_parameter(): key found, index or type invalid.");
    return false;
}

bool ParameterStorage::get_parameter(const std::string& key, const size_t index, int32_t& value) const noexcept {
    if (!p_parameter_root_->containsKey(key)) {
        // arduino::Serial.println("PS::get_parameter(): key not found.");
        return false;
    }

    const auto& array { p_parameter_root_->get<JsonArray>(key) };
    if (array.is<int32_t>(index)) {
        value = array.get<int32_t>(index);
        // arduino::Serial.print("PS::get_parameter(): key found, val=");
        // arduino::Serial.println(value, 10);
        return true;
    }
    // arduino::Serial.println("PS::get_parameter(): key found, index or type invalid.");
    return false;
}

bool ParameterStorage::get_parameter(const std::string& key, const size_t index, float& value) const noexcept {
    if (!p_parameter_root_->containsKey(key)) {
        // arduino::Serial.println("PS::get_parameter(): key not found.");
        return false;
    }

    const auto& array { p_parameter_root_->get<JsonArray>(key) };
    if (array.is<float>(index)) {
        value = array.get<float>(index);
        // arduino::Serial.print("PS::get_parameter(): key found, val=");
        // arduino::Serial.println(value, 2);
        return true;
    }
    // arduino::Serial.println("PS::get_parameter(): key found, index or type invalid.");
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
        // arduino::Serial.println("PS::set_parameter(): array created.");
    } else {
        p_array = &p_parameter_root_->get<JsonArray>(key);
        // arduino::Serial.println("PS::set_parameter(): array found.");
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
        // arduino::Serial.println("PS::set_parameter(): array created.");
    } else {
        p_array = &p_parameter_root_->get<JsonArray>(key);
        // arduino::Serial.println("PS::set_parameter(): array found.");
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
        // arduino::Serial.println("PS::set_parameter(): array created.");
    } else {
        p_array = &p_parameter_root_->get<JsonArray>(key);
        // arduino::Serial.println("PS::set_parameter(): array found.");
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
    // arduino::Serial.print("PS::dump(): stack free before: ");
    // arduino::Serial.println(Scheduler::get_free_stack());

    arduino::String str; // FIXME: improve buffer?
    p_parameter_root_->printTo(str);
    auto ret { std::make_unique<std::string>(str.c_str()) };

    // arduino::Serial.print("PS::dump(): stack free after: ");
    // arduino::Serial.println(Scheduler::get_free_stack());

    return ret;
}

bool ParameterStorage::flush() const {
    // arduino::Serial.print("PS::flush(): stack free before 1: ");
    // arduino::Serial.println(Scheduler::get_free_stack());

    SD.remove(config_file_.c_str());
    File f { SD.open(config_file_.c_str(), O_WRITE | O_CREAT) };
    if (f) {
        // arduino::Serial.print("PS::flush(): stack free before 2: ");
        // arduino::Serial.println(Scheduler::get_free_stack());

        p_parameter_root_->printTo(f);
        f.close();
    } else {
        // arduino::Serial.println("ParameterStorage::flush(): file create failed.");
        return false;
    }
    // arduino::Serial.print("PS::flush(): stack free after: ");
    // arduino::Serial.println(Scheduler::get_free_stack());
    // arduino::Serial.println("ParameterStorage::flush(): done.");
    return true;
}
} // namespace ctbot
