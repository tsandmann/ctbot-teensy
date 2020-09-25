/*
 * This file is part of the c't-Bot teensy framework.
 * Copyright (c) 2019 Timo Sandmann
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
 * @file    actuator.h
 * @brief   Abstraction for actuators
 * @author  Timo Sandmann
 * @date    26.05.2019
 */

#pragma once

#include "resource.h"
#include "resource_container.h"

#include <cstdint>
#include <limits>
#include <map>
#include <tuple>
#include <memory>
#include <algorithm>
#include <mutex>


namespace ctbot {

/**
 * @brief Actuator abstraction
 *
 * @tparam T Type to be used for the underlying resource
 * @tparam min Minimal useable value for the actuator
 * @tparam max Maximal useable value for the actuator
 *
 * @startuml{Actuator.png}
 *  !include behavior/actuator.puml
 *  set namespaceSeparator ::
 *  skinparam classAttributeIconSize 0
 * @enduml
 */
template <typename T, T min = std::numeric_limits<T>::min(), T max = std::numeric_limits<T>::max()>
class Actuator : public Resource<T> {
protected:
    std::map<uint16_t /*prio*/, std::tuple<T /*value*/, std::function<void(const Actuator&, bool)> /*callback*/>>
        value_map_; /**< map to store all values added to this actuator */
    mutable std::mutex mutex_;

public:
    using Ptr = std::unique_ptr<Actuator>;

    /**
     * @brief Add a new value to this actuator
     *
     * @param[in] x Value as constant reference to add
     * @param[in] prio Priority for the new value
     * @param[in] callback Callback functor called in commit_value(), to notify if this value is the selected one
     * @return true, if new value has highest priority so far
     */
    FLASHMEM bool add_value(const T& x, const uint16_t prio, std::function<void(const Actuator&, bool)> callback = nullptr) {
        std::unique_lock<std::mutex> lk { mutex_ };
        const bool res { value_map_.size() ? value_map_.rbegin()->first <= prio : true };
        T val { 0 };
        if (x) {
            val = std::min<T>(std::max<T>(min, static_cast<T>(std::abs(x))), max);
        }
        if (std::numeric_limits<T>::min() < 0 && x < 0) {
            val = static_cast<T>(-val);
        }
        value_map_[prio] = std::make_tuple(val, callback);
        return res;
    }

    /**
     * @brief Commits values added since last commit and calls any registered callback
     */
    FLASHMEM void commit_value() {
        std::unique_lock<std::mutex> lk { mutex_ };
        if (!value_map_.empty()) {
            this->write(std::get<0>(value_map_.rbegin()->second));
            if (std::get<1>(value_map_.rbegin()->second)) {
                std::get<1>(value_map_.rbegin()->second)(*this, true);
            }
            value_map_.erase(value_map_.rbegin()->first);
            std::for_each(value_map_.begin(), value_map_.end(), [this](typename decltype(value_map_)::value_type v) {
                if (std::get<1>(v.second)) {
                    std::get<1>(v.second)(*this, false);
                }
            });
            value_map_.clear();
        }
    }
};

/**
 * @brief Specialization for governors
 *
 * @tparam min Minimal useable value for the governor
 * @tparam max Maximal useable value for the governor
 */
template <int16_t min, int16_t max>
using Governor = Actuator<int16_t, min, max>;

/**
 * @brief Specialization for motors controlled by percentage values
 */
using AMotor = Governor<-100, 100>;

// /**
//  * @brief Specialization for servos
//  *
//  * @tparam min Minimal useable value for the servo
//  * @tparam max Maximal useable value for the servo
//  */
// template <uint8_t min, uint8_t max>
// using AServo = Actuator<uint8_t, min, max>; // FIXME: currently not used


/**
 * @brief Actuator aggregation
 *
 * @tparam T Type of aggregated actuators
 *
 * @startuml{ActuatorContainer.png}
 *  !include behavior/actuator.puml
 *  set namespaceSeparator ::
 *  skinparam classAttributeIconSize 0
 * @enduml
 */
template <class T>
class ActuatorContainer : public ResourceContainer {
public:
    using Ptr = std::unique_ptr<ActuatorContainer>;

    /**
     * @brief Create an actuator instance
     *
     * @tparam Args... Types of Arguments for constructor of actuator class
     * @param[in] name Name of actuator instance as C-String
     * @param[in] active Flag to indicated, if added resource should be active or inactive
     * @param args... Arguments for constructor of actuator class
     * @return Pointer to created instance or nullptr, if add_resource() failed
     */
    template <typename... Args>
    FLASHMEM T* create_actuator(const char* name, bool active, Args&&... args) {
        std::unique_ptr<T> p_res { std::make_unique<T>(std::forward<Args>(args)...) };
        T* ptr { p_res.get() };
        if (add_resource(name, std::move(p_res), active)) {
            return ptr;
        }
        return nullptr;
    }

    /**
     * @brief Commits all added values of all actuators in this container
     */
    FLASHMEM void commit_values() {
        for (auto& p_res : resources_) {
            T* p_act { static_cast<T*>(std::get<1>(p_res.second).get()) };
            if (p_act) {
                p_act->commit_value();
            }
        }
    }
};

} // namespace ctbot
