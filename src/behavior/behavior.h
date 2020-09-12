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
 * @file    behavior.h
 * @brief   Abstraction for behaviors
 * @author  Timo Sandmann
 * @date    26.05.2019
 */

#pragma once

#include "ctbot_config.h"
#include "ctbot_behavior.h"
#include "actuator.h"

#include "pprintpp.hpp"

#include <cstdint>
#include <string_view>
#include <memory>
#include <mutex>
#include <atomic>
#include <chrono>
#include <thread>
#include <condition_variable>
#include <functional>


namespace ctbot {

class Pose;
class Speed;

/**
 * @brief Behavior abstraction
 *
 * @startuml{Behavior.png}
 *  !include behavior/behavior.puml
 *  set namespaceSeparator ::
 *  skinparam classAttributeIconSize 0
 * @enduml
 */
class Behavior {
    // FIXME: add documentation
    static constexpr bool DEBUG_ { true };

    uint16_t task_id_;
    const std::string name_;
    std::atomic<bool> finished_;
    CtBotBehavior* const p_ctbot_;
    std::mutex caller_mutex_;
    std::condition_variable caller_cv_;
    Sensors* const p_sensors_;
    AMotor *p_left_, *p_right_;
    Pose* p_pose_;
    Speed* p_speed_;

protected:
    struct Registry {
        template <typename... Args>
        Registry(const std::string_view& name, Args&&... args) {
            if (CtBotConfig::BEHAVIOR_MODEL_AVAILABLE) {
                CtBotBehavior::get_instance().register_behavior(name, std::forward<Args>(args)...);
            }
        }

        Registry(const std::string_view&) {}
    };

    static constexpr uint16_t DEFAULT_PRIORITY { 6 };
    static constexpr uint16_t DEFAULT_CYCLE_TIME { 10 }; // ms
    static constexpr uint32_t DEFAULT_STACK_SIZE { 1024 }; // byte

    std::atomic<bool> abort_request_;

    virtual void run() = 0;

    bool exit();

    FLASHMEM void print_pose(const bool moving = true) const;

    auto get_ctbot() const {
        return p_ctbot_;
    }

    auto get_sensors() const {
        return p_sensors_;
    }

    auto get_motor_l() const {
        return p_left_;
    }

    auto get_motor_r() const {
        return p_right_;
    }

    auto get_pose() const {
        return p_pose_;
    }

    auto get_speed() const {
        return p_speed_;
    }

    void wait_for_model_update();

    template <bool ENABLED = true>
    FLASHMEM void debug_flush() const {
        if (ENABLED) {
            get_ctbot()->get_comm()->flush();
        }
    }

    template <bool ENABLED = true, typename... Args>
    FLASHMEM auto debug_print(Args... args) const {
        if (ENABLED) {
            return get_ctbot()->get_comm()->debug_print(args..., false);
        } else {
            return static_cast<decltype(get_ctbot()->get_comm()->debug_print(args..., false))>(0);
        }
    }

    template <bool ENABLED = true, typename... Args>
    FLASHMEM auto debug_printf(const Args&... args) const {
        if (ENABLED) {
            return get_ctbot()->get_comm()->debug_printf<false>(args...);
        } else {
            return static_cast<decltype(get_ctbot()->get_comm()->debug_printf<false>(args...))>(0);
        }
    }

    template <typename T>
    FLASHMEM bool init_data(const std::string_view& name, T*& p_res) const {
        return get_ctbot()->get_data()->get_resource(name, p_res);
    }

    template <typename T>
    FLASHMEM bool init_data_ptr(const std::string_view& name, T*& ptr) const {
        ptr = get_ctbot()->get_data()->get_res_ptr<T>(name);
        return ptr != nullptr;
    }

    template <typename T>
    FLASHMEM bool init_actuator(const std::string_view& name, T*& p_res) const {
        return get_ctbot()->get_actuators()->get_resource(name, p_res);
    }

    template <class T, typename U = int32_t>
    FLASHMEM static auto INIT(const int32_t p) {
        return static_cast<BasePtr>(behavior_factory<T>(static_cast<U>(p)));
    }

    template <class T, typename U1 = int32_t, typename U2 = int32_t>
    FLASHMEM static auto INIT(const int32_t p1, const int32_t p2) {
        return static_cast<BasePtr>(behavior_factory<T>(static_cast<U1>(p1), static_cast<U2>(p2)));
    }

    template <class T, typename U1 = int32_t, typename U2 = int32_t, typename U3 = int32_t>
    FLASHMEM static auto INIT(const int32_t p1, const int32_t p2, const int32_t p3) {
        return static_cast<BasePtr>(behavior_factory<T>(static_cast<U1>(p1), static_cast<U2>(p2), static_cast<U3>(p3)));
    }

    template <class T, typename U1 = int32_t, typename U2 = int32_t, typename U3 = int32_t, typename U4 = int32_t>
    FLASHMEM static auto INIT(const int32_t p1, const int32_t p2, const int32_t p3, const int32_t p4) {
        return static_cast<BasePtr>(behavior_factory<T>(static_cast<U1>(p1), static_cast<U2>(p2), static_cast<U3>(p3), static_cast<U4>(p4)));
    }

    template <class T, typename... Args>
    FLASHMEM static auto INIT(Args&&... args) {
        return static_cast<BasePtr>(behavior_factory<T>(std::forward<Args>(args)...));
    }

public:
    using BasePtr = std::unique_ptr<Behavior>;

    Behavior(const std::string& name, const uint16_t priority, const uint16_t cycle_time_ms, const uint32_t stack_size);

    Behavior(const std::string& name, const uint16_t priority, const uint16_t cycle_time_ms);

    Behavior(const std::string& name);

    FLASHMEM virtual ~Behavior();

    uint16_t get_priority() const;

    void wait();

    const std::string& get_name() const {
        return name_;
    }

    bool finished() const {
        return finished_;
    }

    void abort_beh() {
        abort_request_ = true;
    }

    template <typename T, T min, T max, typename U>
    bool set_actuator(Actuator<T, min, max>* actuator, const U& value) const {
        return actuator->add_value(static_cast<const T>(value), get_priority());
    }

    template <class T, typename... Args>
    static auto behavior_factory(Args&&... args) {
        return std::make_unique<T>(std::forward<Args>(args)...);
    }

    template <class T, typename... Args>
    static void behavior_factory(std::unique_ptr<T>& ptr, Args&&... args) {
        ptr = std::make_unique<T>(std::forward<Args>(args)...);
    }

    template <class T, typename... Args>
    void behavior_factory(std::unique_ptr<T>& ptr, bool block, Args&&... args) {
        behavior_factory(ptr, std::forward<Args>(args)...);

        if (ptr && block) {
            ptr->wait();
        }
    }

    template <class T, typename... Args>
    auto behavior_factory(bool block, Args&&... args) {
        std::unique_ptr<T> ptr;
        behavior_factory(ptr, block, std::forward<Args>(args)...);
        return ptr;
    }

    template <class T, typename... Args>
    auto switch_to(Args&&... args) {
        std::unique_ptr<T> ptr;
        behavior_factory(ptr, true, std::forward<Args>(args)...);
        return ptr;
    }
};


template <class Beh>
class BehaviorRunUntil : public Behavior {
    static constexpr bool DEBUG_ { true };

public:
    template <typename... Args>
    BehaviorRunUntil(std::function<bool()> check_func, Args&&... args)
        : Behavior("BehaviorRunUntil"), p_beh_ { behavior_factory<Beh>(false, std::forward<Args>(args)...) }, func_ { check_func } {}


    FLASHMEM virtual ~BehaviorRunUntil() override {
        debug_printf<DEBUG_>("BehaviorRunUntil::~BehaviorRunUntil()\r\n");
        debug_flush<DEBUG_>();

        abort_beh();
        wait();
    }

protected:
    std::unique_ptr<Beh> p_beh_;
    std::function<bool()> func_;

    virtual void run() override {
        debug_print<DEBUG_>("BehaviorRunUntil::run().\r\n");
        debug_flush<DEBUG_>();

        do {
            using namespace std::chrono_literals;
            std::this_thread::sleep_for(10ms);
        } while (!p_beh_->finished() && !func_() && !abort_request_);

        if (!p_beh_->finished()) {
            p_beh_->abort_beh();

            debug_printf<DEBUG_>(PP_ARGS("BehaviorRunUntil::run(): behavior \"{s}\" aborted.\r\n", p_beh_->get_name().c_str()));
            debug_flush<DEBUG_>();
        } else {
            debug_printf<DEBUG_>(PP_ARGS("BehaviorRunUntil::run(): behavior \"{s}\" finished.\r\n", p_beh_->get_name().c_str()));
            debug_flush<DEBUG_>();
        }

        exit();
    }
};

} // namespace ctbot
