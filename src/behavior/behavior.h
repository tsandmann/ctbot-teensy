/*
 * This file is part of the ct-Bot teensy framework.
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
#include "logger.h"

#include "pprintpp.hpp"
#include "avr/pgmspace.h"

#include <cstdint>
#include <string_view>
#include <memory>
#include <mutex>
#include <atomic>
#include <chrono>
#include <thread>
#include <condition_variable>
#include <functional>
#include <set>


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

    static std::set<Behavior*> active_;
    static std::mutex active_mutex_;

    uint16_t task_id_;
    const std::string name_;
    const bool uses_motor_;
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
        Registry(const std::string_view& name, auto&&... args) {
            if (CtBotConfig::BEHAVIOR_MODEL_AVAILABLE) {
                CtBotBehavior::get_instance().register_behavior(name, std::forward<decltype(args)>(args)...);
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

    void motor_update_done();

    void set_active(const bool active);

    template <typename Rep, typename Period>
    void sleep_for(const std::chrono::duration<Rep, Period>& time) {
        const auto until { std::chrono::high_resolution_clock::now() + time };
        while (std::chrono::high_resolution_clock::now() < until) {
            wait_for_model_update();
            if (uses_motor_) {
                motor_update_done();
            }
        }
    }

    void sleep_for_ms(const uint32_t time);

    template <bool ENABLED = true>
    FLASHMEM void debug_flush() const {
        if (ENABLED) {
            get_ctbot()->get_logger()->flush();
        }
    }

    template <bool ENABLED = true>
    FLASHMEM_T auto debug_print(logger::PrintfArg auto const... args) const {
        if (ENABLED) {
            get_ctbot()->get_logger()->begin(get_name());
            return get_ctbot()->get_logger()->log(args..., true);
        } else {
            return static_cast<decltype(get_ctbot()->get_logger()->log(args..., false))>(0);
        }
    }

    template <bool ENABLED = true>
    FLASHMEM_T auto debug_printf(logger::PrintfArg auto const... args) const {
        if (ENABLED) {
            get_ctbot()->get_logger()->begin(get_name());
            return get_ctbot()->get_logger()->log<true>(args...);
        } else {
            return static_cast<decltype(get_ctbot()->get_logger()->log<false>(args...))>(0);
        }
    }

    FLASHMEM_T bool init_data(const std::string_view& name, auto*& p_res) const {
        return get_ctbot()->get_data()->get_resource(name, p_res);
    }

    template <typename T>
    FLASHMEM_T bool init_data_ptr(const std::string_view& name, T*& ptr) const {
        ptr = get_ctbot()->get_data()->get_res_ptr<T>(name);
        return ptr != nullptr;
    }

    FLASHMEM_T bool init_actuator(const std::string_view& name, auto*& p_res) const {
        return get_ctbot()->get_actuators()->get_resource(name, p_res);
    }

    template <class T, typename U = int32_t>
    FLASHMEM_T static auto INIT(const int32_t p) {
        return static_cast<BasePtr>(behavior_factory<T>(static_cast<U>(p)));
    }

    template <class T, typename U1 = int32_t, typename U2 = int32_t>
    FLASHMEM_T static auto INIT(const int32_t p1, const int32_t p2) {
        return static_cast<BasePtr>(behavior_factory<T>(static_cast<U1>(p1), static_cast<U2>(p2)));
    }

    template <class T, typename U1 = int32_t, typename U2 = int32_t, typename U3 = int32_t>
    FLASHMEM_T static auto INIT(const int32_t p1, const int32_t p2, const int32_t p3) {
        return static_cast<BasePtr>(behavior_factory<T>(static_cast<U1>(p1), static_cast<U2>(p2), static_cast<U3>(p3)));
    }

    template <class T, typename U1 = int32_t, typename U2 = int32_t, typename U3 = int32_t, typename U4 = int32_t>
    FLASHMEM_T static auto INIT(const int32_t p1, const int32_t p2, const int32_t p3, const int32_t p4) {
        return static_cast<BasePtr>(behavior_factory<T>(static_cast<U1>(p1), static_cast<U2>(p2), static_cast<U3>(p3), static_cast<U4>(p4)));
    }

    template <class T>
    FLASHMEM_T static auto INIT(auto&&... args) {
        return static_cast<BasePtr>(behavior_factory<T>(std::forward<decltype(args)>(args)...));
    }

public:
    using BasePtr = std::unique_ptr<Behavior>;

    static size_t get_motor_requests();

    FLASHMEM Behavior(const std::string& name, const bool uses_motor, const uint16_t priority, const uint16_t cycle_time_ms, const uint32_t stack_size);

    FLASHMEM Behavior(const std::string& name, const bool uses_motor, const uint16_t priority, const uint16_t cycle_time_ms);

    FLASHMEM Behavior(const std::string& name, const bool uses_motor);

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

    template <typename T, T min, T max>
    bool set_actuator(Actuator<T, min, max>* actuator, auto const& value) const {
        return actuator->add_value(static_cast<const T>(value), get_priority());
    }

    template <class T>
    static auto behavior_factory(auto&&... args) {
        return std::make_unique<T>(args...);
    }

    template <class T>
    static void behavior_factory(std::unique_ptr<T>& ptr, auto&&... args) {
        ptr = std::make_unique<T>(args...);
    }

    template <class T>
    void behavior_factory(std::unique_ptr<T>& ptr, bool block, auto&&... args) {
        behavior_factory(ptr, args...);

        if (ptr && block) {
            ptr->wait();
        }
    }

    template <class T>
    auto behavior_factory(bool block, auto&&... args) {
        std::unique_ptr<T> ptr;
        behavior_factory(ptr, block, args...);
        return ptr;
    }

    template <class T>
    auto switch_to(auto&&... args) {
        std::unique_ptr<T> ptr;
        behavior_factory(ptr, true, args...);
        return ptr;
    }
};


template <class Beh>
class BehaviorRunUntil : public Behavior {
    static constexpr bool DEBUG_ { true };

public:
    BehaviorRunUntil(std::function<bool()> check_func, auto&&... args)
        : Behavior(PSTR("BehaviorRunUntil")), p_beh_ { behavior_factory<Beh>(false, args...) }, func_ { check_func } {}


    FLASHMEM virtual ~BehaviorRunUntil() override {
        debug_print<DEBUG_>(PSTR("BehaviorRunUntil::~BehaviorRunUntil()\r\n"));

        abort_beh();
        wait();
    }

protected:
    std::unique_ptr<Beh> p_beh_;
    std::function<bool()> func_;

    virtual void run() override {
        debug_print<DEBUG_>(PSTR("BehaviorRunUntil::run().\r\n"));

        do {
            using namespace std::chrono_literals;
            std::this_thread::sleep_for(10ms);
        } while (!p_beh_->finished() && !func_() && !abort_request_);

        if (!p_beh_->finished()) {
            p_beh_->abort_beh();

            debug_printf<DEBUG_>(PP_ARGS("BehaviorRunUntil::run(): behavior \"{s}\" aborted.\r\n", p_beh_->get_name().c_str()));
        } else {
            debug_printf<DEBUG_>(PP_ARGS("BehaviorRunUntil::run(): behavior \"{s}\" finished.\r\n", p_beh_->get_name().c_str()));
        }

        exit();
    }
};

} // namespace ctbot
