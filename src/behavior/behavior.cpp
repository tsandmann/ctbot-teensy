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
 * @file    behavior.cpp
 * @brief   Abstraction for behaviors
 * @author  Timo Sandmann
 * @date    26.05.2019
 */

#include "behavior.h"
#include "scheduler.h"
#include "pose.h"
#include "speed.h"
#include "timer.h"

#include "pprintpp.hpp"
#include "arduino_freertos.h"

#include <thread>
#include <chrono>


namespace ctbot {

std::set<Behavior*> Behavior::active_;
std::mutex Behavior::active_mutex_;

Behavior::Behavior(const std::string& name, const bool uses_motor, const uint16_t priority, const uint16_t cycle_time_ms, const uint32_t stack_size)
    : name_ { name }, uses_motor_ { uses_motor }, finished_ {}, p_ctbot_ { &CtBotBehavior::get_instance() },
      p_sensors_ { get_ctbot()->get_sensors() }, p_left_ {}, p_right_ {}, p_pose_ {}, p_speed_ {}, abort_request_ {} {
    configASSERT(p_sensors_);

    /* initialize actuator and data pointers */
    init_actuator(PSTR("speed.left"), p_left_); // PID controlled speed for left wheel
    configASSERT(p_left_);
    init_actuator(PSTR("speed.right"), p_right_); // PID controlled speed for right wheel
    configASSERT(p_right_);

    init_data_ptr(PSTR("model.pose_enc"), p_pose_); // current pose based on wheel encoder data
    configASSERT(p_pose_);
    init_data_ptr(PSTR("model.speed_enc"), p_speed_); // current speed based on wheel encoder data
    configASSERT(p_speed_);

    set_active(true);

    task_id_ = get_ctbot()->get_scheduler()->task_add(name, cycle_time_ms, priority, stack_size, [this]() {
        if (!finished()) {
            run();
        }
    });
}

Behavior::Behavior(const std::string& name, const bool uses_motor, const uint16_t priority, const uint16_t cycle_time_ms)
    : Behavior { name, uses_motor, priority, cycle_time_ms, DEFAULT_STACK_SIZE } {}

Behavior::Behavior(const std::string& name, const bool uses_motor) : Behavior { name, uses_motor, DEFAULT_PRIORITY, DEFAULT_CYCLE_TIME, DEFAULT_STACK_SIZE } {}

Behavior::~Behavior() {
    debug_printf<DEBUG_>(PP_ARGS("Behavior::~Behavior() for \"{s}\": removing task {#x}...\r\n", get_name().c_str(), task_id_));

    if (get_ctbot()->get_scheduler()->task_remove(task_id_)) {
        debug_printf<DEBUG_>(PP_ARGS("Behavior::~Behavior() for \"{s}\": task removed.\r\n", get_name().c_str()));
    } else {
        debug_printf<DEBUG_>(PP_ARGS("Behavior::~Behavior() for \"{s}\": remove task failed.\r\n", get_name().c_str()));
    }
    debug_flush<DEBUG_>();
}

uint16_t Behavior::get_priority() const {
    return get_ctbot()->get_scheduler()->task_get(task_id_)->get_priority();
}

void Behavior::wait() {
    debug_printf<DEBUG_>(PP_ARGS("Behavior::wait(): waiting for exit of \"{s}\"...\r\n", get_name().c_str()));

    while (!finished_) {
        {
            std::unique_lock<std::mutex> lk(caller_mutex_);
            caller_cv_.wait(lk);
        }
        if (!finished_) {
            debug_printf<DEBUG_>(PP_ARGS("Behavior::wait(): \"{s}\" woke up, but not finished. Probably a BUG.\r\n", get_name().c_str()));

            using namespace std::chrono_literals;
            std::this_thread::sleep_for(100ms);
        }
    }
    debug_printf<DEBUG_>(PP_ARGS("Behavior::wait(): \"{s}\" finished.\r\n", get_name().c_str()));
}

void Behavior::wait_for_model_update() {
    get_ctbot()->wait_for_model_update(abort_request_);
}

size_t Behavior::get_motor_requests() {
    size_t n {};
    {
        std::lock_guard<std::mutex> lock { active_mutex_ };
        for (auto e : active_) {
            if (e->uses_motor_) {
                ++n;
            }
        }
    }
    return n;
}

void Behavior::motor_update_done() {
    get_ctbot()->motor_update_done();
}

void Behavior::set_active(const bool active) {
    std::lock_guard<std::mutex> lock { active_mutex_ };
    if (active) {
        active_.insert(this);
    } else {
        active_.erase(this);
    }
}

void Behavior::sleep_for_ms(const uint32_t time) {
    const auto until { Timer::get_ms() + time };
    while (Timer::get_ms() < until) {
        wait_for_model_update();
        if (uses_motor_) {
            motor_update_done();
        }
    }
}

bool Behavior::exit() {
    if (!finished_) {
        finished_ = true;
        set_active(false);

        debug_printf<DEBUG_>(PP_ARGS("Behavior::exit() for \"{s}\": notifying caller...\r\n", get_name().c_str()));

        caller_cv_.notify_all();
        return true;
    }

    debug_printf<DEBUG_>(PP_ARGS("Behavior::exit() for \"{s}\": called, but already finished!\r\n", get_name().c_str()));
    return false;
}

void Behavior::print_pose(const bool moving) const {
    if (!moving || get_speed()->get_left() || get_speed()->get_right()) {
        get_pose()->print(*get_ctbot()->get_comm());
        get_ctbot()->get_comm()->debug_print('\t', false);
        get_speed()->print(*get_ctbot()->get_comm());
        get_ctbot()->get_comm()->debug_print(PSTR("\r\n"), false);
    }
}

} // namespace ctbot
