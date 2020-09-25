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
 * @file    behavior.cpp
 * @brief   Abstraction for behaviors
 * @author  Timo Sandmann
 * @date    26.05.2019
 */

#include "behavior.h"
#include "scheduler.h"
#include "pose.h"
#include "speed.h"

#include "pprintpp.hpp"
#include "arduino_freertos.h"

#include <thread>
#include <chrono>


namespace ctbot {

Behavior::Behavior(const std::string& name, const uint16_t priority, const uint16_t cycle_time_ms, const uint32_t stack_size)
    : name_ { name }, finished_ {}, p_ctbot_ { &CtBotBehavior::get_instance() },
      p_sensors_ { get_ctbot()->get_sensors() }, p_left_ {}, p_right_ {}, p_pose_ {}, p_speed_ {}, abort_request_ {} {
    configASSERT(p_sensors_); // FIXME: wrapper for assert

    /* initialize actuator and data pointers */
    init_actuator(PSTR("speed.left"), p_left_); // PID controlled speed for left wheel
    configASSERT(p_left_);
    init_actuator(PSTR("speed.right"), p_right_); // PID controlled speed for right wheel
    configASSERT(p_right_);

    init_data_ptr(PSTR("model.pose_enc"), p_pose_); // current pose based on wheel encoder data
    configASSERT(p_pose_);
    init_data_ptr(PSTR("model.speed_enc"), p_speed_); // current speed based on wheel encoder data
    configASSERT(p_speed_);

    task_id_ = get_ctbot()->get_scheduler()->task_add(name, cycle_time_ms, priority, stack_size, [this]() {
        if (!finished()) {
            run();
        }
    });
}

Behavior::Behavior(const std::string& name, const uint16_t priority, const uint16_t cycle_time_ms)
    : Behavior { name, priority, cycle_time_ms, DEFAULT_STACK_SIZE } {}

Behavior::Behavior(const std::string& name) : Behavior { name, DEFAULT_PRIORITY, DEFAULT_CYCLE_TIME, DEFAULT_STACK_SIZE } {}

Behavior::~Behavior() {
    debug_printf<DEBUG_>(PP_ARGS("Behavior::~Behavior() for \"{s}\": removing task {#x}...\r\n", get_name().c_str(), task_id_));
    // debug_flush<DEBUG_>();

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
    // debug_flush<DEBUG_>();

    while (!finished_) {
        {
            std::unique_lock<std::mutex> lk(caller_mutex_);
            caller_cv_.wait(lk);
        }
        if (!finished_) {
            debug_printf<DEBUG_>(PP_ARGS("Behavior::wait(): \"{s}\" woke up, but not finished. Probably a BUG.\r\n", get_name().c_str()));
            // debug_flush<DEBUG_>();

            using namespace std::chrono_literals;
            std::this_thread::sleep_for(100ms);
        }
    }
    debug_printf<DEBUG_>(PP_ARGS("Behavior::wait(): \"{s}\" finished.\r\n", get_name().c_str()));
    // debug_flush<DEBUG_>();
}

void Behavior::wait_for_model_update() {
    get_ctbot()->wait_for_model_update(abort_request_);
}

bool Behavior::exit() {
    if (!finished_) {
        finished_ = true;

        debug_printf<DEBUG_>(PP_ARGS("Behavior::exit() for \"{s}\": notifying caller...\r\n", get_name().c_str()));
        // debug_flush<DEBUG_>();

        caller_cv_.notify_all();
        return true;
    }

    debug_printf<DEBUG_>(PP_ARGS("Behavior::exit() for \"{s}\": called, but already finished!\r\n", get_name().c_str()));
    // debug_flush<DEBUG_>();
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
