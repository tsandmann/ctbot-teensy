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
 * @file    behavior_drive.cpp
 * @brief   Drive distance behavior
 * @author  Timo Sandmann
 * @date    26.05.2019
 */

#include "behavior_drive.h"
#include "ctbot_behavior.h"
#include "scheduler.h"
#include "speed.h"

#include "pprintpp.hpp"

#include <thread>
#include <chrono>


namespace ctbot {

decltype(BehaviorDrive::reg_) BehaviorDrive::reg_ { "drive", [](const int32_t p) { return INIT<BehaviorDrive, int16_t>(p); } };

BehaviorDrive::BehaviorDrive(const int16_t distance, const uint16_t priority)
    : Behavior { PSTR("DriveBeh"), priority, DEFAULT_CYCLE_TIME, STACK_SIZE }, state_ { State::DRIVE }, start_ {},
      distance_2_ { distance * distance }, forward_ { distance >= 0 } {
    debug_printf<DEBUG_>(PP_ARGS("BehaviorDrive::BehaviorDrive({},{})\r\n", distance, priority));

    /* save current pose */
    start_ = *get_pose();
    debug_print<DEBUG_>(PSTR("BehaviorDrive::BehaviorDrive(): start_="));
    if (DEBUG_) {
        start_.print(*get_ctbot()->get_comm());
    }
    debug_printf<DEBUG_>(PP_ARGS("\r\nBehaviorDrive::BehaviorDrive(): forward_={}\r\n", forward_));
}

BehaviorDrive::~BehaviorDrive() {
    debug_print<DEBUG_>(PSTR("BehaviorDrive::~BehaviorDrive()\r\n"));

    abort_beh();
    wait();
}

void BehaviorDrive::run() {
    /* wait for sensor data to get updated */
    wait_for_model_update();

    if (abort_request_) {
        state_ = State::ABORT;
    }

    switch (state_) {
        case State::DRIVE: {
            const Pose current { *get_pose() };
            if (get_pose()->get_dist_square(start_) < distance_2_) {
                set_actuator(get_motor_l(), forward_ ? SPEED : -SPEED);
                set_actuator(get_motor_r(), forward_ ? SPEED : -SPEED);
            } else {
                set_actuator(get_motor_l(), 0);
                set_actuator(get_motor_r(), 0);
                state_ = State::END;
            }
            break;
        }

        case State::END:
            /* wait for overrun finished */
            if (get_speed()->get_left() == 0 && get_speed()->get_right() == 0) {
                /* wait for both wheel standing still for at least 100 ms */
                using namespace std::chrono_literals;
                std::this_thread::sleep_for(100ms);
                if (get_speed()->get_left() == 0 && get_speed()->get_right() == 0) {
                    debug_print<DEBUG_>(PSTR("BehaviorDrive::run(): done.\r\n"));
                    if (DEBUG_) {
                        std::this_thread::sleep_for(1s); // just for testing
                    }

                    /* everything done, exit this behavior */
                    print_pose(false);
                    exit();
                }
            }
            break;

        case State::ABORT:
            debug_print<DEBUG_>(PSTR("BehaviorDrive::run(): aborted.\r\n"));
            exit();
            return;
    }

    /* print some debug output */
    if (DEBUG_) {
        print_pose();
    }
}

} // namespace ctbot
