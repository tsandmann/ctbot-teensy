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
 * @file    behavior_test.cpp
 * @brief   Test behavior
 * @author  Timo Sandmann
 * @date    26.05.2019
 */

#include "behavior_test.h"
#include "ctbot_behavior.h"
#include "scheduler.h"
#include "actuator.h"
#include "pose.h"
#include "speed.h"

#include <thread>
#include <chrono>


namespace ctbot {

decltype(BehaviorTest::reg_) BehaviorTest::reg_ { "test", []() { return INIT<BehaviorTest>(); } };

BehaviorTest::BehaviorTest() : Behavior { PSTR("TestBeh") }, mode_ {} {}

void BehaviorTest::run() {
    using namespace std::chrono_literals;
    wait_for_model_update();

    // print_pose();

    switch (mode_) {
        case 0:
            if (get_pose()->get_heading() < 180.f) {
                set_actuator(get_motor_l(), -20);
                set_actuator(get_motor_r(), 20);
            } else {
                print_pose();
                std::this_thread::sleep_for(1s);
                mode_ = 1;
            }
            break;

        case 1:
            if (get_pose()->get_heading() > 5.f) {
                set_actuator(get_motor_l(), 20);
                set_actuator(get_motor_r(), -20);
            } else {
                print_pose();
                std::this_thread::sleep_for(5s);
                mode_ = 0;
                exit();
            }
            break;
    }
}

} // namespace ctbot
