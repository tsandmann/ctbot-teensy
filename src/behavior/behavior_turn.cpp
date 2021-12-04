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
 * @file    behavior_turn.cpp
 * @brief   Turn behavior
 * @author  Timo Sandmann
 * @date    26.05.2019
 * @note    Based on original version https://github.com/tsandmann/ct-bot/blob/master/bot-logic/behaviour_turn.c by Torsten Evers
 */

#include "behavior_turn.h"
#include "ctbot_behavior.h"
#include "scheduler.h"
#include "timer.h"
#include "pose.h"
#include "speed.h"

#include "pprintpp.hpp"


namespace ctbot {

decltype(BehaviorTurn::reg_) BehaviorTurn::reg_ { "turn", [](const int32_t p) { return INIT<BehaviorTurn, int16_t>(p); } };

BehaviorTurn::BehaviorTurn(const int16_t degrees, const uint16_t priority, const uint16_t min_speed, const uint16_t max_speed)
    : Behavior { PSTR("TurnBeh"), true, priority, DEFAULT_CYCLE_TIME, STACK_SIZE }, min_speed_ { min_speed },
      max_speed_ { max_speed }, state_ { State::TURN }, target_ {}, old_head_ {}, turn_direction_ {} {
    debug_printf<DEBUG_>(PP_ARGS("BehaviorTurn::BehaviorTurn({},{},{},{})\r\n", degrees, priority, min_speed, max_speed));

    /* limit to one full rotation at most */
    int16_t to_turn { degrees };
    if (std::abs(to_turn) != 360) {
        to_turn %= 360;
    }
    debug_printf<DEBUG_>(PP_ARGS("BehaviorTurn::BehaviorTurn(): to_turn={}\r\n", to_turn));

    /* save current angle */
    old_head_ = get_pose()->get_heading();
    debug_printf<DEBUG_>(PP_ARGS("BehaviorTurn::BehaviorTurn(): old_head_={.2}\r\n", old_head_));

    /* calculate target angle */
    target_ = old_head_ + static_cast<float>(to_turn);
    debug_printf<DEBUG_>(PP_ARGS("BehaviorTurn::BehaviorTurn(): target_={.2}\r\n", target_));

    /* save turning direction, true: mathematical direction of rotation */
    turn_direction_ = to_turn >= 0;
    debug_printf<DEBUG_>(PP_ARGS("BehaviorTurn::BehaviorTurn(): turn_direction_={}\r\n", turn_direction_));
}

BehaviorTurn::~BehaviorTurn() {
    debug_print<DEBUG_>(PSTR("BehaviorTurn::~BehaviorTurn()\r\n"));

    abort_beh();
    wait();
}

void BehaviorTurn::run() {
    /* wait for sensor data to get updated */
    debug_printf<DEBUG_>(PSTR("BehaviorTurn::run(): wait for model at %u ms.\r\n"), Timer::get_ms());
    wait_for_model_update();

    debug_printf<DEBUG_>(PSTR("BehaviorTurn::run(): model updated at %u ms.\r\n"), Timer::get_ms());

    if (abort_request_) {
        state_ = State::ABORT;
    }

    /* calculate remaining angle to turn */
    float diff;
    if (!turn_direction_) { // clockwise
        if (get_pose()->get_heading() > old_head_ && old_head_ < 10.f) {
            target_ += 360.f; // an overflow of heading happened
            debug_printf<DEBUG_>(PP_ARGS("BehaviorTurn::run(): Ueberlauf, head={.2} old={.2}\r\n", get_pose()->get_heading(), old_head_));
        }
        diff = get_pose()->get_heading() - target_;
    } else { // anti-clockwise
        if (get_pose()->get_heading() < old_head_ && old_head_ > 350.f) {
            target_ -= 360.f; // an overflow of heading happened
            debug_printf<DEBUG_>(PP_ARGS("BehaviorTurn::run(): Ueberlauf, head={.2} old={.2}\r\n", get_pose()->get_heading(), old_head_));
        }
        diff = target_ - get_pose()->get_heading();
    }
    old_head_ = get_pose()->get_heading();

    /* print some debug output */
    if (DEBUG_) {
        // print_pose();
    }

    switch (state_) {
        case State::TURN:
            /* rotate until target angle is reached */
            if (diff > 0.75f) {
                const int16_t d_max_speed { static_cast<int16_t>(max_speed_ - min_speed_) };
                /* rotate the bot until target angle reached */
                int16_t new_speed { 0 };
                float diff_9 { diff - 90.f };
                if (diff_9 > 2.f) {
                    const float x { diff_9 < 90.f ? diff_9 / (360.f / 3.14159265358979f) : 3.14159265358979f / 2.f }; // (0; pi/2]
                    new_speed = static_cast<int16_t>(std::sin(x) * static_cast<float>(d_max_speed)); // [0; max_speed_ - min_speed_]
                }
                new_speed = new_speed + min_speed_; // [min_speed_; max_speed_]
                // debug_printf<DEBUG_>(PP_ARGS("BehaviorTurn::run(): new_speed={}\r\n", new_speed));

                set_actuator(get_motor_l(), turn_direction_ ? -new_speed : new_speed);
                set_actuator(get_motor_r(), turn_direction_ ? new_speed : -new_speed);
            } else {
                /* target angle reached, stop rotating */
                set_actuator(get_motor_l(), 0);
                set_actuator(get_motor_r(), 0);
                state_ = State::END;
            }
            break;

        case State::END:
            /* wait for overrun finished */
            if (get_speed()->get_left() == 0 && get_speed()->get_right() == 0) {
                motor_update_done();

                /* wait for both wheel standing still for at least 100 ms */
                sleep_for_ms(100);

                if (get_speed()->get_left() == 0 && get_speed()->get_right() == 0) {
                    print_pose(false);
                    debug_print<DEBUG_>(PSTR("BehaviorTurn::run(): done.\r\n"));

                    /* everything done, exit this behavior */
                    exit();
                }
                return;
            }
            break;

        case State::ABORT:
            debug_print<DEBUG_>(PSTR("BehaviorTurn::run(): aborted.\r\n"));
            exit();
            break;
    }

    debug_printf<DEBUG_>(PSTR("BehaviorTurn::run(): notify for motor update at %u ms.\r\n"), Timer::get_ms());
    motor_update_done();
}

} // namespace ctbot
