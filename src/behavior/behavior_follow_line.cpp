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
 * @file    behavior_follow_line.cpp
 * @brief   Follow line behavior
 * @author  Timo Sandmann
 * @date    26.05.2019
 */

#include "behavior_follow_line.h"
#include "sensors.h"

#include <algorithm>


namespace ctbot {

/* register behavior to global behavior list */
decltype(BehaviorFollowLine::reg_) BehaviorFollowLine::reg_ { "line", []() { return INIT<BehaviorFollowLine>(); } };

BehaviorFollowLine::BehaviorFollowLine(const uint16_t priority)
    : Behavior { PSTR("LineBeh"), true, priority, Behavior::DEFAULT_CYCLE_TIME, STACK_SIZE }, last_speed_l_ {}, last_speed_r_ {} {
    debug_printf<DEBUG_>(PP_ARGS("BehaviorFollowLine::BehaviorFollowLine({})\r\n", priority));
}

BehaviorFollowLine::~BehaviorFollowLine() {
    debug_print<DEBUG_>(PSTR("BehaviorFollowLine::~BehaviorFollowLine()\r\n"));

    abort_beh();
    wait();
}

void BehaviorFollowLine::run() {
    /* wait for sensor data to get updated */
    wait_for_model_update();

    if (abort_request_) {
        debug_print<DEBUG_>(PSTR("FollowLineBeh::run(): aborted.\r\n"));
        motor_update_done();
        exit();
        return;
    }

    int16_t new_speed_l, new_speed_r;
    if (get_sensors()->get_line_l() >= LINE_TRESHOLD && get_sensors()->get_line_r() < LINE_TRESHOLD) {
        /* bot is driving on right edge of line */
        debug_print<DEBUG_>(PSTR("FollowLineBeh::run(): ON RIGHT EDGE OF LINE.\r\n"));
        new_speed_l = SPEED_ON_LINE;
        new_speed_r = SPEED_ON_LINE;
    } else if (get_sensors()->get_line_l() < LINE_TRESHOLD) {
        /* bot is driving next to right edge of line -> turn to the left */
        debug_print<DEBUG_>(PSTR("FollowLineBeh::run(): NEXT TO RIGHT EDGE.\r\n"));
        new_speed_l = std::max<int16_t>(last_speed_l_ - 1, -SPEED_OFF_LINE); // decrease speed of left wheel down to -SPEED_OFF_LINE
        new_speed_r = std::min<int16_t>(last_speed_r_ + 1, SPEED_OFF_LINE); // increase speed of right wheel up to +SPEED_OFF_LINE

        if (new_speed_l == 0 && new_speed_r == 0) {
            /* updated speed reached zero -> set speeds to slow turning anticlockwise */
            new_speed_l = -5;
            new_speed_r = 5;
        }
    } else {
        /* bot is driving on line */
        debug_print<DEBUG_>(PSTR("FollowLineBeh::run(): ON LINE.\r\n"));
        debug_printf<DEBUG_>(PP_ARGS("FollowLineBeh::run(): current speed={} {}\r\n", get_motor_l()->read(), get_motor_r()->read()));
        new_speed_l = std::min<int16_t>(last_speed_l_ + 1, SPEED_OFF_LINE); // increase speed of left wheel up to +SPEED_OFF_LINE
        new_speed_r = std::max<int16_t>(last_speed_r_ - 1, -SPEED_OFF_LINE / 2); // decrease speed of right wheel down to -SPEED_OFF_LINE/2
        debug_printf<DEBUG_>(PP_ARGS("FollowLineBeh::run(): new speed={} {}\r\n", new_speed_l, new_speed_r));

        if (new_speed_l == 0 && new_speed_r == 0) {
            /* updated speed reached zero -> set speeds to slow turning clockwise */
            new_speed_l = 5;
            new_speed_r = -5;
        }
    }

    /* set new speed */
    set_actuator(get_motor_l(), new_speed_l); // new speed for left wheel in percentage terms
    set_actuator(get_motor_r(), new_speed_r); // new speed for right wheel in percentage terms

    last_speed_l_ = new_speed_l;
    last_speed_r_ = new_speed_r;

    /* some debug output */
    debug_printf<DEBUG_>(PP_ARGS("FollowLineBeh::run(): line={#x} {#x} speed={} {} at {} ms.\r\n", get_sensors()->get_line_l(), get_sensors()->get_line_r(),
        new_speed_l, new_speed_r, Timer::get_ms()));

    motor_update_done();
}

} // namespace ctbot
