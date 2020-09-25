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
 * @file    behavior_servo.cpp
 * @brief   Servo behavior
 * @author  Timo Sandmann
 * @date    26.05.2019
 */

#include "behavior_servo.h"
#include "../sensors.h"
#include "../servo.h"


namespace ctbot {

decltype(BehaviorServo::active_) BehaviorServo::active_ { false, false };

BehaviorServo::BehaviorServo(uint8_t servo, uint8_t position, bool auto_stop)
    : Behavior { PSTR("Servo") }, p_servo_ { get_ctbot()->get_servos()[servo == 2 ? 1 : 0] }, servo_nr_ { static_cast<uint8_t>(servo == 2 ? 1 : 0) },
      servo_pos_ { position }, stop_ { auto_stop } {
    if (servo_nr_ == 0 && servo_pos_ > POS_OPEN) {
        servo_pos_ = POS_OPEN;
    } else if (servo_nr_ == 0 && servo_pos_ < POS_CLOSE) {
        servo_pos_ = POS_CLOSE;
    }
}

void BehaviorServo::run() {
    using namespace std::chrono_literals;

    if (!p_servo_ || active_[servo_nr_]) {
        exit();
        return;
    }

    active_[servo_nr_] = true;

    if (servo_nr_ == 0) {
        if (servo_pos_ == POS_OPEN) {
            for (auto i { p_servo_->get_position() }; i <= POS_OPEN; ++i) {
                p_servo_->set(i);
                std::this_thread::sleep_for(30ms);
            }
        } else if (servo_pos_ == POS_CLOSE) {
            for (auto i { p_servo_->get_position() }; i >= POS_CLOSE; --i) {
                p_servo_->set(i);
                std::this_thread::sleep_for(30ms);
            }
        }
    } else {
        p_servo_->set(servo_pos_);
        std::this_thread::sleep_for(1s);
    }

    if (stop_) {
        std::this_thread::sleep_for(2s);
        p_servo_->disable();
    }

    active_[servo_nr_] = false;

    exit();
}

} /* namespace ctbot */
