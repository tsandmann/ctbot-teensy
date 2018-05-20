/*
 * This file is part of the c't-Bot teensy framework.
 * Copyright (c) 2018 Timo Sandmann
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
 * @file    servo.cpp
 * @brief   Servo driver
 * @author  Timo Sandmann
 * @date    13.05.2018
 */

#include "servo.h"

#include <PWMServo.h>


namespace ctbot {

Servo::Servo(const uint8_t pin, const uint8_t initial_pos) : pin_ { pin }, p_impl_ { new PWMServo() }, position_ { initial_pos } {
    if (! p_impl_) {
        return;
    }

    p_impl_->attach(pin);
    set(position_);
    // FIXME: wait for servo to move to target
    disable();
}

void Servo::set(const uint8_t pos) {
    if (! p_impl_) {
        return;
    }

    /* set servo position by turning the PWM signal on */
    p_impl_->write(pos);
    position_ = pos;
}

void Servo::disable() const {
    if (! p_impl_) {
        return;
    }

    /* turn servo off to save power */
    p_impl_->disable();
}

} /* namespace ctbot */
