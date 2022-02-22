/*
 * This file is part of the ct-Bot teensy framework.
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
 * @note    Pulse width calculation based on "Hardware Servo Timer Library"
 *          <http://arduiniana.org/libraries/pwmservo>
 *          by Jim Studt, David A. Mellis, Mikal Hart, Paul Stoffregen
 */

#include "servo.h"

#include "scheduler.h"

#include "arduino_freertos.h"


namespace ctbot {

Servo::Servo(const uint8_t pin, const uint8_t initial_pos) : Servo { pin, 690U, 2400U, initial_pos } {}

Servo::Servo(const uint8_t pin, const uint16_t min, const uint16_t max, const uint8_t initial_pos)
    : pin_ { pin }, min_ { static_cast<uint16_t>(min >> 4U) }, max_ { static_cast<uint16_t>(max >> 4U) }, position_ { initial_pos }, active_ { true } {
    if (pin >= 64U) {
        return;
    }
    if (!arduino::digitalPinHasPWM(pin)) {
        return;
    }

    Scheduler::enter_critical_section();
    arduino::analogWriteFrequency(pin, 50.f);
    arduino::digitalWriteFast(pin, false);
    arduino::pinMode(pin, arduino::OUTPUT);
    Scheduler::exit_critical_section();

    set(position_);
    // FIXME: wait for servo to move to target
    disable();
}

void Servo::set(const uint8_t pos) {
    if (pos > 180) {
        position_ = 180;
    } else {
        position_ = pos;
    }

    // float usec = (float)((max16 - min16) << 4) * ((float)angle / 180.f) + (float)(min16 << 4);
    // uint32_t duty = (int)(usec / 20000.f * 4096.f);
    const uint32_t us { (((max_ - min_) * 46603U * position_) >> 11U) + (min_ << 12U) }; // us * 256
    const uint32_t duty { (us * 3355U) >> 22U };

    Scheduler::enter_critical_section();
    const uint32_t oldres = arduino::analogWriteResolution(12);
    arduino::analogWrite(pin_, duty);
    arduino::analogWriteResolution(oldres);
    Scheduler::exit_critical_section();

    active_ = true;
}

void Servo::disable() {
    /* turn servo off to save power */
    Scheduler::enter_critical_section();
    arduino::analogWrite(pin_, 0);
    Scheduler::exit_critical_section();

    active_ = false;
}

} // namespace ctbot
