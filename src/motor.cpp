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
 * @file    motor.cpp
 * @brief   Motor driver
 * @author  Timo Sandmann
 * @date    13.05.2018
 */

#include "motor.h"
#include "ctbot.h"
#include "scheduler.h"
#include "arduino_freertos.h"

#include <cmath>
#include <cstdlib>


namespace ctbot {

Motor::Motor(Encoder& enc, const uint8_t pin_pwm, const uint8_t pin_dir, const bool invert)
    : pwm_ {}, pwm_pin_ { pin_pwm }, dir_pin_ { pin_dir }, invert_dir_ { invert }, enc_ { enc } {
    Scheduler::enter_critical_section();
    arduino::pinMode(pin_pwm, arduino::OUTPUT);
    arduino::pinMode(pin_dir, arduino::OUTPUT);
    arduino::analogWriteFrequency(pin_pwm, PWM_FREQUENCY);
    Scheduler::exit_critical_section();

    set(0);
}

void Motor::set(int new_pwm) {
    if (new_pwm > 0) {
        enc_.set_direction(true);
    } else if (new_pwm < 0) {
        enc_.set_direction(false);
    }

    pwm_ = std::abs(new_pwm) > CtBotConfig::MOT_PWM_MAX ? (new_pwm < 0 ? -CtBotConfig::MOT_PWM_MAX : CtBotConfig::MOT_PWM_MAX) : new_pwm;

    if (invert_dir_) {
        new_pwm = -new_pwm;
    }
    arduino::digitalWriteFast(dir_pin_, new_pwm >= 0);

    const int pwm { static_cast<int>(static_cast<float>(std::abs(pwm_)) / (CtBotConfig::MOT_PWM_MAX / static_cast<float>(1 << PWM_RESOLUTION))) };
    Scheduler::enter_critical_section();
    const uint32_t old_res { arduino::analogWriteResolution(PWM_RESOLUTION) };
    arduino::analogWrite(pwm_pin_, pwm);
    arduino::analogWriteResolution(old_res);
    Scheduler::exit_critical_section();
}

} // namespace ctbot
