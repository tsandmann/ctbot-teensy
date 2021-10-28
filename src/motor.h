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
 * @file    motor.h
 * @brief   Motor driver
 * @author  Timo Sandmann
 * @date    13.05.2018
 */

#pragma once

#include "encoder.h"

#include <cstdint>


namespace ctbot {

/**
 * @brief Motor driver
 *
 * @startuml{Motor.png}
 *  !include motor.puml
 *  set namespaceSeparator ::
 *  skinparam classAttributeIconSize 0
 * @enduml
 */
class Motor {
protected:
    static constexpr float PWM_FREQUENCY { 29'296.f }; /**< Pwm frequency in Hz */

    int16_t pwm_;
    const uint8_t pwm_pin_;
    const uint8_t dir_pin_;
    const bool invert_dir_;
    Encoder& enc_;

public:
    static constexpr uint8_t PWM_RESOLUTION { 11 }; /**< Pwm resolution in bits */

    /**
     * @brief Construct a new Motor object
     * @param[in] enc: Reference to encoder sensor of this motor (used to set direction)
     * @param[in] pin_pwm: Pin number of the pwm signal, only used for initialization
     * @param[in] pin_dir: Pin number of the direction signal
     * @param[in] invert: Invert motor direction setting; set to true, if wheel turning direction should be inverted
     */
    FLASHMEM Motor(Encoder& enc, const uint8_t pin_pwm, const uint8_t pin_dir, const bool invert);

    /**
     * @brief Set a new pwm duty cycle
     * @param[in] pwm: New pwm duty cycle to set; [- CtBotConfig::MOT_PWM_MAX; CtBotConfig::MOT_PWM_MAX]
     */
    void set(const int pwm);

    /**
     * @brief Set a new pwm duty cycle relative to max speed
     * @param[in] pwm_rel: New pwm duty cycle as ratio of max speed; [-1; +1]
     */
    void set(const float pwm_rel) {
        set(static_cast<int>(pwm_rel * CtBotConfig::MOT_PWM_MAX));
    }

    /**
     * @return Current pwm duty cycle set; [- CtBotConfig::MOT_PWM_MAX; CtBotConfig::MOT_PWM_MAX]
     */
    auto get() const {
        return pwm_;
    }
};

} // namespace ctbot
