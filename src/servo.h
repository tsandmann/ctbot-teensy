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
 * @file    servo.h
 * @brief   Servo driver
 * @author  Timo Sandmann
 * @date    13.05.2018
 */

#ifndef SRC_SERVO_H_
#define SRC_SERVO_H_

#include "ctbot_config.h"

#include <cstdint>


class PWMServo;

namespace ctbot {

/**
 * @brief Servo management
 */
class Servo {
public:
    /**
     * @brief Create a servo instance for given servo number
     * @param[in] pin: Pin used for the servo
     * @param[in] initial_pos: Position to set at initialization
     */
    Servo(const uint8_t pin, const uint8_t initial_pos = 90);

    /**
     * @brief Set the servo to a position
     * @param[in] pos: Target position [0; 180] or POS_OFF to turn servo off
     */
    void set(const uint8_t pos);

    /**
     * @brief Get the last set servo position
     * @return Position of servo or POS_OFF, if servo turned off
     */
    auto get() const {
        return position_;
    }

    /**
     * @brief Disable the servo (set the PWM signal low)
     */
    void disable() const;

protected:
    const uint8_t pin_; /**< Pin used for the servo */
    PWMServo* p_impl_; /**< Pointer to underlying PWMServo instance to use */
    uint8_t position_; /**< Target position for servo */
};

} /* namespace ctbot */

#endif /* SRC_SERVO_H_ */
