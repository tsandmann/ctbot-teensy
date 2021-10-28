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
 * @file    servo.h
 * @brief   Servo driver
 * @author  Timo Sandmann
 * @date    13.05.2018
 * @note    Pulse width calculation based on "Hardware Servo Timer Library"
 *          <http://arduiniana.org/libraries/pwmservo>
 *          by Jim Studt, David A. Mellis, Mikal Hart, Paul Stoffregen
 */

#pragma once

#include "ctbot_config.h"

#include "avr/pgmspace.h"

#include <cstdint>


class PWMServo;

namespace ctbot {

/**
 * @brief Servo management
 *
 * @startuml{Servo.png}
 *  !include servo.puml
 *  set namespaceSeparator ::
 *  skinparam classAttributeIconSize 0
 * @enduml
 */
class Servo {
public:
    /**
     * @brief Create a servo instance for given servo number
     * @param[in] pin: Pin used for the servo
     * @param[in] initial_pos: Position to set at initialization
     */
    FLASHMEM Servo(const uint8_t pin, const uint8_t initial_pos = 90);

    /**
     * @brief Create a servo instance for given servo number
     * @param[in] pin: Pin used for the servo
     * @param[in] min:
     * @param[in] max:
     * @param[in] initial_pos: Position to set at initialization
     */
    FLASHMEM Servo(const uint8_t pin, const uint16_t min, const uint16_t max, const uint8_t initial_pos = 90);

    /**
     * @brief Set the servo to a position
     * @param[in] pos: Target position [0; 180] or POS_OFF to turn servo off
     */
    void set(const uint8_t pos);

    /**
     * @brief Disable the servo (set the PWM signal low)
     */
    void disable();

    /**
     * @brief Get the last set servo position
     * @return Position of servo or POS_OFF, if servo turned off
     */
    auto get_position() const {
        return position_;
    }

    /**
     * @brief Get the current servo status
     * @return true, if servo is currently active (duty cylce > 0)
     */
    auto get_active() const {
        return active_;
    }

protected:
    const uint8_t pin_; /**< Pin used for the servo */
    const uint16_t min_; /**< Minimum pulse, 16 uS units (default is 34) */ // FIXME: improve resolution?
    const uint16_t max_; /**< Maximum pulse, 16 uS units, 0-4 ms range (default is 150) */
    uint8_t position_; /**< Target position for servo */
    bool active_; /**< Flag to indicate if servo is currently active (duty cylce > 0) */
};

} // namespace ctbot
