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
 * @file    behavior_servo.h
 * @brief   Servo behavior
 * @author  Timo Sandmann
 * @date    26.05.2019
 */

#pragma once

#include "behavior.h"

#include <chrono>
#include <memory>


namespace ctbot {

class Servo;

class BehaviorServo : public Behavior {
    static constexpr bool DEBUG_ { true };
    static constexpr uint32_t STACK_SIZE { 4096 };

public:
    using Ptr = std::unique_ptr<BehaviorServo>;

    static constexpr uint8_t POS_CLOSE { 10 }; // FIXME: correct value
    static constexpr uint8_t POS_OPEN { 100 }; // FIXME: correct value

    FLASHMEM BehaviorServo(uint8_t servo, uint8_t position, bool auto_stop = true);

protected:
    static bool active_[2];
    Servo* const p_servo_;
    const uint8_t servo_nr_;
    uint8_t servo_pos_;
    const bool stop_;

    virtual void run() override;
};

} /* namespace ctbot */
