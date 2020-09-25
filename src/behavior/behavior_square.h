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
 * @file    behavior_square.h
 * @brief   Drive square behavior
 * @author  Timo Sandmann
 * @date    26.05.2019
 */

#pragma once

#include "behavior.h"


namespace ctbot {

/**
 * @brief Drive square behavior
 *
 * @startuml{BehaviorSquare.png}
 *  !include behavior/behavior_square.puml
 *  set namespaceSeparator ::
 *  skinparam classAttributeIconSize 0
 * @enduml
 */
class BehaviorSquare : public Behavior {
public:
    using Ptr = std::unique_ptr<BehaviorSquare>; /**< shortcut alias just for convenience */

    /**
     * @brief Constructor of BehaviorSquare class using custom priority
     *
     * @param[in] priority Custom priority to use for behavior task
     */
    FLASHMEM BehaviorSquare(const uint16_t priority);

    /**
     * @brief Constructor of BehaviorSquare class using default priority
     */
    BehaviorSquare() : BehaviorSquare { Behavior::DEFAULT_PRIORITY } {}

protected:
    /**
     * @brief Specialization of run method for simple behavior.
     *
     * This method implements the functionality of the behavior.
     */
    virtual void run() override;

private:
    static constexpr bool DEBUG_ { true }; /**< switch for debug outputs to console */
    static constexpr uint32_t STACK_SIZE { 2048 }; /**< stack size in byte for behavior task */

    static const Registry reg_; /**< helper object for global behavior registry */

    /**
     * @brief All possible states of this behavior
     */
    enum class State : uint8_t {
        DRIVE,
        TURN,
    };

    State state_; /**< current state of behavior */
    uint8_t counter_; /**< counter for already driven edges */
    BasePtr p_sub_beh_; /**< pointer to last called and currently running sub-behavior */
};

} // namespace ctbot
