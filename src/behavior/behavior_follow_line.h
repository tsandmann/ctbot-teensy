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
 * @file    behavior_follow_line.h
 * @brief   Follow line behavior
 * @author  Timo Sandmann
 * @date    26.05.2019
 */

#pragma once

#include "behavior.h"


namespace ctbot {

/**
 * @brief Follow line behavior
 *
 * @startuml{BehaviorFollowLine.png}
 *  !include behavior/behavior_follow_line.puml
 *  set namespaceSeparator ::
 *  skinparam classAttributeIconSize 0
 * @enduml
 */
class BehaviorFollowLine : public Behavior {
public:
    using Ptr = std::unique_ptr<BehaviorFollowLine>; /**< shortcut alias just for convenience */

    /**
     * @brief Constructor of BehaviorFollowLine class using custom priority
     *
     * @param[in] priority Custom priority to use for behavior task
     */
    FLASHMEM BehaviorFollowLine(const uint16_t priority);

    /**
     * @brief Constructor of BehaviorFollowLine class using default priority
     */
    BehaviorFollowLine() : BehaviorFollowLine { Behavior::DEFAULT_PRIORITY } {}

    FLASHMEM ~BehaviorFollowLine();

protected:
    /**
     * @brief Specialization of run method for follow line behavior.
     *
     * This method implements the functionality of the behavior.
     */
    virtual void run() override;

private:
    static constexpr bool DEBUG_ { true }; /**< switch for debug outputs to console */
    static constexpr uint32_t STACK_SIZE { 2048 }; /**< stack size in byte for behavior task */
    static constexpr uint16_t LINE_TRESHOLD { 300 }; /**< threshold value of line sensors for (black) line identification */
    static constexpr int16_t SPEED_ON_LINE { 10 }; /**< max. speed in percentage terms for wheels, if bot is driving on the line */
    static constexpr int16_t SPEED_OFF_LINE { 20 }; /**< max. speed in percentage terms for wheels, if bot is driving next to the line */

    static const Registry reg_; /**< helper object for global behavior registry */

    int16_t last_speed_l_, last_speed_r_;
};

} // namespace ctbot
