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
 * @file    behavior_turn.h
 * @brief   Turn behavior
 * @author  Timo Sandmann
 * @date    26.05.2019
 * @note    Based on original version https://github.com/tsandmann/ct-bot/blob/master/bot-logic/behaviour_turn.c by Torsten Evers
 */

#pragma once

#include "behavior.h"


namespace ctbot {

/**
 * @brief Turn behavior
 *
 * @startuml{BehaviorTurn.png}
 *  !include behavior/behavior_turn.puml
 *  set namespaceSeparator ::
 *  skinparam classAttributeIconSize 0
 * @enduml
 */
class BehaviorTurn : public Behavior {
    // FIXME: add documentation
public:
    using Ptr = std::unique_ptr<BehaviorTurn>;

    FLASHMEM BehaviorTurn(const int16_t degrees, const uint16_t priority, const uint16_t min_speed, const uint16_t max_speed);

    BehaviorTurn(const int16_t degrees, const uint16_t min_speed, const uint16_t max_speed)
        : BehaviorTurn { degrees, DEFAULT_PRIORITY, min_speed, max_speed } {}

    BehaviorTurn(const int16_t degrees, const uint16_t priority) : BehaviorTurn { degrees, priority, 8, 38 } {}

    BehaviorTurn(const int16_t degrees) : BehaviorTurn { degrees, DEFAULT_PRIORITY } {}

    FLASHMEM virtual ~BehaviorTurn() override;

protected:
    virtual void run() override;

private:
    static constexpr bool DEBUG_ { true };
    static constexpr uint32_t STACK_SIZE { 4096 };

    static const Registry reg_;

    enum class State : uint8_t {
        TURN,
        END,
        ABORT,
    };
    const uint16_t min_speed_;
    const uint16_t max_speed_;
    State state_;
    float target_;
    float old_head_;
    bool turn_direction_; /**< true: mathematical direction of rotation */
};

} // namespace ctbot
