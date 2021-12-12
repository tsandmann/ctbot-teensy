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
 * @file    behavior_drive.h
 * @brief   Drive distance behavior
 * @author  Timo Sandmann
 * @date    26.05.2019
 */

#pragma once

#include "behavior.h"
#include "pose.h"


namespace ctbot {

/**
 * @brief Drive behavior
 *
 * @startuml{BehaviorDrive.png}
 *  !include behavior/behavior_drive.puml
 *  set namespaceSeparator ::
 *  skinparam classAttributeIconSize 0
 * @enduml
 */
class BehaviorDrive : public Behavior {
    // FIXME: add documentation
public:
    using Ptr = std::unique_ptr<BehaviorDrive>;

    FLASHMEM BehaviorDrive(const int16_t distance, const uint16_t priority);

    BehaviorDrive(const int16_t distance) : BehaviorDrive { distance, DEFAULT_PRIORITY } {}

    FLASHMEM virtual ~BehaviorDrive() override;

protected:
    virtual void run() override;

private:
    static constexpr bool DEBUG_ { false };
    static constexpr int16_t SPEED { 25 };
    static constexpr uint32_t STACK_SIZE { 2048 };

    static const Registry reg_;

    enum class State : uint8_t {
        DRIVE,
        END,
        ABORT,
    };

    State state_;
    Pose start_;
    const int32_t distance_2_;
    const bool forward_;
};

} // namespace ctbot
