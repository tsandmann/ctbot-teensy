/*
 * This file is part of the ct-Bot teensy framework.
 * Copyright (c) 2022 Timo Sandmann
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
 * @file    behavior_search_eggs.h
 * @brief   Search eggs behavior
 * @author  Timo Sandmann
 * @date    15.04.2022
 */

#pragma once

#include "behavior.h"


namespace ctbot {

class BehaviorSearchEggs : public Behavior {
public:
    using Ptr = std::unique_ptr<BehaviorSearchEggs>;

    FLASHMEM BehaviorSearchEggs(const uint16_t priority);

    BehaviorSearchEggs() : BehaviorSearchEggs { Behavior::DEFAULT_PRIORITY } {}

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

    enum class State : uint8_t {
        SEARCH,
        DRIVE,
        RETURN,
        UNLOAD,
        TURN_AWAY,
    };

    State state_; /**< current state of behavior */
    BasePtr p_sub_beh_; /**< pointer to last called and currently running sub-behavior */
};

} // namespace ctbot
