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
 * @file    behavior_square.cpp
 * @brief   Drive square behavior
 * @author  Timo Sandmann
 * @date    26.05.2019
 */

#include "behavior_square.h"
#include "behavior_drive.h"
#include "behavior_turn.h"

#include <thread>
#include <chrono>


namespace ctbot {

/* register behavior to global behavior list */
decltype(BehaviorSquare::reg_) BehaviorSquare::reg_ { "square", []() { return INIT<BehaviorSquare>(); } };

BehaviorSquare::BehaviorSquare(const uint16_t priority)
    : Behavior { "DriveSquare", priority, Behavior::DEFAULT_CYCLE_TIME, STACK_SIZE }, state_ { State::DRIVE }, counter_ {} {
    debug_print<DEBUG_>("BehaviorSquare::BehaviorSquare(): init done.\r\n");
}

void BehaviorSquare::run() {
    using namespace std::chrono_literals;

    switch (state_) {
        case State::DRIVE:
            /* create drive behavior to drive 300 mm and wait for it to finish */
            p_sub_beh_ = switch_to<BehaviorDrive>(300);

            /* pause for 0.5s and switch to TURN state on next execution */
            std::this_thread::sleep_for(500ms);
            state_ = State::TURN;
            break;

        case State::TURN:
            /* create turn behavior to turn 90 degrees to the left and wait for it to finish */
            p_sub_beh_ = switch_to<BehaviorTurn>(90);

            /* pause for 0.5s and switch to DRIVE state on next execution */
            std::this_thread::sleep_for(500ms);
            state_ = State::DRIVE;

            /* check for a full square */
            if (++counter_ == 4) {
                if (CtBotConfig::AUDIO_AVAILABLE) {
                    /* make some R2-D2 like beeping */
                    get_ctbot()->play_wav("r2d2-1.wav"); // sound is played asynchronously
                }

                /* exit behavior */
                exit();
            }
            break;
    }
}

} // namespace ctbot
