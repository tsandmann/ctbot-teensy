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
 * @file    behavior_search_eggs.cpp
 * @brief   Search eggs behavior
 * @author  Timo Sandmann
 * @date    15.04.2022
 */

#include "behavior_search_eggs.h"

#include "legacy/behavior_legacy_wrapper.h"
#include "behavior_drive.h"
#include "behavior_turn.h"
#include "position_store.h"

namespace ctbot {
/* register behavior to global behavior list */
decltype(BehaviorSearchEggs::reg_) BehaviorSearchEggs::reg_ { "eggs", []() { return INIT<BehaviorSearchEggs>(); } };

BehaviorSearchEggs::BehaviorSearchEggs(const uint16_t priority)
    : Behavior { PSTR("SearchEggs"), false, priority, Behavior::DEFAULT_CYCLE_TIME, STACK_SIZE }, state_ { State::SEARCH } {
    debug_print<DEBUG_>(PSTR("BehaviorSearchEggs::BehaviorSearchEggs(): init done.\r\n"));
}

void BehaviorSearchEggs::run() {
    switch (state_) {
        case State::SEARCH:
            legacy::bot_save_waypos(nullptr, 1);

            /* create catch pillar behavior to catch the egg and wait for it to finish */
            debug_print<DEBUG_>(PSTR("BehaviorSearchEggs::run(): starting BehaviorCatchPillarLegacy(0, 360)...\r\n"));
            p_sub_beh_ = switch_to<BehaviorCatchPillarLegacy>(0, 360);
            if (p_sub_beh_->get_result()) {
                debug_print<DEBUG_>(PSTR("BehaviorSearchEggs::run(): BehaviorCatchPillarLegacy() failed.\r\n"));
                exit(RESULT_FAILURE);
                break;
            }

            /* switch to RETURN state on next execution */
            debug_print<DEBUG_>(PSTR("BehaviorSearchEggs::run(): BehaviorCatchPillarLegacy() finished, bot @ "));
            if constexpr (DEBUG_) {
                print_pose(true);
            }

            state_ = CtBotConfig::SERVO_1_PIN < 255 ? State::RETURN : State::DRIVE;
            break;

        case State::DRIVE:
            /* create drive behavior to load the egg in the pocket and wait for it to finish */
            debug_print<DEBUG_>(PSTR("BehaviorSearchEggs::run(): starting BehaviorDrive(20)...\r\n"));
            p_sub_beh_ = switch_to<BehaviorDrive>(20);

            /* switch to RETURN state on next execution */
            debug_print<DEBUG_>(PSTR("BehaviorSearchEggs::run(): BehaviorDrive() finished, bot @ "));
            if constexpr (DEBUG_) {
                print_pose(true);
            }

            state_ = State::RETURN;
            break;

        case State::RETURN:
            /* create drive stack behavior to return to base and wait for it to finish */
            debug_print<DEBUG_>(PSTR("BehaviorSearchEggs::run(): starting BehaviorDriveStack()...\r\n"));
            p_sub_beh_ = switch_to<BehaviorDriveStack>();

            /* switch to UNLOAD state on next execution */
            debug_print<DEBUG_>(PSTR("BehaviorSearchEggs::run(): BehaviorDriveStack() finished, bot @ "));
            if constexpr (DEBUG_) {
                print_pose(true);
            }

            state_ = State::UNLOAD;
            break;

        case State::UNLOAD:
            /* create unload pillar behavior to unload the egg and wait for it to finish */
            debug_print<DEBUG_>(PSTR("BehaviorSearchEggs::run(): starting BehaviorUnloadPillarLegacy()...\r\n"));
            p_sub_beh_ = switch_to<BehaviorUnloadPillarLegacy>();

            /* switch to TURN_AWAY state on next execution */
            debug_print<DEBUG_>(PSTR("BehaviorSearchEggs::run(): BehaviorUnloadPillarLegacy() finished, bot @ "));
            if constexpr (DEBUG_) {
                print_pose(true);
            }

            state_ = State::TURN_AWAY;
            break;

        case State::TURN_AWAY:
            /* create turn behavior to turn bot to "hide" the egg from it */
            debug_print<DEBUG_>(PSTR("BehaviorSearchEggs::run(): starting BehaviorTurn(45)...\r\n"));
            p_sub_beh_ = switch_to<BehaviorTurn>(45);

            /* exit on next execution */
            debug_print<DEBUG_>(PSTR("BehaviorSearchEggs::run(): BehaviorTurn() finished, bot @ "));
            if constexpr (DEBUG_) {
                print_pose(true);
            }

            // sleep_for_ms(500);
            exit(RESULT_SUCCESS);
            break;
    }
}

} // namespace ctbot
