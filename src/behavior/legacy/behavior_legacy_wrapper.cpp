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
 * @file    behavior_legacy_wrapper.cpp
 * @brief   Support layer for legacy behaviors
 * @author  Timo Sandmann
 * @date    26.05.2019
 */

#include "behavior_legacy_wrapper.h"

#include "pprintpp.hpp"

#include <chrono>
#include <thread>


namespace ctbot {

extern "C" {
namespace legacy {
#include "available_behaviours.h"
}
}

std::unordered_map<std::string /*name*/, legacy::Behaviour_t*> BehaviorLegacyWrapper::running_behaviors;

BehaviorLegacyWrapper::BehaviorLegacyWrapper(const std::string& name, legacy::BehaviourFunc_t func)
    : Behavior { name, false, Behavior::DEFAULT_PRIORITY - 1, Behavior::DEFAULT_CYCLE_TIME, STACK_SIZE }, beh_func_ { func } {}

BehaviorLegacyWrapper::~BehaviorLegacyWrapper() {
    debug_print<DEBUG_>(PSTR("BehaviorLegacyWrapper::~BehaviorLegacyWrapper().\r\n"));

    cleanup();

    debug_printf<DEBUG_>(PSTR("BehaviorLegacyWrapper::~BehaviorLegacyWrapper() done.\r\n"));
}

void BehaviorLegacyWrapper::clear_all() {
    running_behaviors.clear();
}

void BehaviorLegacyWrapper::run() {
    debug_print<DEBUG_>(PSTR("BehaviorLegacyWrapper::run().\r\n"));

    if (running_behaviors[get_name()]) {
        debug_print<DEBUG_>(PSTR("BehaviorLegacyWrapper::run(): legacy behavior already running!\r\n"));

        exit();
        return;
    }

    const auto ptr { BehaviorLegacy::get_instance()->get_behavior(beh_func_) };
    if (!ptr) {
        debug_print<DEBUG_>(PSTR("BehaviorLegacyWrapper::run(): legacy behavior not registered!\r\n"));

        exit();
        return;
    }

    running_behaviors[get_name()] = ptr;

    debug_print<DEBUG_>(PSTR("BehaviorLegacyWrapper::run(): calling start() ...\r\n"));

    start();

    debug_print<DEBUG_>(PSTR("BehaviorLegacyWrapper::run(): start() called.\r\n"));

    auto& beh_legacy { *BehaviorLegacy::get_instance() };
    do {
        using namespace std::chrono_literals;
        std::this_thread::sleep_for(100ms); // FIXME: condition?
        // debug_print<DEBUG_>("BehaviorLegacyWrapper::run(): waiting for legacy behavior to finish...\r\n");
        // debug_flush<DEBUG_>();
    } while (get_ctbot()->get_ready() && beh_legacy.behavior_is_activated(beh_func_));

    debug_print<DEBUG_>(PSTR("BehaviorLegacyWrapper::run(): legacy behavior finished!\r\n"));

    running_behaviors[get_name()] = nullptr;

    exit();
}

void BehaviorLegacyWrapper::cleanup() {
    auto legacy_beh { BehaviorLegacy::get_instance() };
    legacy_beh->abort_behavior(legacy_beh->get_behavior(beh_func_));

    if (running_behaviors[get_name()]) {
        legacy_beh->deactivate_called_behaviors(running_behaviors[get_name()]);
        legacy_beh->deactivate_behavior(running_behaviors[get_name()]);
        running_behaviors[get_name()] = nullptr;
    }
}


#if !defined BEHAVIOUR_GOTO_POS_AVAILABLE && defined BEHAVIOUR_DRIVE_DISTANCE_AVAILABLE
decltype(BehaviorDriveDistance::reg_) BehaviorDriveDistance::reg_ { REGISTRY_HELPER("DriveDistance",
    [](const int8_t curve, const int16_t speed, const int16_t cm) { return INIT<BehaviorDriveDistance, int8_t, int16_t, int16_t>(curve, speed, cm); }) };

BehaviorDriveDistance::BehaviorDriveDistance(const int8_t curve, const int16_t speed, const int16_t cm)
    : BehaviorLegacyWrapper { PSTR("DriveDistanceBeh"), legacy::bot_drive_distance_behaviour }, curve_ { curve }, speed_ { speed }, length_ { cm } {
    debug_printf<DEBUG_>(PP_ARGS("BehaviorDriveDistance::BehaviorDriveDistance({}, {}, {})\r\n", static_cast<int16_t>(curve_), speed_, length_));
    debug_flush<DEBUG_>();
}

void BehaviorDriveDistance::start() {
    legacy::bot_drive_distance(nullptr, curve_, speed_, length_);
}
#endif // ! BEHAVIOUR_GOTO_POS_AVAILABLE && BEHAVIOUR_DRIVE_DISTANCE_AVAILABLE


#ifdef BEHAVIOUR_SIMPLE_AVAILABLE
decltype(BehaviorSimple::reg_) BehaviorSimple::reg_ { REGISTRY_HELPER("Simple", []() { return INIT<BehaviorSimple>(); }) };

BehaviorSimple::BehaviorSimple() : BehaviorLegacyWrapper { PSTR("SimpleBeh"), legacy::bot_simple_behaviour } {
    debug_print<DEBUG_>(PSTR("BehaviorSimple::BehaviorSimple().\r\n"));
}

void BehaviorSimple::start() {
    legacy::bot_simple(nullptr);
}
#endif // BEHAVIOUR_SIMPLE_AVAILABLE


#ifdef BEHAVIOUR_GOTO_POS_AVAILABLE
decltype(BehaviorGotoPos::reg1_) BehaviorGotoPos::reg1_ { REGISTRY_HELPER(
    "GotoPos", [](const int16_t pos_x, const int16_t pos_y, const int16_t heading, const bool relative) {
        return INIT<BehaviorGotoPos, int16_t, int16_t, int16_t, bool>(pos_x, pos_y, heading, relative);
    }) };

decltype(BehaviorGotoPos::reg2_) BehaviorGotoPos::reg2_ { REGISTRY_HELPER(
    "GotoDist", [](const int16_t distance, const int8_t direction, const int16_t heading) {
        return INIT<BehaviorGotoPos, int16_t, int8_t, int16_t>(distance, direction, heading);
    }) };

BehaviorGotoPos::BehaviorGotoPos(int16_t pos_x, int16_t pos_y, int16_t heading, bool relative)
    : BehaviorLegacyWrapper { PSTR("GotoPosBeh"), legacy::bot_goto_pos_behaviour }, x_ { pos_x }, y_ { pos_y }, head_ { heading }, rel_ { relative }, dist_ {},
      dir_ {} {
    debug_printf<DEBUG_>(PP_ARGS("BehaviorGotoPos::BehaviorGotoPos({}, {}, {}, {}).\r\n", pos_x, pos_y, heading, relative));
}

BehaviorGotoPos::BehaviorGotoPos(int16_t distance, int8_t direction)
    : BehaviorLegacyWrapper { PSTR("GotoPosBeh"), legacy::bot_goto_pos_behaviour }, x_ {}, y_ {}, head_ { -1 }, rel_ {}, dist_ { distance }, dir_ {
          direction
      } {
    debug_printf<DEBUG_>(PP_ARGS("BehaviorGotoPos::BehaviorGotoPos({}, {}).\r\n", distance, static_cast<int16_t>(direction)));
}

BehaviorGotoPos::BehaviorGotoPos(int16_t distance, int8_t direction, int16_t heading)
    : BehaviorLegacyWrapper { PSTR("GotoPosBeh"), legacy::bot_goto_pos_behaviour }, x_ {}, y_ {}, head_ { heading }, rel_ {}, dist_ { distance }, dir_ {
          direction
      } {
    debug_printf<DEBUG_>(PP_ARGS("BehaviorGotoPos::BehaviorGotoPos({}, {}, {}).\r\n", distance, static_cast<int16_t>(direction), heading));
}

void BehaviorGotoPos::start() {
    if (dist_ && head_ == -1) {
        debug_printf<DEBUG_>(PP_ARGS("BehaviorGotoPos::start(): calling bot_goto_dist({}, {}) ...\r\n", dist_, static_cast<int16_t>(dir_)));

        legacy::bot_goto_dist(nullptr, dist_, dir_);
    } else if (dist_) {
        debug_printf<DEBUG_>(PP_ARGS("BehaviorGotoPos::start(): calling bot_goto_dist_head({}, {}, {}) ...\r\n", dist_, static_cast<int16_t>(dir_), head_));

        legacy::bot_goto_dist_head(nullptr, dist_, dir_, head_);
    } else if (rel_) {
        debug_printf<DEBUG_>(PP_ARGS("BehaviorGotoPos::start(): calling bot_goto_pos_rel({}, {}, {}) ...\r\n", x_, y_, head_));

        legacy::bot_goto_pos_rel(nullptr, x_, y_, head_);
    } else {
        debug_printf<DEBUG_>(PP_ARGS("BehaviorGotoPos::start(): calling bot_goto_pos({}, {}, {}) ...\r\n", x_, y_, head_));

        legacy::bot_goto_pos(nullptr, x_, y_, head_);
    }
}
#endif // BEHAVIOUR_GOTO_POS_AVAILABLE


#ifdef BEHAVIOUR_DRIVE_SQUARE_AVAILABLE
decltype(BehaviorDriveSquareLegacy::reg1_) BehaviorDriveSquareLegacy::reg1_ { REGISTRY_HELPER(
    "DriveSquare", []() { return INIT<BehaviorDriveSquareLegacy>(); }) };
decltype(BehaviorDriveSquareLegacy::reg2_) BehaviorDriveSquareLegacy::reg2_ { REGISTRY_HELPER(
    "DriveSquareLen", [](const uint16_t length) { return INIT<BehaviorDriveSquareLegacy, uint16_t>(length); }) };

BehaviorDriveSquareLegacy::BehaviorDriveSquareLegacy() : BehaviorLegacyWrapper { PSTR("DriveSquareBeh"), legacy::bot_drive_square_behaviour }, length_ { 400 } {
    debug_print<DEBUG_>(PSTR("BehaviorDriveSquareLegacy::BehaviorDriveSquareLegacy().\r\n"));
}

BehaviorDriveSquareLegacy::BehaviorDriveSquareLegacy(const uint16_t length)
    : BehaviorLegacyWrapper { PSTR("DriveSquareBeh"), legacy::bot_drive_square_behaviour }, length_ { length } {
    debug_printf<DEBUG_>(PP_ARGS("BehaviorDriveSquareLegacy::BehaviorDriveSquareLegacy({}).\r\n", length_));
}

void BehaviorDriveSquareLegacy::start() {
    legacy::bot_drive_square_len(nullptr, length_);
}
#endif // BEHAVIOUR_DRIVE_SQUARE_AVAILABLE


#ifdef BEHAVIOUR_CATCH_PILLAR_AVAILABLE
decltype(BehaviorCatchPillarLegacy::reg1_) BehaviorCatchPillarLegacy::reg1_ { REGISTRY_HELPER(
    "CatchPillar", [](const uint8_t mode) { return INIT<BehaviorCatchPillarLegacy, uint8_t>(mode); }) };

decltype(BehaviorCatchPillarLegacy::reg2_) BehaviorCatchPillarLegacy::reg2_ { REGISTRY_HELPER(
    "CatchPillarTurn", [](const uint8_t mode, const int16_t max_turn) { return INIT<BehaviorCatchPillarLegacy, uint8_t, int16_t>(mode, max_turn); }) };

BehaviorCatchPillarLegacy::BehaviorCatchPillarLegacy(uint8_t mode, int16_t max_turn)
    : BehaviorLegacyWrapper { PSTR("CatchPillarTurnBeh"), legacy::bot_catch_pillar_behaviour }, mode_ { mode }, max_turn_ { max_turn } {
    debug_printf<DEBUG_>(PP_ARGS("BehaviorCatchPillarLegacy::BehaviorCatchPillarLegacy({}, {}).\r\n", mode_, max_turn_));
}

BehaviorCatchPillarLegacy::BehaviorCatchPillarLegacy(const uint8_t mode) : BehaviorCatchPillarLegacy(mode, 360) {}

void BehaviorCatchPillarLegacy::start() {
    legacy::bot_catch_pillar(nullptr, mode_);
}


decltype(BehaviorUnloadPillarLegacy::reg1_) BehaviorUnloadPillarLegacy::reg1_ { REGISTRY_HELPER(
    "UnloadPillar", []() { return INIT<BehaviorUnloadPillarLegacy>(); }) };

BehaviorUnloadPillarLegacy::BehaviorUnloadPillarLegacy() : BehaviorLegacyWrapper { PSTR("UnloadPillarBeh"), legacy::bot_unload_pillar_behaviour } {
    debug_print<DEBUG_>(PSTR("BehaviorUnloadPillarLegacy::BehaviorUnloadPillarLegacy().\r\n"));
}

void BehaviorUnloadPillarLegacy::start() {
    legacy::bot_unload_pillar(nullptr);
}
#endif // BEHAVIOUR_CATCH_PILLAR_AVAILABLE


#ifdef BEHAVIOUR_ADVENTCAL_AVAILABLE
decltype(BehaviorAdventcal::reg_) BehaviorAdventcal::reg_ { REGISTRY_HELPER("Advent", []() { return INIT<BehaviorAdventcal>(); }) };

BehaviorAdventcal::BehaviorAdventcal() : BehaviorLegacyWrapper { PSTR("AdventBeh"), legacy::bot_adventcal_behaviour } {
    debug_print<DEBUG_>(PSTR("BehaviorAdventcal::BehaviorAdventcal().\r\n"));
}

void BehaviorAdventcal::start() {
    legacy::bot_adventcal(nullptr);
}
#endif // BEHAVIOUR_ADVENTCAL_AVAILABLE

} /* namespace ctbot */
