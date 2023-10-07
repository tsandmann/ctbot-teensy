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
 * @file    behavior_legacy.cpp
 * @brief   Support layer for legacy behaviors
 * @author  Timo Sandmann
 * @date    26.05.2019
 */

#include "behavior_legacy.h"

#include "../pose.h"
#include "../speed.h"
#include "../behavior_drive.h"
#include "../behavior_follow_line.h"
#include "../behavior_servo.h"
#include "../behavior_square.h"
#include "../behavior_turn.h"

#include "../../sensors.h"
#include "../../driver/lc_display.h"
#include "../../driver/leds_i2c.h"
#include "../../driver/servo.h"

#include <cstdarg>
#include <cstdio>
#include <cstring>


namespace ctbot {

decltype(BehaviorLegacy::p_sensors_) BehaviorLegacy::p_sensors_ {};
decltype(BehaviorLegacy::p_pose_) BehaviorLegacy::p_pose_ {};
decltype(BehaviorLegacy::p_speed_) BehaviorLegacy::p_speed_ {};

extern "C" {
namespace legacy {
#include "available_behaviours.h"
}

int16_t legacy::speedWishLeft;
int16_t legacy::speedWishRight;
float legacy::factorWishLeft;
float legacy::factorWishRight;
int16_t legacy::sensEncL;
int16_t legacy::sensEncR;
int16_t legacy::sensMouseX;
int16_t legacy::sensMouseY;
int16_t legacy::sensDistL;
int16_t legacy::sensDistR;
int16_t legacy::sensDistLRaw;
int16_t legacy::sensDistRRaw;
uint8_t legacy::sensDistLToggle;
uint8_t legacy::sensDistRToggle;
int16_t legacy::sensBorderL;
int16_t legacy::sensBorderR;
int16_t legacy::sensLineL;
int16_t legacy::sensLineR;
int16_t legacy::sensLDRL;
int16_t legacy::sensLDRR;

uint8_t legacy::sensTrans;
uint8_t legacy::sensDoor;
uint8_t legacy::sensError;

int16_t legacy::x_pos;
int16_t legacy::y_pos;
float legacy::heading;
float legacy::heading_mou;
int16_t legacy::heading_int;
int16_t legacy::heading_10_int;
float legacy::heading_sin;
float legacy::heading_cos;
int16_t legacy::v_enc_left;
int16_t legacy::v_enc_right;
int16_t legacy::v_mou_left;
int16_t legacy::v_mou_right;

const float legacy::WHEEL_TO_WHEEL_DIAMETER { 97.2f };
const float legacy::WHEEL_DIAMETER { 56.7f };
const int legacy::ENCODER_MARKS { 60 };
const float legacy::WHEEL_PERIMETER { 178.1283f };
const int legacy::DISTSENSOR_POS_FW { 47 };
const int legacy::DISTSENSOR_POS_SW { 32 };
const int legacy::BORDERSENSOR_POS_FW { DISTSENSOR_POS_FW };
const int legacy::BORDERSENSOR_POS_SW { DISTSENSOR_POS_SW + 5 };
const int legacy::SENS_IR_MIN_DIST { 70 };
const int legacy::SENS_IR_MAX_DIST { 1000 };
const int legacy::SENS_IR_INFINITE { 9999 };
const int legacy::SENS_IR_SAFE_DIST { 120 };
const int legacy::BORDER_DANGEROUS { 0x3da };
const int legacy::LINE_SENSE { 0x350 };
const int legacy::MAX_PILLAR_DISTANCE { 500 };

const int16_t legacy::BOT_SPEED_IGNORE { 9'999 }; /**< wird verwendet um einen Eintrag zu ignorieren */
const int16_t legacy::BOT_SPEED_STOP { 0 }; /**< Motor aus */
const int16_t legacy::BOT_SPEED_MIN { 32 }; /**< langsamste Fahrt in mm/s */
const int16_t legacy::BOT_SPEED_SLOW { 50 }; /**< langsame Fahrt in mm/s */
const int16_t legacy::BOT_SPEED_FOLLOW { 70 }; /**< vorsichtige Fahrt, fuer Folgeverhalten in mm/s */
const int16_t legacy::BOT_SPEED_MEDIUM { 100 }; /**< mittlere Fahrt in mm/s */
const int16_t legacy::BOT_SPEED_NORMAL { 150 }; /**< normale Fahrt in mm/s  */
const int16_t legacy::BOT_SPEED_FAST { 300 }; /**< schnelle Fahrt in mm/s */
const int16_t legacy::BOT_SPEED_MAX { 400 }; /**< maximale Fahrt in mm/s */

const int legacy::LED_RECHTS { 1 << 0 };
const int legacy::LED_LINKS { 1 << 1 };
const int legacy::LED_ROT { 1 << 2 };
const int legacy::LED_ORANGE { 1 << 3 };
const int legacy::LED_GELB { 1 << 4 };
const int legacy::LED_GRUEN { 1 << 5 };
const int legacy::LED_TUERKIS { 1 << 6 };
const int legacy::LED_WEISS { 1 << 7 };
const int legacy::LED_ALL { 0xff };

const int legacy::SERVO_1 { 1 };
const int legacy::SERVO_2 { 2 };
const int legacy::DOOR_CLOSE { 10 }; /**< Rechter Anschlag des Servos */
const int legacy::DOOR_OPEN { 100 }; /**< Linker Anschlag des Servos */
const int legacy::SERVO_OFF { 255 };

legacy::Behaviour_t* switch_to_behaviour(legacy::Behaviour_t* from, void (*to)(legacy::Behaviour_t*), uint8_t mode) {
    return BehaviorLegacy::get_instance()->switch_to_behavior(from, to, mode);
}

void exit_behaviour(legacy::Behaviour_t* data, uint8_t state) {
    return BehaviorLegacy::get_instance()->exit_behavior(data, state);
}

void return_from_behaviour(legacy::Behaviour_t* data) {
    return BehaviorLegacy::get_instance()->return_from_behavior(data);
}

uint8_t behaviour_is_activated(legacy::BehaviourFunc_t function) {
    return BehaviorLegacy::get_instance()->behavior_is_activated(function);
}

legacy::Behaviour_t* activateBehaviour(legacy::Behaviour_t* from, void (*to)(legacy::Behaviour_t*)) {
    return BehaviorLegacy::get_instance()->activateBehavior(from, to);
}

void deactivate_behaviour(legacy::Behaviour_t* beh) {
    return BehaviorLegacy::get_instance()->deactivate_behavior(beh);
}

void deactivate_called_behaviours(legacy::Behaviour_t* caller) {
    return BehaviorLegacy::get_instance()->deactivate_called_behaviors(caller);
}

legacy::Behaviour_t* get_behaviour(legacy::BehaviourFunc_t function) {
    return BehaviorLegacy::get_instance()->get_behavior(function);
}

FLASHMEM int8_t register_emergency_proc(void (*function)()) {
    return BehaviorLegacy::get_instance()->register_emergency_proc(function);
}

void start_registered_emergency_procs() {
    return BehaviorLegacy::get_instance()->start_registered_emergency_procs();
}

void legacy_abort(legacy::Behaviour_t* caller) {
    BehaviorLegacy::get_instance()->abort_behavior(caller);
}

void legacy_abort_helper(legacy::Behaviour_t*) {}

void legacy_caller_behaviour(legacy::Behaviour_t* data) {
    BehaviorLegacy::get_instance()->legacy_caller_behaviour(data);
}

/**
 * Fahre den Servo an eine Position
 * @param[in] servo  Nummer des Servos
 * @param[in] pos    Zielposition des Servos
 */
legacy::Behaviour_t* bot_servo_wrapper(legacy::Behaviour_t* caller, uint8_t servo, uint8_t pos) {
    if (servo != 1 && servo != 2) {
        if (caller) {
            caller->subResult = BehaviorLegacy::BEHAVIOUR_SUBFAIL;
        }
        return nullptr;
    }

    return BehaviorLegacy::get_instance()->call<BehaviorServo>(caller, servo, pos);
}

#ifndef BEHAVIOUR_SERVO_AVAILABLE
legacy::BehaviourFunc_t bot_servo_behaviour = legacy_abort_helper;
#endif

/**
 * Dreht den Bot im mathematischen Drehsinn im Rahmen der angegebenen Geschwindigkeiten.
 * @param[in] degrees   Grad, um die der Bot gedreht wird zwischen -360 und +360. Negative Zahlen drehen im (mathematisch negativen) Uhrzeigersinn.
 * @param[in] minspeed  minimale Drehgeschwindigkeit [mm/s]
 * @param[in] maxspeed  maximale Drehgeschwindigkeit [mm/s]
 */
legacy::Behaviour_t* bot_turn_speed_wrapper(legacy::Behaviour_t* caller, int16_t degrees, int16_t minspeed, int16_t maxspeed) {
    return BehaviorLegacy::get_instance()->call<BehaviorTurn>(
        caller, degrees, (100.f / legacy::BOT_SPEED_MAX) * minspeed, (100.f / legacy::BOT_SPEED_MAX) * maxspeed);
}

/**
 * Dreht den Bot im mathematischen Drehsinn.
 * @param degrees   Grad, um die der Bot gedreht wird zwischen -360 und +360. Negative Zahlen drehen im (mathematisch negativen) Uhrzeigersinn.
 */
legacy::Behaviour_t* bot_turn_wrapper(legacy::Behaviour_t* caller, int16_t degrees) {
    return bot_turn_speed_wrapper(caller, degrees, legacy::BOT_SPEED_MIN, legacy::BOT_SPEED_SLOW);
}

/**
 * Dreht den Bot im mathematischen Drehsinn hoechstens mit der angegebenen Geschwindigkeit.
 * @param degrees   Grad, um die der Bot gedreht wird zwischen -360 und +360. Negative Zahlen drehen im (mathematisch negativen) Uhrzeigersinn.
 * @param speed     maximale Drehgeschwindigkeit [mm/s]
 */
legacy::Behaviour_t* bot_turn_maxspeed_wrapper(legacy::Behaviour_t* caller, int16_t degrees, int16_t speed) {
    return bot_turn_speed_wrapper(caller, degrees, legacy::BOT_SPEED_MIN, speed);
}

#ifndef BEHAVIOUR_TURN_AVAILABLE
legacy::BehaviourFunc_t bot_turn_behaviour = legacy_abort_helper;
#endif

legacy::Behaviour_t* bot_drive_wrapper(legacy::Behaviour_t* caller, int16_t distance) {
    return BehaviorLegacy::get_instance()->call<BehaviorDrive>(caller, distance);
}

#ifndef BEHAVIOUR_DRIVE_DISTANCE_AVAILABLE
legacy::BehaviourFunc_t bot_drive_distance_behaviour = legacy_abort_helper;
#endif

legacy::Behaviour_t* bot_square_wrapper(legacy::Behaviour_t* caller) {
    return BehaviorLegacy::get_instance()->call<BehaviorSquare>(caller);
}

#ifndef BEHAVIOUR_DRIVE_SQUARE_AVAILABLE
legacy::BehaviourFunc_t bot_drive_square_behaviour = legacy_abort_helper;
#endif

legacy::Behaviour_t* bot_follow_line_wrapper(legacy::Behaviour_t* caller, uint8_t search) {
    if (search) {
        legacy::log_error(PSTR("bot_follow_line_wrapper(): search not implemented."));
    }
    return BehaviorLegacy::get_instance()->call<BehaviorFollowLine>(caller);
}

#ifndef BEHAVIOUR_FOLLOW_LINE_AVAILABLE
legacy::BehaviourFunc_t bot_follow_line_behaviour = legacy_abort_helper;
#endif

uint32_t timer_get_tickCount32() {
    return BehaviorLegacy::get_instance()->timer_get_tickCount32();
}

int16_t turned_angle(int16_t angle) {
    int16_t diff = 0;
    if (legacy::v_enc_left == legacy::v_enc_right) {
        /* Drehrichtung nicht ermittelbar */
        return -1;
    }
    if (legacy::v_enc_right > legacy::v_enc_left) {
        /* Drehung im positiven Sinn */
        diff = legacy::heading_int - angle;
    } else {
        /* Drehung im negativen Sinn */
        diff = angle - legacy::heading_int;
    }
    if (diff < 0) {
        /* Ueberlauf */
        diff += 360;
    }
    return diff;
}

void LED_set(uint8_t led) {
    return BehaviorLegacy::get_instance()->led_set(led);
}

void servo_set(uint8_t servo, uint8_t pos) {
    BehaviorLegacy::get_instance()->servo_set(servo, pos);
}

uint8_t servo_get_pos(uint8_t servo) {
    return BehaviorLegacy::get_instance()->servo_get_pos(servo);
}

uint8_t servo_get_active(uint8_t servo) {
    return BehaviorLegacy::get_instance()->servo_get_active(servo);
}

void display_clear() {
    return BehaviorLegacy::get_instance()->display_clear();
}

void display_cursor(int16_t row, int16_t column) {
    return BehaviorLegacy::get_instance()->display_cursor(row, column);
}

FLASHMEM uint8_t display_printf(const char* format, ...) {
    va_list args;
    va_start(args, format);
    const auto res { BehaviorLegacy::get_instance()->display_printf(format, args) };
    va_end(args);

    return res;
}

FLASHMEM uint8_t display_puts(const char* text) {
    return BehaviorLegacy::get_instance()->display_puts(text);
}

FLASHMEM size_t log_error(const char* format, ...) {
    va_list args;
    va_start(args, format);
    const auto n { BehaviorLegacy::print_log(PSTR("ERROR: "), BehaviorLegacy::length_helper("ERROR: "), format, args) };
    va_end(args);
    return n;
}

FLASHMEM size_t log_info(const char* format, ...) {
    va_list args;
    va_start(args, format);
    const auto n { BehaviorLegacy::print_log(PSTR("INFO: "), BehaviorLegacy::length_helper("INFO: "), format, args) };
    va_end(args);
    return n;
}

FLASHMEM size_t log_debug(const char* format, ...) {
    if (BehaviorLegacy::GET_DEBUG()) {
        va_list args;
        va_start(args, format);
        const auto n { BehaviorLegacy::print_log(PSTR("DEBUG: "), BehaviorLegacy::length_helper("DEBUG: "), format, args) };
        va_end(args);
        return n;
    } else {
        return 0;
    }
}
} // extern C


FLASHMEM size_t BehaviorLegacy::print_log(const char* type, const size_t type_len, const char* format, va_list vlist) {
    va_list args;
    va_copy(args, vlist);
    const auto size { std::vsnprintf(nullptr, 0, format, vlist) + type_len };
    auto p_str { new std::string(size + 3, '\0') };
    std::strncpy(p_str->data(), type, type_len + 1);
    std::vsnprintf(p_str->data() + type_len, p_str->size() - type_len, format, args);
    va_end(args);

    p_str->data()[size] = '\r';
    p_str->data()[size + 1] = '\n';

    return CtBotBehavior::get_instance().get_comm()->debug_print(p_str, true);
}

BehaviorLegacy::BehaviorLegacy(const std::string& name)
    : Behavior { name, true, Behavior::DEFAULT_PRIORITY - 1, Behavior::DEFAULT_CYCLE_TIME, STACK_SIZE }, running_ {},
      max_active_priority_ { 255 }, behavior_ {} {
    update_global_data();
    bot_behave_init();
    debug_printf<DEBUG_>(PP_ARGS("BehaviorLegacy::BehaviorLegacy(\"{s}\").\r\n", name.c_str()));
}

BehaviorLegacy::~BehaviorLegacy() {
    debug_print<DEBUG_>(PSTR("BehaviorLegacy::~BehaviorLegacy().\r\n"));

    abort_beh();
    wait(300);

    auto ptr { behavior_ };
    while (ptr) {
        auto next { ptr->next };
        delete ptr;
        ptr = next;
    }
}

void BehaviorLegacy::legacy_caller_behaviour(legacy::Behaviour_t* data) {
    if (!data->caller) {
        debug_print<DEBUG_>(PSTR("BehaviorLegacy::legacy_caller_behaviour(): no caller found, abort.\r\n"));

        running_ = false;
        exit_behavior(data, BEHAVIOUR_SUBFAIL);
        return;
    }

    const auto it { called_behaviors_.find(data->caller) };
    if (it == called_behaviors_.end()) {
        debug_print<DEBUG_>(PSTR("BehaviorLegacy::legacy_caller_behaviour(): no behavior with valid caller found, abort.\r\n"));

        running_ = false;
        exit_behavior(data, BEHAVIOUR_SUBFAIL);
        return;
    }

    if (it->second->finished()) {
        const std::string callee_name { it->second->get_name() };
        debug_printf<DEBUG_>(PP_ARGS("BehaviorLegacy::legacy_caller_behaviour(): behavior \"{s}\" finished.\r\n", callee_name.c_str()));

        called_behaviors_.erase(data->caller);
        debug_printf<DEBUG_>(PP_ARGS("BehaviorLegacy::legacy_caller_behaviour(): \"{s}\" deleted.\r\n", callee_name.c_str()));

        running_ = false;
        return_from_behavior(data);
    }
}

void BehaviorLegacy::abort_behavior(legacy::Behaviour_t* caller) {
    debug_printf<DEBUG_>(PP_ARGS("BehaviorLegacy::abort_behavior(\"{#x}\") for \"{s}\".\r\n", caller, get_name().c_str()));
    debug_printf<DEBUG_>(PP_ARGS("BehaviorLegacy::abort_behavior(): called_behaviors_.size()={}.\r\n", called_behaviors_.size()));

    for (auto& e : called_behaviors_) {
        debug_printf<DEBUG_>(PP_ARGS("BehaviorLegacy::abort_behavior(): called_behaviors: {#x} -> \"{s}\".\r\n", e.first, e.second->get_name().c_str()));
    }

    if (called_behaviors_.count(caller)) {
        debug_printf<DEBUG_>(PP_ARGS("BehaviorLegacy::abort_behavior() called behavior found: \"{s}\"\r\n", called_behaviors_[caller]->get_name().c_str()));

        called_behaviors_[caller]->abort_beh();
    }

    debug_print<DEBUG_>(PSTR("BehaviorLegacy::abort_behavior() done.\r\n"));
}

void BehaviorLegacy::run() {
    if (max_active_priority_ != 255) {
        debug_printf<DEBUG_>(PSTR("BehaviorLegacy::run(): wait for model at %u ms. max_active_priority_=%u\r\n"), Timer::get_ms(), max_active_priority_);
    }
    wait_for_model_update();

    if (max_active_priority_ != 255) {
        debug_printf<DEBUG_>(PSTR("BehaviorLegacy::run(): model updated at %u ms.\r\n"), Timer::get_ms());
    }

    if (abort_request_) {
        exit(RESULT_FAILURE);
        motor_update_done();
        return;
    }

    const auto last_max_active { max_active_priority_ };

    update_global_data();
    bot_behave();

    max_active_priority_ = 0;
    for (auto job { behavior_ }; job; job = job->next) {
        if (job->active) {
            if (job->priority > max_active_priority_) {
                max_active_priority_ = job->priority;
            }
            break; // behavior list is sorted
        }
    }
    if (max_active_priority_ == 0) {
        max_active_priority_ = 255;
    }
    // set_priority(max_active_priority_); // FIXME: set_priority() not implemented yet
    if (last_max_active != max_active_priority_) {
        /* Aenderung in der Verhaltensaktivitaet */
        set_active(max_active_priority_ != 255);
    }
    if (last_max_active) {
        motor_update_done();
    }

    if (finished()) {
        debug_print<DEBUG_>(PSTR("BehaviorLegacy::run(): finished.\r\n"));
        exit(RESULT_SUCCESS);
    }
}

void BehaviorLegacy::init() noexcept {
    p_sensors_ = CtBotBehavior::get_instance().get_sensors();
    p_pose_ = CtBotBehavior::get_instance().get_data()->get_res_ptr<Pose>(PSTR("model.pose_enc"));
    p_speed_ = CtBotBehavior::get_instance().get_data()->get_res_ptr<Speed>(PSTR("model.speed_enc"));
}

void BehaviorLegacy::update_global_data() noexcept {
    configASSERT(p_sensors_);
    configASSERT(p_pose_);
    configASSERT(p_speed_);

    legacy::sensEncL = p_sensors_->get_enc_l().get();
    legacy::sensEncR = p_sensors_->get_enc_r().get();

    legacy::sensMouseX = 0; // mouse not implemented
    legacy::sensMouseY = 0;

    legacy::sensDistL = p_sensors_->get_distance_l();
    legacy::sensDistR = p_sensors_->get_distance_r();
    legacy::sensDistLRaw = 0; // FIXME: distance sensor data conversion?
    legacy::sensDistRRaw = 0;
    legacy::sensDistLToggle = static_cast<uint8_t>(~legacy::sensDistLToggle);
    legacy::sensDistRToggle = static_cast<uint8_t>(~legacy::sensDistRToggle);

    legacy::sensBorderL = p_sensors_->get_border_l();
    legacy::sensBorderR = p_sensors_->get_border_r();

    legacy::sensLineL = p_sensors_->get_line_l();
    legacy::sensLineR = p_sensors_->get_line_r();

    legacy::sensLDRL = 0;
    legacy::sensLDRR = 0;

    legacy::sensTrans = p_sensors_->get_transport();
    legacy::sensError = 1;

    legacy::x_pos = p_pose_->get_x<int16_t>();
    legacy::y_pos = p_pose_->get_y<int16_t>();
    legacy::heading = p_pose_->get_heading<float>();
    legacy::heading_int = p_pose_->get_heading<int16_t>();
    legacy::heading_10_int = static_cast<int16_t>(p_pose_->get_heading<float>() * 10.f);
    legacy::heading_sin = p_pose_->get_heading_sin<float>();
    legacy::heading_cos = p_pose_->get_heading_cos<float>();
    legacy::heading_mou = legacy::heading;
    legacy::v_enc_left = p_speed_->get_left<int16_t>();
    legacy::v_enc_right = p_speed_->get_right<int16_t>();
    legacy::v_mou_left = legacy::v_enc_left;
    legacy::v_mou_right = legacy::v_enc_right;
}

void BehaviorLegacy::bot_behave_init() noexcept {
    insert_behavior_to_list(new_behavior(254, legacy::legacy_caller_behaviour, BEHAVIOUR_INACTIVE));

#ifdef BEHAVIOUR_SERVO_AVAILABLE
    insert_behavior_to_list(new_behavior(253, legacy::bot_servo_behaviour, BEHAVIOUR_INACTIVE));
#endif

#ifdef BEHAVIOUR_SIMPLE_AVAILABLE
    // Demo-Verhalten, ganz einfach. Achtung, im Moment hat es eine hoehere Prioritaet als die Gefahrenerkenner!
    insert_behavior_to_list(new_behavior(252, legacy::bot_simple_behaviour, BEHAVIOUR_INACTIVE));
    insert_behavior_to_list(new_behavior(251, legacy::bot_simple2_behaviour, BEHAVIOUR_INACTIVE));
#endif

    // Verhalten zum Schutz des Bots, hohe Prioritaet, aktiv
#ifdef BEHAVIOUR_AVOID_BORDER_AVAILABLE
    insert_behavior_to_list(new_behavior(249, legacy::bot_avoid_border_behaviour, BEHAVIOUR_INACTIVE));
#endif
#ifdef BEHAVIOUR_AVOID_COL_AVAILABLE
    insert_behavior_to_list(new_behavior(248, legacy::bot_avoid_col_behaviour, BEHAVIOUR_INACTIVE));
#endif
#ifdef BEHAVIOUR_HANG_ON_AVAILABLE
    insert_behavior_to_list(new_behavior(245, legacy::bot_hang_on_behaviour, BEHAVIOUR_INACTIVE));
    // Registrierung des Handlers zur Behandlung des Haengenbleibens
    register_emergency_proc(&legacy::hang_on_handler);
#endif

#ifdef BEHAVIOUR_DELAY_AVAILABLE
    // Delay-Routine als Verhalten
    insert_behavior_to_list(new_behavior(200, legacy::bot_delay_behaviour, BEHAVIOUR_INACTIVE));
#endif

#ifdef BEHAVIOUR_DRIVE_STACK_AVAILABLE
    insert_behavior_to_list(new_behavior(190, legacy::bot_save_waypos_behaviour, BEHAVIOUR_INACTIVE));
    // register_screen(legacy::drive_stack_display, legacy::drivestack_disp_key_handler);
#endif

#ifdef BEHAVIOUR_CANCEL_BEHAVIOUR_AVAILABLE
    // Verhalten, das andere Verhalten abbricht, sobald eine Bedingung erfuellt ist
    insert_behavior_to_list(new_behavior(154, legacy::bot_behaviour_cancel_behaviour, BEHAVIOUR_INACTIVE));
#endif

    // Alle Hilfsroutinen sind relativ wichtig, da sie auch von den Notverhalten her genutzt werden
    // Hilfsverhalten, die Befehle von Boten-Funktionen ausfuehren, erst inaktiv, werden von Boten aktiviert
#ifdef BEHAVIOUR_TURN_AVAILABLE
    insert_behavior_to_list(new_behavior(150, legacy::bot_turn_behaviour, BEHAVIOUR_INACTIVE));
#endif
#ifdef BEHAVIOUR_DRIVE_DISTANCE_AVAILABLE
    insert_behavior_to_list(new_behavior(149, legacy::bot_drive_distance_behaviour, BEHAVIOUR_INACTIVE));
#endif
#ifdef BEHAVIOUR_GOTO_POS_AVAILABLE
    insert_behavior_to_list(new_behavior(146, legacy::bot_goto_pos_behaviour, BEHAVIOUR_INACTIVE));
#endif
#ifdef BEHAVIOUR_GOTO_OBSTACLE_AVAILABLE
    insert_behavior_to_list(new_behavior(145, legacy::bot_goto_obstacle_behaviour, BEHAVIOUR_INACTIVE));
#endif

#ifdef BEHAVIOUR_MEASURE_DISTANCE_AVAILABLE
    insert_behavior_to_list(new_behavior(140, legacy::bot_measure_distance_behaviour, BEHAVIOUR_INACTIVE));
    insert_behavior_to_list(new_behavior(139, legacy::bot_check_distance_behaviour, BEHAVIOUR_INACTIVE));
#endif

#ifdef BEHAVIOUR_ADVENTCAL_AVAILABLE
    // Adventskalender
    insert_behavior_to_list(new_behavior(103, legacy::bot_adventcal_behaviour, BEHAVIOUR_INACTIVE));
#endif

#ifdef BEHAVIOUR_SOLVE_MAZE_AVAILABLE
    // Verhalten, um ein Labyrinth nach der Hoehlenforscher-Methode loesen
    insert_behavior_to_list(new_behavior(100, legacy::bot_solve_maze_behaviour, BEHAVIOUR_INACTIVE));
    insert_behavior_to_list(new_behavior(90, legacy::bot_measure_angle_behaviour, BEHAVIOUR_INACTIVE));
    insert_behavior_to_list(new_behavior(89, legacy::bot_check_wall_behaviour, BEHAVIOUR_INACTIVE));
#endif

#ifdef BEHAVIOUR_FOLLOW_LINE_ENHANCED_AVAILABLE
    // erweiterter Linienfolge, der mit Unterbrechungen und Hindernissen klarkommt
    insert_behavior_to_list(new_behavior(71, legacy::bot_follow_line_enh_behaviour, BEHAVIOUR_INACTIVE));
#endif

#ifdef BEHAVIOUR_FOLLOW_LINE_AVAILABLE
    // Verhalten um einer Linie zu folgen
    insert_behavior_to_list(new_behavior(70, legacy::bot_follow_line_behaviour, BEHAVIOUR_INACTIVE));
#endif

#ifdef BEHAVIOUR_LINE_SHORTEST_WAY_AVAILABLE
    // Linie folgen ueber Kreuzungen hinweg zum Ziel, kuerzester Weg befindet sich danach im Stack
    insert_behavior_to_list(new_behavior(69, legacy::bot_line_shortest_way_behaviour, BEHAVIOUR_INACTIVE));
    // Ueberwacherverhalten auf Fahren in entgegengesetzte Richtung bekommt hohe Prio, um vor bot_turn zu kommen
    insert_behavior_to_list(new_behavior(169, legacy::bot_check_reverse_direction_behaviour, BEHAVIOUR_INACTIVE));
    register_screen(legacy::bot_line_shortest_way_display, legacy::driveline_disp_key_handler);
#endif

#ifdef BEHAVIOUR_OLYMPIC_AVAILABLE
    // unwichtigere Hilfsverhalten
    insert_behavior_to_list(new_behavior(55, legacy::bot_explore_behaviour, BEHAVIOUR_INACTIVE));
    insert_behavior_to_list(new_behavior(54, legacy::bot_do_slalom_behaviour, BEHAVIOUR_INACTIVE));
    // Demo-Verhalten fuer aufwendiges System, inaktiv
    insert_behavior_to_list(new_behavior(60, legacy::bot_olympic_behaviour, BEHAVIOUR_INACTIVE));
#endif

#ifdef BEHAVIOUR_DRIVE_SQUARE_AVAILABLE
    // Demo-Verhalten, etwas komplexer, inaktiv
    insert_behavior_to_list(new_behavior(51, legacy::bot_drive_square_behaviour, BEHAVIOUR_INACTIVE));
#endif

#ifdef BEHAVIOUR_FOLLOW_WALL_AVAILABLE
    // Explorer-Verhalten um einer Wand zu folgen
    insert_behavior_to_list(new_behavior(48, legacy::bot_follow_wall_behaviour, BEHAVIOUR_INACTIVE));
    // Registrierung zur Behandlung des Notfallverhaltens zum R ueckwaertsfahren
    register_emergency_proc(&legacy::border_follow_wall_handler);
#endif

#ifdef BEHAVIOUR_CLASSIFY_OBJECTS_AVAILABLE
    insert_behavior_to_list(new_behavior(45, legacy::bot_classify_objects_behaviour, BEHAVIOUR_INACTIVE));
#endif

#ifdef BEHAVIOUR_CATCH_PILLAR_AVAILABLE
    insert_behavior_to_list(new_behavior(44, legacy::bot_catch_pillar_behaviour, BEHAVIOUR_INACTIVE));
    insert_behavior_to_list(new_behavior(43, legacy::bot_unload_pillar_behaviour, BEHAVIOUR_INACTIVE));
#endif

#ifdef BEHAVIOUR_TRANSPORT_PILLAR_AVAILABLE
    insert_behavior_to_list(new_behavior(41, legacy::bot_transport_pillar_behaviour, BEHAVIOUR_INACTIVE));
    register_screen(legacy::transportpillar_display, legacy::trpill_disp_key_handler);
#endif

#ifdef BEHAVIOUR_FOLLOW_OBJECT_AVAILABLE
    insert_behavior_to_list(new_behavior(40, legacy::bot_follow_object_behaviour, BEHAVIOUR_INACTIVE));
#endif

#ifdef BEHAVIOUR_DRIVE_CHESS_AVAILABLE
    insert_behavior_to_list(new_behavior(39, legacy::bot_drive_chess_behaviour, BEHAVIOUR_INACTIVE));
#endif

#ifdef BEHAVIOUR_ABL_AVAILABLE
    // Verhalten, das ABL interpretiert und per Remote-Call andere Verhalten starten kann
    insert_behavior_to_list(new_behavior(35, legacy::bot_abl_behaviour, BEHAVIOUR_INACTIVE));
#endif

#ifdef BEHAVIOUR_UBASIC_AVAILABLE
    insert_behavior_to_list(new_behavior(34, legacy::bot_ubasic_behaviour, BEHAVIOUR_INACTIVE));
#endif

#ifdef BEHAVIOUR_DRIVE_STACK_AVAILABLE
    insert_behavior_to_list(new_behavior(33, legacy::bot_drive_stack_behaviour, BEHAVIOUR_INACTIVE));
#endif

#ifdef BEHAVIOUR_SCAN_BEACONS_AVAILABLE
    insert_behavior_to_list(new_behavior(32, legacy::bot_scan_beacons_behaviour, BEHAVIOUR_INACTIVE));
#endif

    debug_print<DEBUG_>(PSTR("BehaviorLegacy::bot_behave_init() done.\r\n"));
}

legacy::Behaviour_t* BehaviorLegacy::new_behavior(uint8_t priority, void (*work)(legacy::Behaviour_t*), uint8_t active) const noexcept {
    auto p_new_beh { new legacy::Behaviour_t };

    p_new_beh->priority = priority;
    legacy::bit_t tmp { active };
    p_new_beh->active = tmp.bit;
    p_new_beh->next = nullptr;
    p_new_beh->work = work;
    p_new_beh->caller = nullptr;
    p_new_beh->subResult = BEHAVIOUR_SUBSUCCESS;

    return p_new_beh;
}

void BehaviorLegacy::insert_behavior_to_list(legacy::Behaviour_t* behave) noexcept {
    if (behave == nullptr) {
        return;
    }

    LOG_DEBUG("insert_behavior_to_list(%p): behavior_=%p", behave, behavior_);
    /* Erster Eintrag in der Liste? */
    if (behavior_ == nullptr) {
        behavior_ = behave;
        LOG_DEBUG("insert_behavior_to_list() 1: behavior_ set to %p", behavior_);
    } else {
        /* Gleich mit erstem Eintrag tauschen? */
        if (behavior_->priority < behave->priority) {
            behave->next = behavior_;
            behavior_ = behave;
            LOG_DEBUG("insert_behavior_to_list() 2: behavior_ set to %p", behavior_);
        } else {
            auto ptr { behavior_ };
            /* Mit dem naechsten Eintrag vergleichen */
            while (nullptr != ptr->next) {
                if (ptr->next->priority < behave->priority) {
                    break;
                }

                /* Naechster Eintrag */
                ptr = ptr->next;
            }

            auto temp { ptr->next };
            ptr->next = behave;
            behave->next = temp;
            LOG_DEBUG("insert_behavior_to_list() 3: ptr->next set to %p", ptr->next);
        }
    }
}

uint8_t BehaviorLegacy::bot_behave() const noexcept {
    uint8_t prio {};
    auto factorLeft { 1.f }; // Puffer fuer Modifikatoren
    auto factorRight { 1.f }; // Puffer fuer Modifikatoren
    for (auto job { behavior_ }; job; job = job->next) {
        if (job->active) {
            /* Wunsch-Variablen initialisieren */
            legacy::speedWishLeft = legacy::BOT_SPEED_IGNORE;
            legacy::speedWishRight = legacy::BOT_SPEED_IGNORE;

            legacy::factorWishLeft = 1.f;
            legacy::factorWishRight = 1.f;

            if (job->work) { // hat das Verhalten eine Work-Routine
                job->work(job); // Verhalten ausfuehren
            } else { // wenn nicht: Verhalten deaktivieren, da es nicht sinnvoll arbeiten kann
                job->active = BEHAVIOUR_INACTIVE;
            }

            /* Modifikatoren sammeln  */
            factorLeft *= legacy::factorWishLeft;
            factorRight *= legacy::factorWishRight;

            /* Geschwindigkeit aendern? */
            if ((legacy::speedWishLeft != legacy::BOT_SPEED_IGNORE) || (legacy::speedWishRight != legacy::BOT_SPEED_IGNORE)) {
                if (legacy::speedWishLeft != legacy::BOT_SPEED_IGNORE) {
                    legacy::speedWishLeft = static_cast<int16_t>(legacy::speedWishLeft * factorLeft);
                }
                if (legacy::speedWishRight != legacy::BOT_SPEED_IGNORE) {
                    legacy::speedWishRight = static_cast<int16_t>(legacy::speedWishRight * factorRight);
                }
                prio = job->priority;
                motor_set(legacy::speedWishLeft, legacy::speedWishRight, prio);
                break; // Wenn ein Verhalten Werte direkt setzen will, nicht weitermachen
            }
        }
        /* Dieser Punkt wird nur erreicht, wenn keine Regel im System die Motoren beeinflusen will */
        if (job->next == nullptr) {
            motor_set(legacy::BOT_SPEED_IGNORE, legacy::BOT_SPEED_IGNORE, prio);
            break;
        }
    }
    return prio;
}

legacy::Behaviour_t* BehaviorLegacy::switch_to_behavior(legacy::Behaviour_t* from, void (*to)(legacy::Behaviour_t*), uint8_t mode) noexcept {
    auto job { get_behavior(to) };
    if (job == nullptr) {
        /* Zielverhalten existiert gar nicht */
        if (from) {
            from->subResult = BEHAVIOUR_SUBFAIL;
        }
        LOG_ERROR("switch_to_behavior(): behavior not found");
        return nullptr;
    }

    legacy::bit_t tmp;
    legacy::behaviour_mode_t beh_mode;
    tmp.byte = static_cast<uint8_t>(mode & 1);
    beh_mode.override = tmp.bit;
    tmp.byte = static_cast<uint8_t>((mode & 2) >> 1);
    beh_mode.background = tmp.bit;

    if (job->caller || job->active || job->subResult == BEHAVIOUR_SUBRUNNING) { // Ist das auzurufende Verhalten noch beschaeftigt?
        if (beh_mode.override == BEHAVIOUR_NOOVERRIDE) { // nicht ueberschreiben, sofortige Rueckkehr
            if (from) {
                from->subResult = BEHAVIOUR_SUBFAIL;
            }
            return nullptr;
        }
        if (job->caller) {
            // Wir wollen also ueberschreiben, aber nett zum alten Aufrufer sein und ihn darueber benachrichtigen
            job->caller->active = BEHAVIOUR_ACTIVE; // alten Aufrufer reaktivieren
            job->caller->subResult = BEHAVIOUR_SUBFAIL; // er bekam aber nicht das gewuenschte Resultat
        }
    }

    if (from) {
        if (beh_mode.background == 0) {
            // laufendes Verhalten abschalten
            from->active = BEHAVIOUR_INACTIVE;
            from->subResult = BEHAVIOUR_SUBRUNNING;
        } else {
            from->subResult = BEHAVIOUR_SUBBACKGR;
        }
    }

    // neues Verhalten aktivieren
    job->active = BEHAVIOUR_ACTIVE;
    // Aufrufer sichern
    job->caller = from;

    LOG_DEBUG("switch_to_behavior(): prio=%u activated", job->priority);

    return job;
}

legacy::Behaviour_t* BehaviorLegacy::get_behavior(legacy::BehaviourFunc_t function) const noexcept {
    // Einmal durch die Liste gehen, bis wir den gewuenschten Eintrag haben
    for (auto job { behavior_ }; job; job = job->next) {
        if (job->work == function) {
            return job;
        }
    }
    return nullptr;
}

legacy::Behaviour_t* BehaviorLegacy::get_next_behavior(legacy::Behaviour_t* beh) const noexcept {
    if (beh == nullptr) {
        return behavior_;
    } else {
        return beh->next;
    }
}

/**
 * Rueckgabe von True, wenn das Verhalten gerade laeuft (aktiv ist), sonst False
 * \param function Die Funktion, die das Verhalten realisiert.
 * \return true wenn Verhalten aktiv, sonst false
 */
bool BehaviorLegacy::behavior_is_activated(legacy::BehaviourFunc_t function) const noexcept {
    auto job { legacy::get_behaviour(function) };
    if (job == nullptr) {
        return false;
    }

    return job->active || (job->subResult == BEHAVIOUR_SUBRUNNING);
}

legacy::Behaviour_t* BehaviorLegacy::activateBehavior(legacy::Behaviour_t* from, void (*to)(legacy::Behaviour_t*)) noexcept {
    return switch_to_behavior(from, to, BEHAVIOUR_NOOVERRIDE | BEHAVIOUR_BACKGROUND);
}

void BehaviorLegacy::exit_behavior(legacy::Behaviour_t* data, uint8_t state) const noexcept {
    data->active = BEHAVIOUR_INACTIVE; // Unterverhalten deaktivieren
    if (data->caller) {
        data->caller->active = BEHAVIOUR_ACTIVE; // aufrufendes Verhalten aktivieren

        union {
            uint8_t byte;
            unsigned bits : 3;
        } tmp = { state };
        data->caller->subResult = tmp.bits; // Status beim Aufrufer speichern
    }
    data->caller = nullptr; // Job erledigt, Verweis loeschen
}

/**
 * liefert !=0 zurueck, wenn function ueber eine beliebige Kette (job->caller->caller ....) von anderen Verhalten job aufgerufen hat
 * \param *job          Zeiger auf den Datensatz des aufgerufenen Verhaltens
 * \param *caller_beh   Das Verhalten, das urspruenglich aufgerufen hat
 * \return              0 wenn keine Call-Abhaengigkeit besteht, ansonsten die Anzahl der Stufen
 */
uint8_t BehaviorLegacy::is_in_call_hierarchy(legacy::Behaviour_t* job, legacy::Behaviour_t* caller_beh) const noexcept {
    if (job == nullptr) {
        return 0; // Liste ist leer
    }

    for (uint8_t level { 0 }; job->caller; job = job->caller) {
        level++;
        if (job->caller == caller_beh) {
            return level; // Direkter Aufrufer in Tiefe level gefunden
        }
    }
    return 0; // function kommt in Caller-Liste von job nicht vor
} // O(n), n:=|Caller-Liste|


void BehaviorLegacy::deactivate_behavior(legacy::Behaviour_t* beh) const noexcept {
    if (beh == nullptr) {
        return;
    }
    if (beh->active == BEHAVIOUR_ACTIVE) {
        LOG_DEBUG("Verhalten %u wird deaktiviert", beh->priority);
    }
    beh->active = BEHAVIOUR_INACTIVE;
    beh->caller = nullptr; // Caller loeschen, damit Verhalten auch ohne BEHAVIOUR_OVERRIDE neu gestartet werden koennen
}


void BehaviorLegacy::deactivate_called_behaviors(legacy::Behaviour_t* caller) const noexcept {
    // Einmal durch die Liste gehen, und alle aktiven Funktionen pruefen, ob sie von dem uebergebenen Verhalten aktiviert wurden
    legacy::Behaviour_t* beh_of_function { nullptr };
    uint16_t i { 0 };
    for (auto job { behavior_ }; job; job = job->next) { // n mal
        if (job->active == BEHAVIOUR_ACTIVE) {
            ++i;
            auto level { is_in_call_hierarchy(job, caller) }; // O(n)
            /* die komplette Caller-Liste (aber auch nur die) abschalten */
            legacy::Behaviour_t* ptr { job };
            for (; level > 0; --level) { // n mal
                legacy::Behaviour_t* beh { behavior_ };
                for (; beh; beh = beh->next) { // n mal
                    /* Falls das Verhalten Caller eines anderen Verhaltens ist, duerfen wir es (noch) nicht deaktivieren! */
                    if (beh->caller == ptr) {
                        break;
                    }
                } // O(n)
                legacy::Behaviour_t* tmp { ptr };
                ptr = ptr->caller; // zur naechsten Ebene
                if (beh == nullptr) {
                    tmp->active = BEHAVIOUR_INACTIVE; // callee abschalten
                    tmp->caller = nullptr; // Caller loeschen, damit Verhalten auch ohne BEHAVIOUR_OVERRIDE neu gestartet werden koennen
                }
            } // O(n^2)
        }
        if (job == caller) {
            /* Verhalten fuer spaeter merken, wenn wir hier eh schon die ganze Liste absuchen */
            beh_of_function = job;
        }
    } // O(n^3)
    /* Verhaltenseintrag zu function benachrichtigen und wieder aktiv schalten */
    if (beh_of_function != nullptr) {
        beh_of_function->subResult = BEHAVIOUR_SUBCANCEL; // externer Abbruch
        beh_of_function->active = BEHAVIOUR_ACTIVE;
    }
} // O(n^3)

void BehaviorLegacy::deactivate_all_behaviors() const noexcept {
    // Einmal durch die Liste gehen und (fast) alle deaktivieren, Grundverhalten nicht
    for (auto job { behavior_ }; job; job = job->next) {
        if ((job->priority >= 3) && (job->priority <= 200)) {
            // Verhalten deaktivieren
            job->active = BEHAVIOUR_INACTIVE;
            job->caller = nullptr; // Caller loeschen, damit Verhalten auch ohne BEHAVIOUR_OVERRIDE neu gestartet werden koennen
        }
    }
}

void BehaviorLegacy::return_from_behavior(legacy::Behaviour_t* data) const noexcept {
    return exit_behavior(data, BEHAVIOUR_SUBSUCCESS);
}

void BehaviorLegacy::motor_set(int16_t left, int16_t right, [[maybe_unused]] uint8_t prio) const noexcept {
    /* Geschwindigkeiten pruefen */
    if (left == legacy::BOT_SPEED_IGNORE) {
        left = legacy::BOT_SPEED_STOP;
    }
    if (right == legacy::BOT_SPEED_IGNORE) {
        right = legacy::BOT_SPEED_STOP;
    }

    // set_priority(prio); // FIXME: set_priority() not implemented yet
    set_actuator(get_motor_l(), left * (100.f / legacy::BOT_SPEED_MAX));
    set_actuator(get_motor_r(), right * (100.f / legacy::BOT_SPEED_MAX));
}

void BehaviorLegacy::servo_set(uint8_t servo, uint8_t pos) noexcept {
    if (servo < legacy::SERVO_1 || servo > legacy::SERVO_2) {
        return;
    }

    // FIXME: check get_servos() for nullptr
    if (pos == legacy::SERVO_OFF) {
        get_ctbot()->get_servos()[servo - 1]->disable();
        return;
    }

    get_ctbot()->get_servos()[servo == 2 ? 1 : 0]->set(pos);
}

uint8_t BehaviorLegacy::servo_get_pos(uint8_t servo) const noexcept {
    if (servo < legacy::SERVO_1 || servo > legacy::SERVO_2) {
        return 255;
    }

    // FIXME: check get_servos() for nullptr
    return get_ctbot()->get_servos()[servo - 1]->get_position();
}

uint8_t BehaviorLegacy::servo_get_active(uint8_t servo) const noexcept {
    if (servo < legacy::SERVO_1 || servo > legacy::SERVO_2) {
        return 0;
    }

    // FIXME: check get_servos() for nullptr
    return get_ctbot()->get_servos()[servo - 1]->get_active();
}

uint32_t BehaviorLegacy::timer_get_tickCount32() const noexcept {
    return static_cast<uint32_t>(p_sensors_->get_time() / 176U);
}

void BehaviorLegacy::led_set(uint8_t led) const {
    get_ctbot()->get_leds()->set(static_cast<LedTypes>(led));
}

void BehaviorLegacy::display_clear() const noexcept {
    get_ctbot()->get_lcd()->clear();
}

void BehaviorLegacy::display_cursor(int16_t row, int16_t column) const noexcept {
    get_ctbot()->get_lcd()->set_cursor(static_cast<uint16_t>(row), static_cast<uint16_t>(column));
}

uint8_t BehaviorLegacy::display_printf(const char* format, va_list vlist) const noexcept {
    va_list args2;
    va_copy(args2, vlist);
    const auto size { std::vsnprintf(nullptr, 0, format, vlist) };
    auto p_str { std::make_unique<std::string>(size + 3, '\0') };
    std::vsnprintf(p_str->data(), p_str->size(), format, args2);
    va_end(args2);

    return get_ctbot()->get_lcd()->print(*p_str);
}

uint8_t BehaviorLegacy::display_puts(const char* text) const noexcept {
    return static_cast<uint8_t>(get_ctbot()->get_lcd()->print(text));
}

int8_t BehaviorLegacy::register_screen([[maybe_unused]] void (*function)(), [[maybe_unused]] void (*keyhandler)(int16_t* const)) const noexcept {
    return 0; // FIXME: not implemented yet
}

int8_t BehaviorLegacy::register_emergency_proc(void (*function)()) noexcept {
    emerg_functions_.push_back(function);
    return static_cast<int8_t>(emerg_functions_.size() - 1);
}

void BehaviorLegacy::start_registered_emergency_procs() const noexcept {
    for (auto f : emerg_functions_) {
        if (f) {
            f();
        }
    }
}

} /* namespace ctbot */
