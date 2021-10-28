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
 * @file    available_behaviours.h
 * @brief   Support layer for legacy behaviors
 * @author  Timo Sandmann
 * @date    26.05.2019
 */

#pragma once

/* Demo-Verhalten */
// #define BEHAVIOUR_SIMPLE_AVAILABLE /**< Beispielverhalten */
#define BEHAVIOUR_DRIVE_SQUARE_AVAILABLE /**< Demoverhalten im Quadrat fahren */

/* Notfall-Verhalten */
// #define BEHAVIOUR_AVOID_BORDER_AVAILABLE /**< Abgruenden ausweichen */
// #define BEHAVIOUR_AVOID_COL_AVAILABLE /**< Hindernis ausweichen */
// #define BEHAVIOUR_HANG_ON_AVAILABLE /**< Erkennen des Haengenbleibens als Notfallverhalten */

/* Positionierungs-Verhalten */
// #define BEHAVIOUR_DRIVE_DISTANCE_AVAILABLE /**< Strecke fahren */
// #define BEHAVIOUR_TURN_AVAILABLE /**< Dreh Verhalten */
#define BEHAVIOUR_GOTO_POS_AVAILABLE /**< Position anfahren */
// #define BEHAVIOUR_GOTO_OBSTACLE_AVAILABLE /**< Abstand zu Hindernis einhalten */
#define BEHAVIOUR_DRIVE_STACK_AVAILABLE /**< Abfahren der auf dem Stack gesicherten Koordinaten */

/* Anwendungs-Verhalten */
#define BEHAVIOUR_ADVENTCAL_AVAILABLE /**< Adventskalender-Verhalten */
// #define BEHAVIOUR_SOLVE_MAZE_AVAILABLE /**< Wandfolger */
// #define BEHAVIOUR_FOLLOW_LINE_AVAILABLE /**< Linienfolger */
// #define BEHAVIOUR_FOLLOW_LINE_ENHANCED_AVAILABLE /**< erweiterter Linienfolger, der auch mit Unterbrechungen und Hindernissen klarkommt */
// #define BEHAVIOUR_OLYMPIC_AVAILABLE /**< Olympiadenverhalten */
#define BEHAVIOUR_CATCH_PILLAR_AVAILABLE /**< Suche eine Dose und fange sie ein */
// #define BEHAVIOUR_CLASSIFY_OBJECTS_AVAILABLE /**< Trennt zwei Arten von Dosen (hell / dunkel) */
// #define BEHAVIOUR_TRANSPORT_PILLAR_AVAILABLE /**< Transport-Pillar Verhalten */
// #define BEHAVIOUR_FOLLOW_OBJECT_AVAILABLE /**< verfolge ein (bewegliches) Objekt */
// #define BEHAVIOUR_FOLLOW_WALL_AVAILABLE /**< Follow Wall Explorer Verhalten */
// #define BEHAVIOUR_LINE_SHORTEST_WAY_AVAILABLE /**< Linienfolger ueber Kreuzungen zum Ziel */
// #define BEHAVIOUR_DRIVE_CHESS_AVAILABLE /**< Schach fuer den Bot */
// #define BEHAVIOUR_SCAN_BEACONS_AVAILABLE /**< Suchen von Landmarken zur Lokalisierung */
// #define BEHAVIOUR_UBASIC_AVAILABLE /**< uBasic Verhalten */
// #define BEHAVIOUR_ABL_AVAILABLE /**< ABL-Interpreter */
// #define BEHAVIOUR_NEURALNET_AVAILABLE /**< neuronales Netzwerk */
// #define BEHAVIOUR_DRIVE_NEURALNET_AVAILABLE /**< Fahrverhalten fuer das neuronale Netzwerk */

/* System-Verhalten */
// #define BEHAVIOUR_SERVO_AVAILABLE /**< Kontrollverhalten fuer die Servos */
// #define BEHAVIOUR_MEASURE_DISTANCE_AVAILABLE /**< Distanzesensorasuwertung */
#define BEHAVIOUR_DELAY_AVAILABLE /**< Delay-Routine als Verhalten */
#define BEHAVIOUR_CANCEL_BEHAVIOUR_AVAILABLE /**< Deaktivieren von Verhalten, wenn eine Abbruchbedingung erfuellt ist */


#ifdef BEHAVIOUR_SIMPLE_AVAILABLE
#include "behaviour_simple.h"
#endif

#ifdef BEHAVIOUR_TURN_AVAILABLE
#include "behaviour_turn.h"
#else
static inline Behaviour_t* bot_turn(Behaviour_t* caller, int16_t degrees) {
    return bot_turn_wrapper(caller, degrees);
}
static inline Behaviour_t* bot_turn_speed(Behaviour_t* caller, int16_t degrees, int16_t minspeed, int16_t maxspeed) {
    return bot_turn_speed_wrapper(caller, degrees, minspeed, maxspeed);
}
static inline Behaviour_t* bot_turn_maxspeed(Behaviour_t* caller, int16_t degrees, int16_t speed) {
    return bot_turn_maxspeed_wrapper(caller, degrees, speed);
}
extern BehaviourFunc_t bot_turn_behaviour;
#endif // BEHAVIOUR_TURN_AVAILABLE

#ifdef BEHAVIOUR_DRIVE_DISTANCE_AVAILABLE
#include "behaviour_drive_distance.h"
#elif defined BEHAVIOUR_GOTO_POS_AVAILABLE
static inline Behaviour_t* bot_goto_dist(Behaviour_t* caller, int16_t distance, int8_t dir);
static inline Behaviour_t* bot_drive_distance(Behaviour_t* caller, int8_t curve, int16_t speed, int16_t cm) {
    (void) curve;
    return bot_goto_dist(caller, (int16_t) (cm * 10), speed > 0 ? 1 : -1);
}
#else
static inline Behaviour_t* bot_drive_distance(Behaviour_t* caller, int8_t curve, int16_t speed, int16_t cm) {
    (void) curve;
    return bot_drive_wrapper(caller, speed >= 0 ? abs(cm) * 10 : abs(cm) * -10);
}
extern BehaviourFunc_t bot_drive_distance_behaviour;
static inline Behaviour_t* bot_goto_dist(Behaviour_t* caller, int16_t distance, int8_t dir) {
    return bot_drive_wrapper(caller, dir >= 0 ? distance : -distance);
}
#endif // BEHAVIOUR_DRIVE_DISTANCE_AVAILABLE

#ifdef BEHAVIOUR_GOTO_POS_AVAILABLE
#include "behaviour_goto_pos.h"
#endif

#ifdef BEHAVIOUR_DRIVE_SQUARE_AVAILABLE
#include "behaviour_drive_square.h"
#else
static inline Behaviour_t* bot_drive_square(Behaviour_t* caller) {
    return bot_square_wrapper(caller);
}
extern BehaviourFunc_t bot_drive_square_behaviour;
#endif // BEHAVIOUR_DRIVE_SQUARE_AVAILABLE

#ifdef BEHAVIOUR_AVOID_BORDER_AVAILABLE
#include "behaviour_avoid_border.h"
#endif

#ifdef BEHAVIOUR_AVOID_COL_AVAILABLE
#include "behaviour_avoid_col.h"
#endif

#ifdef BEHAVIOUR_HANG_ON_AVAILABLE
#include "behaviour_hang_on.h"
#endif

#ifdef BEHAVIOUR_MEASURE_DISTANCE_AVAILABLE
#include "behaviour_measure_distance.h"
#endif

#ifdef BEHAVIOUR_ADVENTCAL_AVAILABLE
#include "behaviour_adventcal.h"
#endif

#ifdef BEHAVIOUR_SOLVE_MAZE_AVAILABLE
#include "behaviour_solve_maze.h"
#endif

#ifdef BEHAVIOUR_FOLLOW_LINE_AVAILABLE
#include "behaviour_follow_line.h"
// #include "behaviour_follow_line_enhanced.h"
#else
static inline Behaviour_t* bot_follow_line(Behaviour_t* caller, uint8_t search) {
    return bot_follow_line_wrapper(caller, search);
}
extern BehaviourFunc_t bot_follow_line_behaviour;
#endif // BEHAVIOUR_FOLLOW_LINE_AVAILABLE

#ifdef BEHAVIOUR_DRIVE_STACK_AVAILABLE
#include "behaviour_drive_stack.h"
#endif

#ifdef BEHAVIOUR_OLYMPIC_AVAILABLE
#include "behaviour_olympic.h"
#endif

#ifdef BEHAVIOUR_CATCH_PILLAR_AVAILABLE
#include "behaviour_catch_pillar.h"
#endif

#ifdef BEHAVIOUR_CLASSIFY_OBJECTS_AVAILABLE
#include "behaviour_classify_objects.h"
#endif

#ifdef BEHAVIOUR_FOLLOW_OBJECT_AVAILABLE
#include "behaviour_follow_object.h"
#endif

#ifdef BEHAVIOUR_FOLLOW_WALL_AVAILABLE
#include "behaviour_follow_wall.h"
#endif

#ifdef BEHAVIOUR_TRANSPORT_PILLAR_AVAILABLE
#include "behaviour_transport_pillar.h"
#endif

#ifdef BEHAVIOUR_LINE_SHORTEST_WAY_AVAILABLE
#include "behaviour_line_shortest_way.h"
#endif

#ifdef BEHAVIOUR_SERVO_AVAILABLE
#include "behaviour_servo.h"
#else
static inline Behaviour_t* bot_servo(Behaviour_t* caller, uint8_t servo, uint8_t pos) {
    return bot_servo_wrapper(caller, servo, pos);
}
extern BehaviourFunc_t bot_servo_behaviour;
#endif // BEHAVIOUR_SERVO_AVAILABLE

#ifdef BEHAVIOUR_DELAY_AVAILABLE
#include "behaviour_delay.h"
#endif

#ifdef BEHAVIOUR_GOTO_OBSTACLE_AVAILABLE
#include "behaviour_goto_obstacle.h"
#endif

#ifdef BEHAVIOUR_CANCEL_BEHAVIOUR_AVAILABLE
#include "behaviour_cancel_behaviour.h"
#endif

#ifdef BEHAVIOUR_DRIVE_CHESS_AVAILABLE
#include "behaviour_drive_chess.h"
#endif

#ifdef BEHAVIOUR_UBASIC_AVAILABLE
#include "behaviour_ubasic.h"
#endif

#ifdef BEHAVIOUR_ABL_AVAILABLE
#include "behaviour_abl.h"
#endif

#ifdef BEHAVIOUR_SCAN_BEACONS_AVAILABLE
#include "behaviour_scan_beacons.h"
#endif

#ifdef BEHAVIOUR_NEURALNET_AVAILABLE
#include "behaviour_neuralnet.h"
#endif

#ifdef BEHAVIOUR_DRIVE_NEURALNET_AVAILABLE
#include "behaviour_drive_neuralnet.h"
#endif
