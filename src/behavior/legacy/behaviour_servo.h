/*
 * c't-Bot
 *
 * This program is free software; you can redistribute it
 * and/or modify it under the terms of the GNU General
 * Public License as published by the Free Software
 * Foundation; either version 2 of the License, or (at your
 * option) any later version.
 * This program is distributed in the hope that it will be
 * useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
 * PURPOSE. See the GNU General Public License for more details.
 * You should have received a copy of the GNU General Public
 * License along with this program; if not, write to the Free
 * Software Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307, USA.
 *
 */


/**
 * \file    behaviour_servo.h
 * \brief   kontrolliert die Servos
 * \author  Benjamin Benz
 * \date    07.12.2006
 */

#ifndef BEHAVIOUR_SERVO_H_
#define BEHAVIOUR_SERVO_H_

#include "bot-logic.h"

#ifdef BEHAVIOUR_SERVO_AVAILABLE

/**
 * Dieses Verhalten fuehrt ein Servo-Kommando aus und schaltet danach den Servo wieder ab
 * \param *data der Verhaltensdatensatz
 */
void bot_servo_behaviour(Behaviour_t* data);

/**
 * Fahre den Servo an eine Position
 *
 * Es kann derzeit immer nur ein Servo aktiv sein!
 *
 * \param *caller   Der Aufrufer
 * \param servo     ID des Servos
 * \param pos       Zielposition des Servos
 * \return          Zeigen auf Verhaltensdatensatz
 */
Behaviour_t* bot_servo(Behaviour_t* caller, uint8_t servo, uint8_t pos);

#endif // BEHAVIOUR_SERVO_AVAILABLE
#endif // BEHAVIOUR_SIMPLE_H_
