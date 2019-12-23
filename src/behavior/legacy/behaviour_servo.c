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
 * \file    behaviour_servo.c
 * \brief   kontrolliert die Servos
 * \author  Benjamin Benz
 * \date    07.12.2006
 */


#include "behaviour_servo.h"
#ifdef BEHAVIOUR_SERVO_AVAILABLE

#include <stddef.h>

#define DEBUG_SERVO_BEH // Schalter fuer Debug-Ausgaben

#ifndef DEBUG_SERVO_BEH
#undef LOG_DEBUG
#define LOG_DEBUG(...) \
    {}
#endif

static uint8_t servo_id = 0; /**< 0, wenn kein Servo aktiv, sonst ID des gerade aktiven Servos */

void bot_servo_behaviour(Behaviour_t* data) {
    BLOCK_BEHAVIOUR(data, 1500); // 1.5 s warten

    return_from_behaviour(data); // Verhalten aus
    LOG_DEBUG("bot_servo_behaviour(): servo_set(%u, %u)", servo_id, SERVO_OFF);
    servo_set(servo_id, SERVO_OFF); // Servo aus
    servo_id = 0;
}

Behaviour_t* bot_servo(Behaviour_t* caller, uint8_t servo, uint8_t pos) {
    Behaviour_t* data = NULL;
    if (!servo_get_active(servo) && (data = switch_to_behaviour(caller, bot_servo_behaviour, BEHAVIOUR_NOOVERRIDE))) { // Warte-Verhalten an
        LOG_DEBUG("bot_servo(): servo_set(%u, %u)", servo, pos);
        servo_set(servo, pos); // Servo-PWM einstellen
        servo_id = servo;
    } else {
        LOG_ERROR("bot_servo(%u, %u): Servo %u noch aktiv, Abbruch", servo, pos, servo_id);
    }
    return data;
}
#endif // BEHAVIOUR_SERVO_AVAILABLE
