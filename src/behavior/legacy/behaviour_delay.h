/*
 * ct-Bot
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
 * @file    behaviour_delay.h
 * @brief   Delay-Routinen als Verhalten
 * @author  Benjamin Benz
 * @date    12.07.2007
 */


#ifndef BEHAVIOUR_DELAY_H_
#define BEHAVIOUR_DELAY_H_

#include "bot-logic.h"

#ifdef BEHAVIOUR_DELAY_AVAILABLE

/**
 * Unterbreche das aktuelle Verhalten fuer mindestens X ms
 * Das Makro vereinfacht den Aufruf des Delay-Verhaltens, das funktioniert aber
 * nur von anderen Verhalten aus!
 * Die Verhaltensfunktion wird an der Stelle, wo BLOCK_BEHAVIOUR aufgerufen wird,
 * verlassen und erst nach Ablauf der Wartezeit wieder aufgerufen, diesmal wird der
 * BLOCK_BEHAVIOUR-Aufruf jedoch nicht ausgefuehrt.
 * Sollte das Delay-Verhalten bereits aktiv sein, wird gewartet, bis es fertig ist und
 * anschliessend gewartet.
 * Ein einfaches Beispiel fuer die Verwendung findet sich in bot_servo_behaviour().
 *
 * @param *_caller  Der obligatorische Verhaltensdatensatz des Aufrufers
 * @param _ms       Die Verzoegerungszeit in ms
 */
#define BLOCK_BEHAVIOUR(_caller, _ms)           \
    {                                           \
        static uint8_t _delay_state = 0;        \
        if (_delay_state == 0) {                \
            if (bot_delay(_caller, _ms) == 0) { \
                _delay_state = 1;               \
            }                                   \
            return;                             \
        }                                       \
        _delay_state = 0;                       \
    }

/**
 * Rufe das Delay-Verhalten auf
 * @param *caller   Der obligatorische Verhaltensdatensatz des Aufrufers
 * @param ms        Die Verzoegerungszeit in ms
 * @return          -1 wenn was schief gelaufen ist, sonst 0
 */
#define bot_delay(caller, ms) bot_delay_ticks(caller, (uint16_t) MS_TO_TICKS((uint32_t) ms))

/**
 * Rufe das Delay-Verhalten auf
 * @param *caller   Der obligatorische Verhaltensdatensatz des Aufrufers
 * @param ticks     Die Verzoegerungszeit in ticks
 * @return          -1 wenn was schief gelaufen ist, sonst 0
 */
FLASHMEM int8_t bot_delay_ticks(Behaviour_t* caller, uint16_t ticks);

/**
 * Verhalten fuer Delays
 * @param *data     Der Verhaltensdatensatz
 */
void bot_delay_behaviour(Behaviour_t* data);

#endif // BEHAVIOUR_DELAY_AVAILABLE
#endif // BEHAVIOUR_DELAY_H_
