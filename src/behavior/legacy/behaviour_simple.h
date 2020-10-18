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
 * @file    behaviour_simple.h
 * @brief   Ganz einfache Beispielverhalten
 * @author  Benjamin Benz (bbe@heise.de)
 * @date    03.11.2006
 */

#ifndef BEHAVIOUR_SIMPLE_H_
#define BEHAVIOUR_SIMPLE_H_

#include "bot-logic.h"

#ifdef BEHAVIOUR_SIMPLE_AVAILABLE
/**
 * Ein ganz einfaches Beispiel fuer ein Hilfsverhalten, das selbst SpeedWishes aussert und nach
 * getaner Arbeit die aufrufende Funktion wieder aktiviert Zudem prueft es, ob eine Uebergabebedingung erfuellt ist.
 * Zu diesem Verhalten gehoert die Botenfunktion bot_simple2()
 * bot_simple2_behaviour faehrt den Bot solange geradeaus, bis es dunkler als im Uebergabeparameter spezifiziert ist wird
 *
 * @param *data Der Verhaltensdatensatz
 */
void bot_simple2_behaviour(Behaviour_t* data);

/**
 * Rufe das Simple2-Verhalten auf und uebergebe light
 * @param *caller   Der obligatorische Verhaltensdatensatz des Aufrufers
 * @param light     Uebergabeparameter
 */
FLASHMEM void bot_simple2(Behaviour_t* caller, int16_t light);

/**
 * Ein ganz einfaches Verhalten, es hat maximale Prioritaet
 * @param *data Der Verhaltensdatensatz
 */
void bot_simple_behaviour(Behaviour_t* data);

/**
 * Rufe das Simple-Verhalten auf
 * @param caller Der obligatorische Verhaltensdatensatz des Aufrufers
 */
FLASHMEM void bot_simple(Behaviour_t* caller);

#endif // BEHAVIOUR_SIMPLE_AVAILABLE
#endif // BEHAVIOUR_SIMPLE_H_
