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
 * @file    behaviour_cancel_behaviour.h
 * @brief   Deaktiviert ein anderes Verhalten in Abhaengigkeit einer Check-Funktion
 * @author  Frank Menzel
 * @date    19.10.2007
 *
 * So kann z.B. der Wandfolger (bot_solve_maze) beendet werden, falls dieser auf
 * eine Linie faehrt und der Linienfolger uebernehmen.
 */

#ifndef BEHAVIOUR_CANCEL_H_
#define BEHAVIOUR_CANCEL_H_

#include "bot-logic.h"

#ifdef BEHAVIOUR_CANCEL_BEHAVIOUR_AVAILABLE

/**
 * Verhalten zum bedingten Deaktivieren eines anderen Verhaltens
 * @param *data Verhaltensdatensatz
 */
void bot_behaviour_cancel_behaviour(Behaviour_t* data);

/**
 * Botenfunktion zum Deaktivieren eines Verhaltens, wenn eine Abbruchbedingung erfuellt ist
 * @param *caller       Verhaltensdatensatz des Aufrufers
 * @param *behaviour    Verhaltensdatensatz des abzubrechenden Verhaltens
 * @param *check        Zeiger auf die Abbruchfunktion; liefert diese True, wird das Verhalten beendet
 * @return              Zeiger auf den eigenen Verhaltensdatensatz oder NULL im Fehlerfall
 */
Behaviour_t* bot_add_behaviour_to_cancel(Behaviour_t* caller, Behaviour_t* behaviour, uint8_t (*check)(void));

/**
 * Botenfunktion zum Deaktivieren eines Verhaltens, wenn die Abbruchbedingung erfuellt ist.
 * Alte Version, um Abwaertskompatibilitaet zu erhalten
 * @param *caller   Verhaltensdatensatz des Aufrufers
 * @param behaviour abzubrechendes Verhalten
 * @param *check    Zeiger auf die Abbruchfunktion; liefert diese True, wird das Verhalten beendet
 * @return          Zeiger auf den eigenen Verhaltensdatensatz oder NULL im Fehlerfall
 */
Behaviour_t* bot_cancel_behaviour(Behaviour_t* caller, BehaviourFunc_t behaviour, uint8_t (*check)(void));

#endif // BEHAVIOUR_CANCEL_BEHAVIOUR_AVAILABLE
#endif // BEHAVIOUR_CANCEL_H_
