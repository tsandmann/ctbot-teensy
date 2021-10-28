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
 * @file    behaviour_simple.c
 * @brief   Ganz einfache Beispielverhalten
 * @author  Benjamin Benz
 * @date    03.11.2006
 */

#include "behaviour_simple.h"

#ifdef BEHAVIOUR_SIMPLE_AVAILABLE

static uint8_t simple_state = 0; /**< Status des simple-Verhaltens */
static uint8_t simple2_state = 0; /**< Status des simple2-Verhaltens */
static int16_t simple2_light = 0; /**< Uebergabevariable fuer SIMPLE2 */


void bot_simple_behaviour(Behaviour_t* data) {
    static uint32_t last_ticks;

    switch (simple_state) {
        case 0: {
            bot_drive_distance(data, 0, BOT_SPEED_SLOW, 14);
            simple_state = 1;
            break;
        }

        case 1: {
            last_ticks = timer_get_tickCount32();
            simple_state = 2;
            break;
        }

        case 2: {
            if (timer_ms_passed_32(&last_ticks, 500)) {
                simple_state = 3;
            }
            break;
        }

        case 3: {
            bot_turn(data, 90);
            simple_state = 4;
            break;
        }

        case 4: {
            last_ticks = timer_get_tickCount32();
            simple_state = 5;
            break;
        }

        case 5: {
            if (timer_ms_passed_32(&last_ticks, 500)) {
                simple_state = 0;
            }
            break;
        }

        default: {
            LED_set(0);
            return_from_behaviour(data);
            break;
        }
    }
}

/**
 * Rufe das Simple-Verhalten auf
 * @param *caller Der obligatorische Verhaltensdatensatz des Aufrufers
 */
void bot_simple(Behaviour_t* caller) {
    switch_to_behaviour(caller, bot_simple_behaviour, BEHAVIOUR_OVERRIDE);
    simple_state = 0;
    LED_set(LED_ROT);
}


#define STATE_SIMPLE2_INIT 0
#define STATE_SIMPLE2_SEARCH 1
#define STATE_SIMPLE2_DRIVE 2
#define STATE_SIMPLE2_DONE 3

void bot_simple2_behaviour(Behaviour_t* data) {
    static int16_t max_light;
    static int16_t max_dir;
    static int16_t dir;

    int16_t light = (int16_t) ((sensLDRL + sensLDRR) / 2);
    int16_t free;

    switch (simple2_state) {
        case STATE_SIMPLE2_INIT: { // Initialisieren
            max_light = INT16_MAX;
            max_dir = 0;
            dir = 0;
            simple2_state = STATE_SIMPLE2_SEARCH;
            break;
        }

        case STATE_SIMPLE2_SEARCH: { // Einmal drehen und maximum suchen
            if (light < max_light) {
                max_dir = dir;
                max_light = light;
            }

            if (dir < 360) { // Noch nicht ganz rum?
                bot_turn(data, 10); // drehen
                dir = (int16_t) (dir + 10); // neue Position sichern
            } else { // wir sind ganz rum
                if (simple2_light == 0) {
                    bot_turn(data, max_dir); // zum Licht drehen
                } else {
                    bot_turn(data, (int16_t) (max_dir - 180)); // vom Licht wegdrehen
                }

                simple2_state = STATE_SIMPLE2_DRIVE; // Naechster Zustand
            }

            break;
        }

        case STATE_SIMPLE2_DRIVE: // vorwaerts fahren, bis Hinderniss
            free = (sensDistL < sensDistR) ? sensDistL : sensDistR;
            free = (free > SENS_IR_MAX_DIST) ? (int16_t) SENS_IR_MAX_DIST : free;

            if (free > SENS_IR_SAFE_DIST) {
                bot_goto_dist(data, (int16_t) (free - SENS_IR_SAFE_DIST), 1); // nach vorn
                simple2_state = STATE_SIMPLE2_INIT;
            } else
                simple2_state = STATE_SIMPLE2_DONE; // beenden
            break;

        default: {
            return_from_behaviour(data);
            break;
        }
    }
}

void bot_simple2(Behaviour_t* caller, int16_t light) {
    switch_to_behaviour(caller, bot_simple2_behaviour, BEHAVIOUR_OVERRIDE);
    simple2_light = light;
    simple2_state = 0;
}
#endif // BEHAVIOUR_SIMPLE_AVAILABLE
