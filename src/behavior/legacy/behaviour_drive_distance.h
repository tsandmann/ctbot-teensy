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
 * @file   behaviour_drive_distance.h
 * @brief  Bot faehrt ein Stueck
 * @author Benjamin Benz
 * @date   03.11.2006
 */

/**
 * bot_drive_distance() beruecksichtig keine waehrend der Fahrt aufgetretenen Fehler, daher ist die
 * Endposition nicht unbedingt auch die gewuenschte Position des Bots. Das komplexere Verhalten
 * bot_goto_pos() arbeitet hier deutlich genauer, darum werden jetzt alle bot_drive_distance()-Aufrufe
 * auf das goto_pos-Verhalten "umgeleitet", falls dieses vorhanden ist.
 * Moechte man das jedoch nicht und lieber weiterhin das alte drive_distance-Verhalten, deaktiviert
 * man den Schalter USE_GOTO_POS_DIST ein paar Zeilen unter diesem Text, indem man ihm // voranstellt.
 */

#ifndef BEHAVIOUR_DRIVE_DISTANCE_H_
#define BEHAVIOUR_DRIVE_DISTANCE_H_

#include "bot-logic.h"

#define USE_GOTO_POS_DIST /**< Ersetzt alle drive_distance()-Aufrufe mit dem goto_pos-Verhalten, falls vorhanden */

#ifndef BEHAVIOUR_GOTO_POS_AVAILABLE
#undef USE_GOTO_POS_DIST
#endif

#ifdef BEHAVIOUR_DRIVE_DISTANCE_AVAILABLE
#ifndef USE_GOTO_POS_DIST
/**
 * Das Verhalten laesst den Bot eine vorher festgelegte Strecke fahren.
 * \param *data der Verhaltensdatensatz
 * \see bot_drive_distance()
 */
void bot_drive_distance_behaviour(Behaviour_t* data);

/**
 * Das Verhalten laesst den Bot eine vorher festgelegte Strecke fahren. Dabei legt die Geschwindigkeit fest, ob der Bot vorwaerts oder rueckwaerts fahren soll.
 * \param curve Gibt an, ob der Bot eine Kurve fahren soll. Werte von -127 (So scharf wie moeglich links) ueber 0 (gerade aus) bis 127 (so scharf wie moeglich
 * rechts) \param speed Gibt an, wie schnell der Bot fahren soll. Negative Werte lassen den Bot rueckwaerts fahren. \param cm Gibt an, wie weit der Bot fahren
 * soll. In cm :-) Die Strecke muss positiv sein, die Fahrtrichtung wird ueber speed geregelt. \return Zeiger auf Verhaltensdatensatz
 */
FLASHMEM Behaviour_t* bot_drive_distance(Behaviour_t* caller, int8_t curve, int16_t speed, int16_t cm);

FLASHMEM static inline Behaviour_t* bot_goto_dist(Behaviour_t* caller, int16_t distance, int8_t dir) {
    return bot_drive_distance(caller, 0, dir < 0 ? -BOT_SPEED_NORMAL : BOT_SPEED_NORMAL, distance / 10);
}

#else // USE_GOTO_POS_DIST
/* wenn goto_pos() vorhanden ist und USE_GOTO_POS_DIST an, leiten wir alle drive_distance()-Aufurfe dorthin um */
#undef BEHAVIOUR_DRIVE_DISTANCE_AVAILABLE

FLASHMEM static Behaviour_t* bot_goto_dist(Behaviour_t* caller, int16_t distance, int8_t dir);

FLASHMEM static inline Behaviour_t* bot_drive_distance(Behaviour_t* caller, int8_t curve __attribute__((unused)), const int16_t speed, const int16_t cm) {
    return bot_goto_dist(caller, (int16_t) (cm * 10), (int8_t) (speed < 0 ? -1 : 1));
}
#endif // USE_GOTO_POS_DIST
#endif // BEHAVIOUR_DRIVE_DISTANCE_AVAILABLE

#if defined BEHAVIOUR_DRIVE_DISTANCE_AVAILABLE || defined BEHAVIOUR_OLYMPIC_AVAILABLE
/**
 * laesst den Bot in eine Richtung fahren.
 * Es handelt sich hierbei nicht im eigentlichen Sinn um ein Verhalten, sondern ist nur eine Abstraktion der Motorkontrollen.
 * \param curve Gibt an, ob der Bot eine Kurve fahren soll. Werte von -127 (So scharf wie moeglich links) ueber 0 (gerade aus) bis 127 (so scharf wie moeglich
 * rechts) \param speed Gibt an, wie schnell der Bot fahren soll. */
FLASHMEM void bot_drive(int8_t curve, int16_t speed);
#endif // BEHAVIOUR_DRIVE_DISTANCE_AVAILABLE || BEHAVIOUR_OLYMPIC_AVAILABLE
#endif // BEHAVIOUR_DRIVE_DISTANCE_H_
