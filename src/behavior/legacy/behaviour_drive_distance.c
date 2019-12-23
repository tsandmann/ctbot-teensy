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
 * @file   behaviour_drive_distance.c
 * @brief  Bot faehrt ein Stueck
 * @author Benjamin Benz (bbe@heise.de)
 * @date   03.11.2006
 */

#include "behaviour_drive_distance.h"

#if defined BEHAVIOUR_DRIVE_DISTANCE_AVAILABLE || defined BEHAVIOUR_OLYMPIC_AVAILABLE
/**
 * laesst den Bot in eine Richtung fahren.
 * Es handelt sich hierbei nicht im eigentlichen Sinn um ein Verhalten, sondern ist nur eine Abstraktion der Motorkontrollen.
 * @param curve Gibt an, ob der Bot eine Kurve fahren soll. Werte von -127 (So scharf wie moeglich links) ueber 0 (gerade aus) bis 127 (so scharf wie moeglich
 * rechts) \param speed Gibt an, wie schnell der Bot fahren soll. */
void bot_drive(int8_t curve, int16_t speed) {
    // Wenn etwas ausgewichen wurde, bricht das Verhalten hier ab, sonst wuerde es evtl. die Handlungsanweisungen von bot_avoid_harm() stoeren.
    // if(bot_avoid_harm()) return;
    if (curve < 0) {
        speedWishLeft = (int16_t)(speed * (1.f + 2.f * ((float) curve / 127.f)));
        speedWishRight = speed;
    } else if (curve > 0) {
        speedWishRight = (int16_t)(speed * (1.f - 2.f * ((float) curve / 127.f)));
        speedWishLeft = speed;
    } else {
        speedWishLeft = speed;
        speedWishRight = speed;
    }
}
#endif // BEHAVIOUR_DRIVE_DISTANCE_AVAILABLE || defined BEHAVIOUR_OLYMPIC_AVAILABLE


#ifdef BEHAVIOUR_DRIVE_DISTANCE_AVAILABLE

/* Parameter fuer das bot_drive_distance_behaviour() */
static int16_t drive_distance_target; /**< Zu fahrende Distanz bzw. angepeilter Stand der Radencoder sensEncL bzw. sensEncR */
static int8_t drive_distance_curve; /**< Kruemmung der zu fahrenden Strecke. */
static int16_t drive_distance_speed; /**< Angepeilte Geschwindigkeit. */

/**
 * Das Verhalten laesst den Bot eine vorher festgelegte Strecke fahren.
 * @param *data der Verhaltensdatensatz
 * @see bot_drive_distance()
 */
void bot_drive_distance_behaviour(Behaviour_t* data) {
    int16_t* encoder;
    int16_t to_drive;

    if (drive_distance_curve > 0) {
        // Es handelt sich um eine Rechtskurve, daher wird mit dem linken Encoder gerechnet
        encoder = &sensEncL;
    } else {
        encoder = &sensEncR;
    }

    to_drive = drive_distance_target - *encoder;
    if (drive_distance_speed < 0) {
        to_drive = -to_drive;
    }

    // LOG_DEBUG("bot_drive_distance_behaviour(): to_drive=%d", to_drive);

    if (to_drive <= 0) {
        LOG_DEBUG("bot_drive_distance_behaviour(): done.");
        return_from_behaviour(data);
    } else {
        if ((drive_distance_speed > BOT_SPEED_SLOW || drive_distance_speed < -BOT_SPEED_SLOW) && to_drive < (0.1f * ENCODER_MARKS)) {
            bot_drive(drive_distance_curve, drive_distance_speed / 2);
        } else {
            bot_drive(drive_distance_curve, drive_distance_speed);
        }
    }
}

/**
 * Das Verhalten laesst den Bot eine vorher festgelegte Strecke fahren. Dabei legt die Geschwindigkeit fest, ob der Bot vorwaerts oder rueckwaerts fahren soll.
 * @param *caller   Der Aufrufer
 * @param curve     Gibt an, ob der Bot eine Kurve fahren soll. Werte von -127 (so scharf wie moeglich links) ueber 0 (gerade aus) bis 127 (rechts)
 * @param speed     Gibt an, wie schnell der Bot fahren soll. Negative Werte lassen den Bot rueckwaerts fahren.
 * @param cm        Gibt an, wie weit der Bot fahren soll. In cm :-) Die Strecke muss positiv sein, die Fahrtrichtung wird ueber speed geregelt.
 * @return          Zeiger auf Verhaltensdatensatz
 */
Behaviour_t* bot_drive_distance(Behaviour_t* caller, int8_t curve, int16_t speed, int16_t cm) {
    LOG_DEBUG("curve= %d, speed= %d, cm =%d", curve, speed, cm);

    int32_t tmp = cm;
    tmp *= 10 * ENCODER_MARKS;
    tmp = (int32_t)((float) tmp / (float) WHEEL_PERIMETER);
    int16_t marks_to_drive = (int16_t) tmp;

    int16_t* encoder;
    drive_distance_curve = curve;
    drive_distance_speed = speed;

    if (curve > 0) {
        // Es handelt sich um eine Rechtskurve, daher wird mit dem linken Encoder gerechnet
        encoder = &sensEncL;
    } else {
        encoder = &sensEncR;
    }
    if (speed < 0) {
        // Es soll rueckwaerts gefahren werden. Der Zielwert ist also kleiner als der aktuelle Encoder-Stand.
        drive_distance_target = *encoder - marks_to_drive;
    } else {
        drive_distance_target = *encoder + marks_to_drive;
    }

    return switch_to_behaviour(caller, bot_drive_distance_behaviour, BEHAVIOUR_NOOVERRIDE);
}
#endif // BEHAVIOUR_DRIVE_DISTANCE_AVAILABLE
