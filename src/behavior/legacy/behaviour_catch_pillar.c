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
 * \file 	behaviour_catch_pillar.c
 * \brief 	Sucht nach einer Dose und faengt sie ein
 * \author 	Benjamin Benz
 * \date 	08.12.2006
 */

#include "behaviour_catch_pillar.h"

#ifdef BEHAVIOUR_CATCH_PILLAR_AVAILABLE
#include <math.h>
#include <stdlib.h>

#define CATCH_PILLAR_VERSION                                                                                                                                   \
    3 /**< Version 1: Altes Verfahren; Version 2: Ermittlung der Objektkoordinaten mit measure_distance(); Version 3: Ermittlung der Objektkoordinaten aus dem \
         Drehwinkel */
#define OBJECT_WIDTH 30
#define BEAM_WIDTH 4.f

#define DEBUG_CATCH_PILLAR

#ifdef DEBUG_CATCH_PILLAR
#pragma GCC diagnostic ignored "-Wdouble-promotion"
#else
#undef LOG_DEBUG
#define LOG_DEBUG(...) \
    {}
#endif // DEBUG_CATCH_PILLAR

#define START 0
#define SEARCH_LEFT 1
#define LEFT_FOUND 2
#define TURN_MIDDLE 3
#define GO 4
#define END 99

// Zustaende fuer das Ausladeverhalten START und END sind bereits weiter oben definiert
#define GO_BACK 21
#define CLOSE_DOOR 22

static uint8_t catch_pillar_state = START; /**< Statusvariable fuer das Einfang-Verhalten */
static uint8_t unload_pillar_state = START; /**< Statusvariable fuer das Auslade-Verhalten */

#define SERVO1 1
#define M_PI_F ((float) M_PI)

#ifndef BEHAVIOUR_GOTO_POS_AVAILABLE
#undef CATCH_PILLAR_VERSION
#define CATCH_PILLAR_VERSION 1
#endif

#ifdef PC
#undef BEAM_WIDTH
#define BEAM_WIDTH 1.5f
#endif

#if CATCH_PILLAR_VERSION == 1

static int16_t startangle = 0; /**< gemerkter Anfangswinkel einfach als Integer */
/**
 * Fange eine Dose ein
 * \param *data der Verhaltensdatensatz
 */
void bot_catch_pillar_behaviour(Behaviour_t* data) {
    static uint8_t cancelcheck = False;
    static float angle;

    switch (catch_pillar_state) {
        case START:
            startangle = heading_int;
            cancelcheck = False;
            catch_pillar_state = SEARCH_LEFT;
            break;

        case SEARCH_LEFT:
            // Nach 1x Rundumsuche und nix gefunden ist Schluss; Dazu wird der Check zum Start
            // nach Ueberschreiten der 5Grad-Toleranz eingeschaltet und die Drehung dann beendet wenn
            // der Winkel wieder innerhalb dieser Toleranz liegt
            if (cancelcheck) {
                if (fabs(heading - startangle) < 5)
                    // Damit nicht endlos rundum gedreht wird Abbruch, wenn kein Gegenstand gefunden
                    catch_pillar_state = END;
            } else {
                if (fabs(heading - startangle) >= 5)
                    cancelcheck = True;
            }

            // linker Sensor muss was sehen, der rechte aber nicht
            if (sensDistL < MAX_PILLAR_DISTANCE && sensDistR > MAX_PILLAR_DISTANCE) { // sieht der linke Sensor schon was?
                angle = heading;
                catch_pillar_state = LEFT_FOUND;
            } else {
                speedWishLeft = -BOT_SPEED_SLOW;
                speedWishRight = +BOT_SPEED_SLOW;
                // bot_turn(data,5);	// Ein Stueck drehen
            }
            break;

        case LEFT_FOUND:
            // links und rechts darf nicht gleichzeitig was gesehen werden, sonst weiter mit drehen
            if (sensDistL < MAX_PILLAR_DISTANCE && sensDistR < MAX_PILLAR_DISTANCE) {
                catch_pillar_state = SEARCH_LEFT;
                break;
            }

            if (sensDistR < MAX_PILLAR_DISTANCE) { // sieht der rechte Sensor schon was?
                angle = heading - angle;
                if (angle < 0)
                    angle += 360;
                angle = heading - angle / 2;
                catch_pillar_state = TURN_MIDDLE;
                //				bot_turn(data,-angle/2);
            } else {
                speedWishLeft = -BOT_SPEED_SLOW;
                speedWishRight = +BOT_SPEED_SLOW;
                //				bot_turn(data,5); // Eins Stueck drehen
            }
            break;

        case TURN_MIDDLE:
            if (fabs(heading - angle) > 2) {
                speedWishLeft = +BOT_SPEED_SLOW;
                speedWishRight = -BOT_SPEED_SLOW;
            } else {
                bot_servo(data, SERVO1, DOOR_OPEN); // Klappe auf
                catch_pillar_state = GO;
            }
            break;

        case GO:
            if (sensTrans == 0) {
                speedWishLeft = +BOT_SPEED_SLOW;
                speedWishRight = +BOT_SPEED_SLOW;
                // nicht endlos vorwaertslaufen, weiter mit Suche bis Drehende
                if (sensDistL < MAX_PILLAR_DISTANCE && sensDistR < MAX_PILLAR_DISTANCE)
                    catch_pillar_state = SEARCH_LEFT;
            } else {
                bot_servo(data, SERVO1, DOOR_CLOSE);
                catch_pillar_state = END;
            }

            break;

        default:
            catch_pillar_state = START;
            return_from_behaviour(data);
            break;
    }
}

/**
 * Fange ein Objekt ein
 * \param caller Der obligatorische Verhaltensdatensatz des Aufrufers
 */
void bot_catch_pillar(Behaviour_t* caller) {
    catch_pillar_state = START;
    unload_pillar_state = END; // Sicherheitshalber das Unloade-Verhalten auf Ende setzen
    // Zielwerte speichern
    switch_to_behaviour(caller, bot_catch_pillar_behaviour, BEHAVIOUR_OVERRIDE);
}

#elif CATCH_PILLAR_VERSION == 2

static int16_t start_heading; /**< gemerkter Winkel bei Start, der damit auch Zielwinkel ist zur Differenzbildung */
static int8_t check_sensorside; /** Seite, auf der der Sensorabstand gecheckt wird */
static position_t obj_posL; /**< berechnete Position des Objekts (links) */
static position_t obj_posR; /**< berechnete Position des Objekts (rechts) */
static uint8_t state_after_cancel; /**< Status, der nach einem Abbruch eingestellt wird */

#define MIN_DIST_SENSES 90 /**< groesster zulaessiger Abstand in mm zwischen beiden Sensoren */

#define OBJECT_FOUND_AFTER_CANCEL 3
#define MEASURE_DIST 5
#define OBJECT_FOUND_AFTER_MEASURE 6
#define TURN_BACK 7
#define OPEN_DOOR 8
#define GO_FORWARD 9
#define GO_TO_POINT 10
#define TURN_TO_RIGHT_SENSOR 11
#define GO_FOR_DROP 12
#define OBJECT_INVALID 13

/**
 * liefert True wenn Objekt nicht gueltig ist zum Einfangen, z.B. Wand
 * \return	True wenn Objekt ungueltig, sonst False
 */
static uint8_t object_invalid(void) {
    if ((sensDistL < MAX_PILLAR_DISTANCE && sensDistR < MAX_PILLAR_DISTANCE) || abs(sensDistL - sensDistR) < MIN_DIST_SENSES) {
        return True;
    }

    return False;
}

/**
 * Abbruchfunktion fuer das Cancelverhalten waehrend der Drehung zum
 * Erkennen eines Einfangobjektes. Anschliessend geht es mit
 * state_after_cancel weiter.
 * \return	True wenn gueltiges Objekt erkannt wurde, sonst False
 */
static uint8_t turn_cancel_check(void) {
    if (object_invalid()) {
        return False; // beide zu klein gilt als ungueltiges Objekt
    }

    if (check_sensorside == -1) {
        /* links */
        if (sensDistL <= MAX_PILLAR_DISTANCE) {
            catch_pillar_state = state_after_cancel;
            return True;
        }
    } else {
        /* rechts */
        if (sensDistR <= MAX_PILLAR_DISTANCE) {
            catch_pillar_state = state_after_cancel;
            return True;
        }
    }

    return False;
}

/**
 * Fange ein Objekt ein
 * \param *data	Der Verhaltensdatensatz
 */
void bot_catch_pillar_behaviour(Behaviour_t* data) {
    static int16_t distLeft, distRight;

    switch (catch_pillar_state) {
        case START:
            bot_turn_maxspeed(data, 360, BOT_SPEED_SLOW);
            bot_cancel_behaviour(data, bot_turn_behaviour, turn_cancel_check);
            state_after_cancel = OBJECT_FOUND_AFTER_CANCEL;
            catch_pillar_state = END;
            break;

        case OBJECT_FOUND_AFTER_CANCEL: // Objekt erkannt
            // kommt hierher weil Cancel zugeschlagen hatte und Objekt erkannt wurde
            // ist schon zu weit gedreht, wieder etwas zurueckdrehen sonst genau Abstand messen

            // sehen beide Sensoren zu kleinen Abstand, dann ist dies wohl eine Wand und es geht weiter mit Drehen
            if (object_invalid()) {
                catch_pillar_state = OBJECT_INVALID;
                LOG_DEBUG("OBJECT_INVALID nach OBJECT_FOUND_AFTER_CANCEL");
                break;
            }

            // bei Gueltigkeit evtl. Korrektur notwendig
            catch_pillar_state = TURN_BACK;
            break;

        case TURN_BACK:
            BLOCK_BEHAVIOUR(data, 2000); // Nachlauf abwarten
            if (turn_cancel_check()) {
                // Falls Sensor Objekt erkennt, genauen Abstand messen
                catch_pillar_state = MEASURE_DIST;
                break;
            }
            bot_turn_maxspeed(data, -20, BOT_SPEED_SLOW);
            bot_cancel_behaviour(data, bot_turn_behaviour, turn_cancel_check);
            state_after_cancel = OBJECT_FOUND_AFTER_CANCEL;
            catch_pillar_state = OBJECT_INVALID;
            break;

        case OBJECT_INVALID: {
            LOG_DEBUG("OBJECT_INVALID");
            int16_t turned = (int16_t) heading - start_heading;
            if (turned < 0)
                turned += 360;
            bot_turn_maxspeed(data, 360 - turned, BOT_SPEED_SLOW);
            bot_cancel_behaviour(data, bot_turn_behaviour, turn_cancel_check);
            state_after_cancel = OBJECT_FOUND_AFTER_CANCEL;
            catch_pillar_state = END;
            break;
        }

        case MEASURE_DIST:
            // genaue Entfernung zum Hindernis messen
            bot_measure_distance(data, &distLeft, &distRight, 15);
            catch_pillar_state = OPEN_DOOR;
            break;

        case OPEN_DOOR:
            if (distLeft > MAX_PILLAR_DISTANCE && distRight > MAX_PILLAR_DISTANCE) {
                // bei ungueltigem Abstand nach Messung Restdrehung
                catch_pillar_state = OBJECT_INVALID;
                LOG_DEBUG("OBJECT_INVALID nach OPEN_DOOR");
                break;
            }
            catch_pillar_state = OBJECT_FOUND_AFTER_MEASURE;
            break;

        case OBJECT_FOUND_AFTER_MEASURE:
            // nehme den Sensorabstand vom Sensor, welcher auch Objekt sieht
            if (distLeft <= MAX_PILLAR_DISTANCE) {
                obj_posL = calc_point_in_distance(heading, DISTSENSOR_POS_FW + distLeft, DISTSENSOR_POS_SW);
                catch_pillar_state = TURN_TO_RIGHT_SENSOR;
            } else {
                obj_posR = calc_point_in_distance(heading, DISTSENSOR_POS_FW + distRight, -DISTSENSOR_POS_SW);
                catch_pillar_state = GO_TO_POINT;
                bot_servo(data, SERVO1, DOOR_OPEN); // Klappe auf
            }
            break;

        case TURN_TO_RIGHT_SENSOR:
            check_sensorside = 1;
            catch_pillar_state = START;
            break;

        case GO_TO_POINT:
            bot_goto_pos(data, ((obj_posL.x + obj_posR.x) / 2), ((obj_posL.y + obj_posR.y) / 2), 999);
            catch_pillar_state = CLOSE_DOOR;
            break;

        case CLOSE_DOOR:
            if (sensTrans == 1) {
                // Klappe schliessen falls Objekt eingefangen wurde
                bot_servo(data, SERVO1, DOOR_CLOSE);
            }
            catch_pillar_state = END;
            break;

        default:
            exit_behaviour(data, sensTrans); // == BEHAVIOUR_SUBSUCCESS, falls Objekt eingefangen
            break;
    }
}

/**
 * Fange ein Objekt ein
 * \param *caller	Der obligatorische Verhaltensdatensatz des Aufrufers
 */
void bot_catch_pillar(Behaviour_t* caller) {
    switch_to_behaviour(caller, bot_catch_pillar_behaviour, BEHAVIOUR_OVERRIDE);
    catch_pillar_state = START;
    check_sensorside = -1; // linker Sensor ergibt Abbruchbedingung
    start_heading = heading;
    obj_posL.x = 0;
    obj_posL.y = 0;
    obj_posR.x = 0;
    obj_posR.y = 0;
    /* Kollisions-Verhalten ausschalten  */
#ifdef BEHAVIOUR_AVOID_COL_AVAILABLE
    deactivateBehaviour(bot_avoid_col_behaviour);
#endif
}

#elif CATCH_PILLAR_VERSION == 3

#define OBJECT_FOUND 5
#define OPEN_DOOR 6
#define GO_TO_POINT 7
#define TURN 8

static uint8_t state_after_cancel; /**< Status, der nach einem Abbruch eingestellt wird */
static uint8_t side; /**< Auswahl des Distanzsensors (0: links, 1: rechts) */
static float headingL; /**< heading bei Erkennung links [Grad] */
static position_t posL;
static float headingR; /**< heading bei Erkennung rechts [Grad] */
static position_t posR;
static int16_t max_turn; /**< Wie weit [Grad] maximal gedreht werden soll */
/** Modus zur Berechnung der Objektposition: 0: Drehwinkel, 1: linker Distanzsensor, 2: rechter Distanzsensor,
 * 3: Mittelwert linker und rechter Distanzsensor */
static uint8_t dist_mode;

/**
 * Abbruchfunktion fuer das Cancelverhalten waehrend der Drehung zum
 * Erkennen eines Einfangobjektes. Anschliessend geht es mit
 * state_after_cancel weiter.
 * Wurde das Objekt links erkannt, wird erst abgebrochen, nachdem
 * es auch rechts erkannt wurde.
 * \return	True wenn gueltiges Objekt erkannt wurde, sonst False
 */
static uint8_t turn_cancel_check(void) {
    switch (side) {
        case 0:
            /* Check mit linkem Sensor */
            if (sensDistL <= MAX_PILLAR_DISTANCE) {
                headingL = fmodf(heading + BEAM_WIDTH, 360.f);
                posL = calc_point_in_distance(headingL, DISTSENSOR_POS_FW - 22 + 10 + sensDistL, DISTSENSOR_POS_SW + OBJECT_WIDTH / 2);
                side = 1;
                LOG_DEBUG("(%4d|%4d): Objekt links erkannt:", x_pos, y_pos);
                LOG_DEBUG(" sensDistL=%d headingL=%.2f posL=(%4d|%4d)", sensDistL, headingL, posL.x, posL.y);
                return False;
            }
            break;
        case 1:
            /* Check mit rechtem Sensor */
            if (sensDistR <= MAX_PILLAR_DISTANCE) {
                catch_pillar_state = state_after_cancel;
                headingR = fmodf(heading + BEAM_WIDTH, 360.f);
                posR = calc_point_in_distance(headingR, DISTSENSOR_POS_FW - 22 + 10 + sensDistR, -DISTSENSOR_POS_SW + OBJECT_WIDTH / 2);
                LOG_DEBUG("(%4d|%4d): Objekt rechts erkannt:", x_pos, y_pos);
                LOG_DEBUG(" sensDistR=%d headingR=%.2f posR=(%4d|%4d)", sensDistR, headingR, posR.x, posR.y);
                return True;
            }
            break;
    }

    return False;
}

/**
 * Bricht goto_pos() ab, sobald das Objekt im Fach erkannt wird.
 * \return	True, falls Transportfach voll, sonst False
 */
static uint8_t goto_pos_cancel(void) {
    if (sensTrans) {
        LOG_DEBUG("Objekt im Fach erkannt");
    }
    return sensTrans;
}

/**
 * Fange ein Objekt ein
 * \param *data	Der Verhaltensdatensatz
 */
void bot_catch_pillar_behaviour(Behaviour_t* data) {
    static position_t obj_pos;
    float dHead = 0.f;

    switch (catch_pillar_state) {
        /* Auf los geht's los */
        case START:
            /* Drehen mit Abbruch bei Objekterkennung */
            LOG_DEBUG("Starte Drehung um max. %d Grad", max_turn);
            bot_turn_maxspeed(data, max_turn, BOT_SPEED_MIN);
            bot_cancel_behaviour(data, bot_turn_behaviour, turn_cancel_check);
            side = 0;
            headingL = -1.f;
            headingR = -1.f;
            state_after_cancel = OBJECT_FOUND;
            catch_pillar_state = END;
            LOG_DEBUG("Pausiere catch_pillar() Verhalten");
            break;

            /* Objekt erkannt */
        case OBJECT_FOUND:
            LOG_DEBUG("catch_pillar() wurde reaktiviert");
            if (headingL < 0 || headingR < 0) {
                /* Erkennung fehlgeschlagen */
                catch_pillar_state = END;
                LOG_DEBUG("Erkennung fehlgeschlagen: headingL=%.2f headingR=%.2f", headingL, headingR);
                return;
            }
            catch_pillar_state = TURN;

            if (dist_mode == 0) {
                /* Abstand zum Objekt berechnen aus gedrehtem Winkel */
                dHead = headingR - headingL;
                if (dHead < 0.f) {
                    dHead += 360.f;
                }
                LOG_DEBUG("dHead=%.2f", dHead);
                int16_t dist = (int16_t) ((DISTSENSOR_POS_SW * 2.f) / dHead * (180.f / M_PI_F));
                /* Objektkoordis berechnen */
                LOG_DEBUG("heading=%.2f headingL=%.2f headingR=%.2f", heading, headingL, headingR);
                if (dist > MAX_PILLAR_DISTANCE + 100) {
                    LOG_DEBUG("Objekt erkannt, aber Distanz zu gross");
                    catch_pillar_state = END;
                }

                obj_pos = calc_point_in_distance(headingL, DISTSENSOR_POS_FW - 22 + 10 + dist, DISTSENSOR_POS_SW + OBJECT_WIDTH / 2);
                LOG_DEBUG("Objekt links erkannt, dist=%d obj_pos=(%4d|%4d)", dist, obj_pos.x, obj_pos.y);
                obj_pos = calc_point_in_distance(headingR, DISTSENSOR_POS_FW - 22 + 10 + dist, -DISTSENSOR_POS_SW + OBJECT_WIDTH / 2);
                LOG_DEBUG("Objekt rechts erkannt, dist=%d obj_pos=(%4d|%4d)", dist, obj_pos.x, obj_pos.y);
            } else if (dist_mode == 1) {
                obj_pos = posL;
                LOG_DEBUG("Objekt erkannt (linker Sensor), obj_pos=(%4d|%4d)", obj_pos.x, obj_pos.y);
            } else if (dist_mode == 2) {
                obj_pos = posR;
                LOG_DEBUG("Objekt erkannt (rechter Sensor), obj_pos=(%4d|%4d)", obj_pos.x, obj_pos.y);
            } else if (dist_mode == 3) {
                const int16_t dx = posL.x - posR.x;
                obj_pos.x = posL.x - dx / 2;
                const int16_t dy = posL.y - posR.y;
                obj_pos.y = posL.y - dy / 2;
                LOG_DEBUG("Objekt erkannt (Mittelwert), obj_pos=(%4d|%4d)", obj_pos.x, obj_pos.y);
            }
            break;

        case TURN:
            // bot_goto_pos_rel(data, 0, 0, fmodf(headingR - 2.f, 360.f));
            catch_pillar_state = OPEN_DOOR;
            break;

            /* Klappe auf */
        case OPEN_DOOR:
            catch_pillar_state = GO_TO_POINT;
            bot_servo(data, SERVO1, DOOR_OPEN);
            LOG_DEBUG("Oeffne Klappe...");
            break;

            /* zum Objekt fahren und Stopp, wenn das Objekt im Fach erkannt wird */
        case GO_TO_POINT:
            LOG_DEBUG("Fahre zu Punkt (%4d|%4d) ...", obj_pos.x, obj_pos.y);
            bot_goto_pos(data, obj_pos.x, obj_pos.y, 999);
            bot_cancel_behaviour(data, bot_goto_pos_behaviour, goto_pos_cancel);
            catch_pillar_state = CLOSE_DOOR;
            break;

            /* Klappe zu */
        case CLOSE_DOOR:
            LOG_DEBUG("Ziel erreicht: (%4d|%4d)", x_pos, y_pos);
            if (sensTrans == 1) {
                // Klappe schliessen falls Objekt eingefangen wurde
                bot_servo(data, SERVO1, DOOR_CLOSE);
                LOG_DEBUG("Schliesse Klappe...");
            }
            catch_pillar_state = END;
            break;

            /* Ende */
        default:
            exit_behaviour(data, sensTrans); // == BEHAVIOUR_SUBSUCCESS, falls Objekt eingefangen
            LOG_DEBUG("catch_pillar() wird beendet");
            break;
    }
}

/**
 * Fange ein Objekt ein
 * \param *caller	Der obligatorische Verhaltensdatensatz des Aufrufers
 * \param degrees	Wie weit [Grad] soll maximal gedreht werden?
 */
void bot_catch_pillar_turn(Behaviour_t* caller, int16_t degrees) {
    LOG_DEBUG("bot_catch_pillar_turn(%d)", degrees);
    LOG_DEBUG("Startpunkt: (%4d|%4d)", x_pos, y_pos);
    LOG_DEBUG("Modus: %u", dist_mode);
    switch_to_behaviour(caller, bot_catch_pillar_behaviour, BEHAVIOUR_OVERRIDE);
    catch_pillar_state = START;
    max_turn = degrees;
    /* Kollisions-Verhalten ausschalten */
#ifdef BEHAVIOUR_AVOID_COL_AVAILABLE
    deactivateBehaviour(bot_avoid_col_behaviour);
#endif
}

/**
 * Fange ein Objekt ein
 * \param *caller	Der obligatorische Verhaltensdatensatz des Aufrufers
 * \param mode		Modus zur Berechnung der Objektposition: 0: Drehwinkel, 1: linker Distanzsensor, 2: rechter Distanzsensor,
 * 					3: Durchschnitt linker und rechter Distanzsensor
 */
void bot_catch_pillar(Behaviour_t* caller, uint8_t mode) {
    dist_mode = mode < 4 ? mode : 0;
    bot_catch_pillar_turn(caller, 360);
}

#endif // CATCH_PILLAR_VERSION

/**
 * Gibt die Dose wieder aus, Entladevorgang
 * \param *data der Verhaltensdatensatz
 */
void bot_unload_pillar_behaviour(Behaviour_t* data) {
    switch (unload_pillar_state) {
        case START:
            if (sensTrans == 1) { // ist was im Bauch gehts los
                unload_pillar_state = GO_BACK;
                // Klappe auf und danach Rueckwaerts
                bot_servo(data, SERVO1, DOOR_OPEN);
            } else
                // nix zu tun
                unload_pillar_state = END;
            break;
        case GO_BACK:
            bot_drive_distance(data, 0, -BOT_SPEED_FOLLOW, 10);
            // 10 cm rueckwaerts nur bei Hindernis
            unload_pillar_state = CLOSE_DOOR;
            break;
        case CLOSE_DOOR:
            bot_servo(data, SERVO1, DOOR_CLOSE);
            unload_pillar_state = END;
            break;
        default:
            unload_pillar_state = START; // bei Umschaltung via Verhaltensscreen ist dies notwendig
            return_from_behaviour(data);
            break;
    }
}

/**
 * Entlaedt das Objekt wieder
 * \param caller Der obligatorische Verhaltensdatensatz des Aufrufers
 */
void bot_unload_pillar(Behaviour_t* caller) {
    unload_pillar_state = START;
    catch_pillar_state = END; // Sicherheitshalber das catch-Verhalten auf Ende setzen
    switch_to_behaviour(caller, bot_unload_pillar_behaviour, BEHAVIOUR_OVERRIDE);
}

#endif // BEHAVIOUR_CATCH_PILLAR_AVAILABLE
