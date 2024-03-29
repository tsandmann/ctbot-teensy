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
 * @file    behaviour_follow_line.c
 * @brief   Linienverfolger
 * @author  Torsten Evers Version 1
 * @author  Timo Sandmann Version 2
 * @author  Frank Menzel Version 3
 * @date    21.09.2007
 */

#include "behaviour_follow_line.h"

#ifdef BEHAVIOUR_FOLLOW_LINE_AVAILABLE

#define DEBUG_BEHAVIOUR_FOLLOW_LINE // Schalter fuer Debug-Code

#ifndef DEBUG_BEHAVIOUR_FOLLOW_LINE
#undef LOG_DEBUG
#define LOG_DEBUG(...)
#endif


#if FOLLOW_LINE_VERSION == 1 // urspruenglich altes Linienverhalten

/* Konstanten fuer das Verhalten */
#define CORNER_LEFT 1
#define CORNER_RIGHT 2
/* Zustaende fuer das Verhalten */
#define CHECK_LINE 0 /**< Feststellen ob wir ueber einer Linie sind */
#define FOLLOW_LINE 1 /**< Folgen einer geraden oder leicht gekruemmten Linie */
#define CHECK_BORDER 2 /**< Abgrundsensoren haben Alarm geschlagen. Feststellen ob wirklich Abgrund oder Ecke */
#define CORNER_TURN 3 /**< Drehung in Richtun detektiertem Abgrund */
#define CORRECT_POS 4 /**< Nach der Drehung 1cm vorfahren zum Ausgleichen */
#define ADVANCE_CORNER 5 /**< Auf die Ecke zufahren, bis die Linie verschwindet */
#define RETREAT_AND_STOP 6 /**< Zurueckfahren mit voller Geschwindigkeit, dann Stop und Verhalten verlassen */

/* Status- und Hilfsvariablen */
static int8_t lineState = CHECK_LINE;
static int8_t cornerDetected = False;

void bot_follow_line_behaviour(Behaviour_t* data) {
    switch (lineState) {
        case CHECK_LINE: /* sind beide Sensoren ueber einer Linie? */
            if (sensLineL >= LINE_SENSE && sensLineR >= LINE_SENSE) {
                /* zunaechst alle Hilfsverhalten ausschalten, die den Algorithmus stoeren */
                /* Abgrund- und Kollisions-Verhalten ausschalten */
#ifdef BEHAVIOUR_AVOID_COL_AVAILABLE
                deactivateBehaviour(bot_avoid_col_behaviour);
#endif
#ifdef BEHAVIOUR_AVOID_BORDER_AVAILABLE
                deactivateBehaviour(bot_avoid_border_behaviour);
#endif
                /* bot_glance() stoert bot_turn() */
                // deactivateBehaviour(bot_glance_behaviour);
                /* losfahren und nach FOLLOW_LINE wechseln */
                speedWishLeft = BOT_SPEED_FOLLOW;
                speedWishRight = BOT_SPEED_FOLLOW;
                lineState = FOLLOW_LINE;
            }
            break;

        case FOLLOW_LINE:
            /* Pruefen, ob die Abgrundsensoren einen Abgrund sehen */
            if (sensBorderL > BORDER_DANGEROUS || sensBorderR > BORDER_DANGEROUS) {
                /* Abgrund erkannt, das kann jetzt eine Linie sein oder ein richtiger Abgrund.*/
                if (sensBorderL > BORDER_DANGEROUS && sensBorderR > BORDER_DANGEROUS) {
                    /* Wenn beidseitig erkannt, koennen wir damit nicht umgehen ->
                     * Ende des Verhaltens */
                    speedWishLeft = BOT_SPEED_STOP;
                    speedWishRight = BOT_SPEED_STOP;
                    // LOG_INFO("Stopp in FOLLOW_LINE");
                    return_from_behaviour(data);
                    break;
                }
                /* nachsehen, ob der linke oder rechte Liniensensor ohne Kontakt zur Linie ist
                 * und ggfs. gegensteuern */
                if (sensBorderL > BORDER_DANGEROUS) {
                    cornerDetected = CORNER_LEFT;
                } else {
                    cornerDetected = CORNER_RIGHT;
                }
                /* nun zur vermuteten Ecke vorfahren */
                lineState = CHECK_BORDER;
                bot_drive_distance(data, 0, BOT_SPEED_FOLLOW, 3);
                break;
            }
            if (sensLineL < LINE_SENSE && sensLineR > LINE_SENSE) {
                /* links von der Linie abgekommen, daher nach rechts drehen */
                // LOG_DEBUG("Drehe rechts");
                speedWishLeft = BOT_SPEED_FOLLOW;
                speedWishRight = -BOT_SPEED_FOLLOW;
            } else if (sensLineL > LINE_SENSE && sensLineR < LINE_SENSE) {
                /* andersrum, also links drehen */
                // LOG_DEBUG("Drehe links");
                speedWishLeft = -BOT_SPEED_FOLLOW;
                speedWishRight = BOT_SPEED_FOLLOW;
            } else if (sensLineL > LINE_SENSE && sensLineR > LINE_SENSE) {
                /* noch ueber der Linie, also einfach geradeaus weiter */
                // LOG_DEBUG("Fahre geradeaus");
                speedWishLeft = BOT_SPEED_FOLLOW;
                speedWishRight = BOT_SPEED_FOLLOW;
            }
            break;

        case CHECK_BORDER:
            /* wir sollten jetzt direkt an der Kante zum Abgrund stehen, wenn es
             * denn wirklich eine ist. In dem Fall fahren wir ein Stueck zurueck.
             * sonst gehen wir von einer Linie aus, drehen uns in die Richtung,
             * in der wir den "Abgrund" entdeckt haben und machen dann weiter mit
             * der Linienverfolgung */
            if (sensBorderL > BORDER_DANGEROUS || sensBorderR > BORDER_DANGEROUS) {
                /* scheint wirklich ein Abgrund zu sein */
                lineState = RETREAT_AND_STOP;
                speedWishLeft = -BOT_SPEED_MAX;
                speedWishRight = -BOT_SPEED_MAX;
                break;
            }
            /* war nur eine Ecke, noch weiter darauf zu bis kein Kontakt mehr zur Linie */
            lineState = ADVANCE_CORNER;
            speedWishLeft = BOT_SPEED_FOLLOW;
            speedWishRight = BOT_SPEED_FOLLOW;
            break;

        case ADVANCE_CORNER:
            /* auf die Ecke zufahren, bis die Linie verschwindet */
            if (sensLineL < LINE_SENSE && sensLineR < LINE_SENSE) {
                /* Linie weg, also Stop, kurz zurueck und drehen */
                lineState = CORNER_TURN;
                speedWishLeft = -BOT_SPEED_SLOW;
                speedWishRight = -BOT_SPEED_SLOW;
                break;
            }
            speedWishLeft = BOT_SPEED_FOLLOW;
            speedWishRight = BOT_SPEED_FOLLOW;
            break;

        case CORNER_TURN:
            /* 90 Grad in Richtung des detektierten Abgrunds drehen */
            lineState = CORRECT_POS;
            bot_turn(data, (cornerDetected == CORNER_LEFT) ? 90 : -90);
            cornerDetected = False;
            break;

        case CORRECT_POS:
            lineState = FOLLOW_LINE;
            bot_drive_distance(data, 0, BOT_SPEED_SLOW, 2);
            break;

        case RETREAT_AND_STOP:
            /* wir sind an einem Abgrund, Stop und Ende des Verhaltens */
            speedWishLeft = BOT_SPEED_STOP;
            speedWishRight = BOT_SPEED_STOP;
            return_from_behaviour(data);
            break;
    }
}

void bot_follow_line(Behaviour_t* caller, uint8_t dummy __attribute__((unused))) {
    switch_to_behaviour(caller, bot_follow_line_behaviour, BEHAVIOUR_NOOVERRIDE);
    lineState = CHECK_LINE;
    cornerDetected = False;
}

#elif FOLLOW_LINE_VERSION == 2 // neuere Version des Linienfolgers

#ifndef BEHAVIOUR_AVOID_COL_AVAILABLE
#warning "BEHAVIOUR_AVOID_COL_AVAILABLE sollte aktiv sein fuer FOLLOW_LINE_VERSION == 2"
#endif

static uint8_t state; /**< Status des Verhaltens (0: init, 1: Linie suchen, 2: Linie gefunden, 10: Linie verfolgen) */

/**
 * Zeit zwischen zwei Korrekturen [ms]. Groessere Werte bewirken "ruhigeres" Fahren,
 * erhoehen damit aber auch die Reaktionszeit (z.B. bei scharfen Kurven problematisch)
 */
#define CORRECTION_DELAY 150

void bot_follow_line_behaviour(Behaviour_t* data) {
    static int16_t lastLeft = 0;
    static int16_t lastRight = 0;
    static uint8_t lastCorrection = 0;
    static uint32_t lastCorrectionTime = 0;
    static uint32_t find_i = 1;

    switch (state) {
        case 0:
            /* Initialisierung */
            lastCorrectionTime = 0;
            find_i = 1;
            state = 1;
#ifdef BEHAVIOUR_AVOID_COL_AVAILABLE
            activateBehaviour(data, bot_avoid_col_behaviour);
#endif
            break;

        case 1:
            /* Linie suchen */
            if (sensLineL < LINE_SENSE && sensLineR < LINE_SENSE) {
                /* Bot steht nicht auf der Linie -> Spirale gegen den Uhrzeigersinn fahren */
                if (TIMER_GET_TICKCOUNT_32 - lastCorrectionTime < MS_TO_TICKS(500)) {
                    speedWishLeft = BOT_SPEED_STOP;
                    speedWishRight = BOT_SPEED_FOLLOW;
                    return;
                } else if (TIMER_GET_TICKCOUNT_32 - lastCorrectionTime < MS_TO_TICKS(find_i)) {
                    speedWishLeft = BOT_SPEED_SLOW;
                    speedWishRight = BOT_SPEED_SLOW;
                    return;
                }
                lastCorrectionTime = TIMER_GET_TICKCOUNT_32;
                find_i += 10;
                speedWishLeft = BOT_SPEED_IGNORE;
                speedWishRight = BOT_SPEED_IGNORE;
            } else if (sensLineL >= LINE_SENSE && sensLineR >= LINE_SENSE) {
                /* Beide Sensoren sehen die Linie -> im Uhrzeigersinn drehen */
                speedWishLeft = BOT_SPEED_SLOW;
                speedWishRight = BOT_SPEED_STOP;
            } else if (sensLineL >= LINE_SENSE && sensLineR < LINE_SENSE) {
                /* rechte Linienkante gefunden */
                LOG_DEBUG("follow_line: rechte Kante gefunden.");
                state = 2;
            } else if (sensLineL < LINE_SENSE && sensLineR >= LINE_SENSE) {
                /* linke Linienkante gefunden -> um 180 Grad drehen */
                LOG_DEBUG("follow_line: linke Kante gefunden.");
                speedWishLeft = BOT_SPEED_STOP;
                speedWishRight = BOT_SPEED_STOP;
                bot_turn(data, 180);
                state = 2;
            }
            break;

        case 2:
            /* Linie gefunden */
#ifdef BEHAVIOUR_AVOID_COL_AVAILABLE
            deactivateBehaviour(bot_avoid_col_behaviour);
#endif
            state = 10;
            break;

        default: {
            /* Linie verfolgen */
            uint8_t correction = 0;
            if (sensLineL >= LINE_SENSE && sensLineR < LINE_SENSE) {
                /* Bot faehrt auf rechter Linienkante */
                speedWishLeft = BOT_SPEED_SLOW;
                speedWishRight = BOT_SPEED_SLOW;
            } else if (sensLineL < LINE_SENSE) {
                /* Bot fahert rechts neben der Linie */
                speedWishLeft = -BOT_SPEED_FOLLOW;
                speedWishRight = BOT_SPEED_FOLLOW;
                correction = 1;
            } else {
                /* Bot faehrt auf der Linie */
                speedWishLeft = BOT_SPEED_FOLLOW;
                speedWishRight = -BOT_SPEED_SLOW;
                correction = 2;
            }

            if (lastCorrection != correction && !timer_ms_passed_32(&lastCorrectionTime, CORRECTION_DELAY)) {
                /* Falls die letzte Korrektur gerade erst war, reagieren wir (noch) nicht */
                speedWishLeft = lastLeft;
                speedWishRight = lastRight;
                return;
            }

            /* neue Werte merken */
            lastCorrection = correction;
            lastLeft = speedWishLeft;
            lastRight = speedWishRight;
            break;
        }
    }
}

void bot_follow_line(Behaviour_t* caller, uint8_t search) {
    switch_to_behaviour(caller, bot_follow_line_behaviour, BEHAVIOUR_NOOVERRIDE);
    /* stoerende Notfallverhalten aus */
#ifdef BEHAVIOUR_AVOID_COL_AVAILABLE
    deactivateBehaviour(bot_avoid_col_behaviour);
#endif
#ifdef BEHAVIOUR_AVOID_BORDER_AVAILABLE
    deactivateBehaviour(bot_avoid_border_behaviour);
#endif
#ifdef BEHAVIOUR_SCAN_AVAILABLE
    set_scan_otf_border(0); // keine Abgruende (die Linie) in die Map eintragen
    set_scan_otf_mapmode(0); // Kartographiemodus aus
#endif
    state = search ? 0 : 10;
}

#elif FOLLOW_LINE_VERSION == 3 // neueste Version des Linienfolgers, die mit drive_line_shortest_way entstanden ist
#include "math_utils.h"

/**
 * Zeit zwischen zwei Korrekturen [ms]. Groessere Werte bewirken "ruhigeres" Fahren,
 * erhoehen damit aber auch die Reaktionszeit (z.B. bei scharfen Kurven problematisch)
 */
#define CORRECTION_DELAY 100

/** Kennung links, welcher der Bordersensoren zugeschlagen hat zur Erkennung der Kreuzungen, notwendig
 *  weil sicht nicht immer beide gleichzeitig ueber Kreuzungslinie befinden */
static uint8_t border_side_l_fired = 0;

/** Kennung links, welcher der Bordersensoren zugeschlagen hat zur Erkennung der Kreuzungen, notwendig
 *  weil sicht nicht immer beide gleichzeitig ueber Kreuzungslinie befinden */
static uint8_t border_side_r_fired = 0;

static int16_t lastpos_x = 0;
static int16_t lastpos_y = 0;

/** nach dieser gefahrenen Strecke in mm werden Borderkennungen rueckgesetzt */
#define CHECK_DISTANCE 100
#define CHECK_DISTANCE_QUAD (CHECK_DISTANCE * CHECK_DISTANCE) /**< Quadrat der gefahrenen Strecke */


static uint8_t distance_reached(int16_t* last_xpoint, int16_t* last_ypoint) {
    // Abstand seit letztem Observerlauf ermitteln
    uint16_t diff = (uint16_t) get_dist(x_pos, y_pos, *last_xpoint, *last_ypoint);

    // erst nach gewissem Abstand oder gleich bei noch initialem Wert Mappruefung
    if (diff >= CHECK_DISTANCE_QUAD) {
        *last_xpoint = x_pos;
        *last_ypoint = y_pos;
        return True;
    }
    return False;
}

void bot_follow_line_behaviour(Behaviour_t* data) {
    static int16_t lastLeft = 0;
    static int16_t lastRight = 0;
    static uint8_t lastCorrection = 0;
    static uint32_t lastCorrectionTime = 0;
    uint8_t correction = 0;

    if (sensLineL >= LINE_SENSE) {
        /* Bot faehrt auf linker Linienkante oder beide Sensoren auf Linie */
        speedWishLeft = BOT_SPEED_SLOW;
        speedWishRight = BOT_SPEED_SLOW;
        lastCorrection = 0;

        if (sensBorderL > BORDER_DANGEROUS || sensBorderR > BORDER_DANGEROUS) {
            // Kennungen setzen auf Querlinie erkannt links oder rechts voraus, also wenn Abgrundsensor Linie (vorausgesetzt Abgrund gibt es nicht) erkennt
            if (sensBorderR > BORDER_DANGEROUS) {
                border_side_r_fired = True;
            } else {
                border_side_l_fired = True;
            }

            LOG_DEBUG("Border l/r: %1d %1d", border_side_l_fired, border_side_r_fired);

            // Beide erkennen Querlinie, ist Abgrund oder auch moeglicherweise X-Kreuzung
            if (sensBorderL > BORDER_DANGEROUS && sensBorderR > BORDER_DANGEROUS) {
                border_side_l_fired = True;
                border_side_r_fired = True;
            }

            if (border_side_l_fired && border_side_r_fired) {
                LOG_DEBUG("beide zugeschlagen => Ende");
                return_from_behaviour(data);
            }

        } else {
            if (lastLeft < 0 || lastRight < 0) {
                // bei Wechsel zu geradeaus Abgrundsensorkennungen ruecksetzen
                if (distance_reached(&lastpos_x, &lastpos_y)) {
                    LOG_DEBUG("Abstand gefahren und false l/r %1d %1d", sensBorderL, sensBorderR);
                    border_side_l_fired = False;
                    border_side_r_fired = False;
                }
            }
        }

        // Naechstes klappt prima im Sim, beim echten Bot jedoch nicht und dreht dort links, scheinbar wird die Var zu frueh rueckgesetzt
    } else if (sensLineL < LINE_SENSE && sensLineR < LINE_SENSE && !border_side_r_fired) {
        /* Linker Sensor und rechter nicht auf Linie, dann rechts daneben und links drehen */
        speedWishLeft = -BOT_SPEED_FOLLOW;
        speedWishRight = BOT_SPEED_FOLLOW;
        correction = 1;
    } else {
        /* Bot befindet sich links von der Linie und rechts drehen */
        speedWishLeft = BOT_SPEED_FOLLOW;
        speedWishRight = -BOT_SPEED_FOLLOW;
        correction = 2;
    }

    if (lastCorrection != correction && !timer_ms_passed_32(&lastCorrectionTime, CORRECTION_DELAY)) {
        /* Falls die letzte Korrektur gerade erst war, reagieren wir (noch) nicht */
        speedWishLeft = lastLeft;
        speedWishRight = lastRight;
        //		LOG_DEBUG("correction %u delayed", correction);
        //		LOG_DEBUG("lastLeft=%d\tlastRight=%d", lastLeft, lastRight);
        //		LOG_DEBUG("lastCorrectionTime=%lu", lastCorrectionTime);
        //		LOG_DEBUG("time=%lu", TIMER_GET_TICKCOUNT_32);
        //		if (TIMER_GET_TICKCOUNT_32 > lastCorrectionTime + MS_TO_TICKS(CORRECTION_DELAY)) {
        //			LOG_ERROR("*** TIMER-BUG! ***");
        //		}
        return;
    }

    /* neue Werte merken */
    lastCorrection = correction;
    lastLeft = speedWishLeft;
    lastRight = speedWishRight;
}

void bot_follow_line(Behaviour_t* caller, uint8_t dummy __attribute__((unused))) {
    switch_to_behaviour(caller, bot_follow_line_behaviour, BEHAVIOUR_NOOVERRIDE);

    // Kennungen init.
    border_side_l_fired = False;
    border_side_r_fired = False;
    lastpos_x = x_pos;
    lastpos_y = y_pos;

    /* stoerende Notfallverhalten aus */
#ifdef BEHAVIOUR_AVOID_COL_AVAILABLE
    deactivateBehaviour(bot_avoid_col_behaviour);
#endif
#ifdef BEHAVIOUR_AVOID_BORDER_AVAILABLE
    deactivateBehaviour(bot_avoid_border_behaviour);
#endif
}

#endif // VERSION
#endif // BEHAVIOUR_FOLLOW_LINE_AVAILABLE
