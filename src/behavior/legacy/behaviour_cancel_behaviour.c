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
 * @file    behaviour_cancel_behaviour.c
 * @brief   Deaktiviert ein anderes Verhalten in Abhaengigkeit einer Check-Funktion
 * @author  Frank Menzel
 * @date    19.10.2007
 * @note    So kann z.B. der Wandfolger (bot_solve_maze) beendet werden, falls dieser auf eine Linie faehrt und der Linienfolger uebernehmen.
 */

#include "behaviour_cancel_behaviour.h"

#ifdef BEHAVIOUR_CANCEL_BEHAVIOUR_AVAILABLE
#include <stdlib.h>

#define DEBUG_CANCEL /**< Schalter fuer Debug-Code */

#define MAX_JOBS 8 /**< maximale Anzahl an Jobs */

#ifndef DEBUG_CANCEL
#undef LOG_DEBUG
#define LOG_DEBUG(a, ...) \
    {}
#endif

/** Liste aller Ueberwachungsauftraege */
static struct {
    Behaviour_t* beh; /**< Zeiger auf Verhaltensdatensatz des zu ueberwachenden Verhaltens */
    uint8_t (*cond)(void); /**< Zeiger auf Funktion fuer die Abbruchbedingung */
} jobs[MAX_JOBS] = { { NULL, NULL } };

void bot_behaviour_cancel_behaviour(Behaviour_t* data) {
    uint8_t idle = True;
    uint8_t i;
    for (i = 0; i < sizeof(jobs) / sizeof(jobs[0]); ++i) {
        if (jobs[i].beh != NULL) {
            if ((jobs[i].beh->active != BEHAVIOUR_ACTIVE && jobs[i].beh->subResult != BEHAVIOUR_SUBRUNNING) || jobs[i].cond == NULL) {
                LOG_DEBUG("erledigten Auftrag gefunden, wird entfernt");
                jobs[i].beh = NULL;
                jobs[i].cond = NULL;
                continue;
            }
            // LOG_DEBUG("zu ueberwachendes Verhalten %u gefunden", jobs[i].beh->priority);
            idle = False;
            if (jobs[i].cond()) {
                /* Check-Funktion vorhanden und Abbruchbedingung erfuellt */
                LOG_DEBUG("Abbruch des Verhaltens %u", jobs[i].beh->priority);
                deactivate_called_behaviours(jobs[i].beh); // vom Zielverhalten aufgerufene Verhalten beenden
                exit_behaviour(jobs[i].beh, BEHAVIOUR_SUBCANCEL); // Zielverhalten beenden, Caller reaktivieren
                jobs[i].beh = NULL;
                jobs[i].cond = NULL;
            }
        } else if (jobs[i].cond) {
            idle = False;
            if (jobs[i].cond()) {
                LOG_DEBUG("Abbruch des Verhaltens 0");
                legacy_abort(data->caller);
                jobs[i].beh = NULL;
                jobs[i].cond = NULL;
            }
        }
    }

    if (idle) {
        /* keine Auftraege mehr -> selbst deaktivieren */
        LOG_DEBUG("keine weiteren Auftraege");
        deactivate_behaviour(data);
    }
}

Behaviour_t* bot_add_behaviour_to_cancel(Behaviour_t* caller, Behaviour_t* behaviour, uint8_t (*check)(void)) {
    LOG_DEBUG("cancel(%p, %p (Prio %u), %p)", caller, behaviour, behaviour ? behaviour->priority : 0, check);
    uint8_t i;
    for (i = 0; i < sizeof(jobs) / sizeof(jobs[0]); ++i) {
        if (jobs[i].cond == NULL) {
            LOG_DEBUG(" i=%u noch frei", i);
            jobs[i].beh = behaviour;
            jobs[i].cond = check;
            break;
        }
    }
    if (i == sizeof(jobs) / sizeof(jobs[0])) {
        LOG_DEBUG("Kein Slot mehr frei, Abbruch");
        return NULL;
    } else {
        LOG_DEBUG("Abbruchbedingung korrekt registriert");
        Behaviour_t* beh = get_behaviour(bot_behaviour_cancel_behaviour);
        if (!beh) {
            LOG_ERROR("bot_behaviour_cancel_behaviour not found");
            return NULL;
        }

        beh->active = BEHAVIOUR_ACTIVE;
        beh->caller = caller;
        return beh;
    }
}

Behaviour_t* bot_cancel_behaviour(Behaviour_t* caller, BehaviourFunc_t behaviour, uint8_t (*check)(void)) {
    if (behaviour == legacy_abort_helper) {
        LOG_DEBUG("cancel for C++-behavior from legacy");
        return bot_add_behaviour_to_cancel(caller, NULL, check);
    } else {
        Behaviour_t* beh = get_behaviour(behaviour);
        if (beh) {
            LOG_DEBUG("cancel for legacy-behavior from legacy");
            return bot_add_behaviour_to_cancel(caller, beh, check);
        }
    }

    return NULL;
}

#endif // BEHAVIOUR_CANCEL_BEHAVIOUR_AVAILABLE
