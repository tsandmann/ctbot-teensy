/*
 * This file is part of the ct-Bot teensy framework.
 * Copyright (c) 2019 Timo Sandmann
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 3.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

/**
 * @file    pos_store.h
 * @brief   Lagecy wrapper for position store implementation
 * @author  Timo Sandmann
 * @date    18.12.2019
 */

#pragma once

#include "bot-logic_legacy.h"

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define POS_STORE_SIZE 256 /**< (maximale) Groesse des Positionsspeichers (pro Platz) */

typedef Behaviour_t pos_store_t;
typedef uint32_t pos_store_size_t;

extern pos_store_t* (*pos_store_create_size)(Behaviour_t*, void*, const pos_store_size_t);
extern pos_store_t* (*pos_store_create)(Behaviour_t*, void*);
extern pos_store_t* (*pos_store_new_size)(Behaviour_t*, const pos_store_size_t);
extern pos_store_t* (*pos_store_new)(Behaviour_t*);
extern pos_store_t* (*pos_store_from_beh)(Behaviour_t*);
extern pos_store_t* (*pos_store_from_index)(const uint8_t);
extern uint8_t (*pos_store_get_index)(pos_store_t*);
extern void (*pos_store_clear)(pos_store_t*);
extern void (*pos_store_release)(pos_store_t*);
extern void (*pos_store_release_all)();
extern uint8_t (*pos_store_pop)(pos_store_t*, position_t*);
extern uint8_t (*pos_store_insert)(pos_store_t*, const position_t);
extern uint8_t (*pos_store_push)(pos_store_t*, const position_t);
extern uint8_t (*pos_store_queue)(pos_store_t*, const position_t);
extern uint8_t (*pos_store_dequeue)(pos_store_t*, position_t*);
extern uint8_t (*pos_store_top)(pos_store_t*, position_t*, const uint8_t);

#ifdef __cplusplus
}
#endif
