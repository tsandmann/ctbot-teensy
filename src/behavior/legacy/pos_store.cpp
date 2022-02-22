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
 * @file    pos_store.cpp
 * @brief   Lagecy wrapper for position store implementation
 * @author  Timo Sandmann
 * @date    18.12.2019
 */

#include "pos_store.h"

#include "../position_store.h"
#include "behavior_legacy.h"

#include <cstdint>


namespace ctbot {
namespace legacy {
extern "C" {
pos_store_t* (*pos_store_create_size)(Behaviour_t*, void*, const pos_store_size_t)(&PositionStoreLegacy::pos_store_create_size);
pos_store_t* (*pos_store_create)(Behaviour_t*, void*) (&PositionStoreLegacy::pos_store_create);
pos_store_t* (*pos_store_new_size)(Behaviour_t*, const pos_store_size_t)(&PositionStoreLegacy::pos_store_new_size);
pos_store_t* (*pos_store_new)(Behaviour_t*) (&PositionStoreLegacy::pos_store_new);
pos_store_t* (*pos_store_from_beh)(Behaviour_t*) (&PositionStoreLegacy::pos_store_from_beh);
pos_store_t* (*pos_store_from_index)(const uint8_t)(&PositionStoreLegacy::pos_store_from_index);
uint8_t (*pos_store_get_index)(pos_store_t*)(&PositionStoreLegacy::pos_store_get_index);
void (*pos_store_clear)(pos_store_t*)(&PositionStoreLegacy::pos_store_clear);
void (*pos_store_release)(pos_store_t*)(&PositionStoreLegacy::pos_store_release);
void (*pos_store_release_all)()(&PositionStoreLegacy::pos_store_release_all);
uint8_t (*pos_store_pop)(pos_store_t*, position_t*)(&PositionStoreLegacy::pos_store_pop);
uint8_t (*pos_store_insert)(pos_store_t*, const position_t)(&PositionStoreLegacy::pos_store_insert);
uint8_t (*pos_store_push)(pos_store_t*, const position_t)(&PositionStoreLegacy::pos_store_push);
uint8_t (*pos_store_queue)(pos_store_t*, const position_t)(&PositionStoreLegacy::pos_store_queue);
uint8_t (*pos_store_dequeue)(pos_store_t*, position_t*)(&PositionStoreLegacy::pos_store_dequeue);
uint8_t (*pos_store_top)(pos_store_t*, position_t*, const uint8_t)(&PositionStoreLegacy::pos_store_top);
} // extern C
} // namespace legacy
} // namespace ctbot
