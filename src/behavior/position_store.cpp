/*
 * This file is part of the c't-Bot teensy framework.
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
 * @file    position_store.cpp
 * @brief   Position store implementation
 * @author  Timo Sandmann
 * @date    18.12.2019
 */

#include "position_store.h"

namespace ctbot {

std::map<legacy::Behaviour_t*, std::unique_ptr<PositionStoreLegacy>> PositionStoreLegacy::stores_;
std::vector<legacy::Behaviour_t*> PositionStoreLegacy::stores_vec_;

PositionStoreLegacy::pos_store_t* PositionStoreLegacy::pos_store_create_size(legacy::Behaviour_t* owner, void*, const pos_store_size_t) noexcept {
    stores_[owner] = std::make_unique<PositionStoreLegacy>();
    stores_vec_.push_back(owner);
    return owner;
}

PositionStoreLegacy::pos_store_t* PositionStoreLegacy::pos_store_from_beh(legacy::Behaviour_t* owner) noexcept {
    if (stores_.find(owner) != stores_.end()) {
        return owner;
    }
    return nullptr;
}

PositionStoreLegacy::pos_store_t* PositionStoreLegacy::pos_store_from_index(const uint8_t index) noexcept {
    if (index >= stores_vec_.size()) {
        return nullptr;
    }
    return stores_vec_[index];
}

uint8_t PositionStoreLegacy::pos_store_get_index(PositionStoreLegacy::pos_store_t* store) noexcept {
    uint8_t i {};
    for (auto owner : stores_vec_) {
        if (owner == store) {
            return i;
        } else {
            ++i;
        }
    }
    return 255;
}

void PositionStoreLegacy::pos_store_clear(PositionStoreLegacy::pos_store_t* store) noexcept {
    stores_.at(store)->positions_.clear();
}

void PositionStoreLegacy::pos_store_release(PositionStoreLegacy::pos_store_t* store) noexcept {
    stores_.erase(store);
}

void PositionStoreLegacy::pos_store_release_all() noexcept {
    stores_.clear();
}

uint8_t PositionStoreLegacy::pos_store_pop(PositionStoreLegacy::pos_store_t* store, legacy::position_t* pos) noexcept {
    Pose position;
    if (!stores_.at(store)->pop(position)) {
        return false;
    }
    pos->x = position.get_x<int16_t>();
    pos->y = position.get_y<int16_t>();
    return true;
}

uint8_t PositionStoreLegacy::pos_store_insert(PositionStoreLegacy::pos_store_t* store, const legacy::position_t pos) noexcept {
    Pose position(pos.x, pos.y, 0.f);
    return stores_.at(store)->insert(position);
}

uint8_t PositionStoreLegacy::pos_store_push(PositionStoreLegacy::pos_store_t* store, const legacy::position_t pos) noexcept {
    Pose position(pos.x, pos.y, 0.f);
    return stores_.at(store)->push(position);
}

uint8_t PositionStoreLegacy::pos_store_dequeue(PositionStoreLegacy::pos_store_t* store, legacy::position_t* pos) noexcept {
    Pose position;
    if (!stores_.at(store)->dequeue(position)) {
        return false;
    }
    pos->x = position.get_x<int16_t>();
    pos->y = position.get_y<int16_t>();
    return true;
}

uint8_t PositionStoreLegacy::pos_store_top(PositionStoreLegacy::pos_store_t* store, legacy::position_t* pos, const uint8_t index) noexcept {
    Pose position;
    if (!stores_.at(store)->top(position, index)) {
        return false;
    }
    pos->x = position.get_x<int16_t>();
    pos->y = position.get_y<int16_t>();
    return true;
}

} /* namespace ctbot */
