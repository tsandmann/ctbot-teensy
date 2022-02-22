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
 * @file    position_store.h
 * @brief   Position store implementation
 * @author  Timo Sandmann
 * @date    18.12.2019
 */

#pragma once

#include "pose.h"
#include "legacy/behavior_legacy.h"

#include <deque>
#include <map>
#include <memory>


namespace ctbot {

class PositionStore {
protected:
    std::deque<Pose> positions_;

public:
    PositionStore() = default;
    virtual ~PositionStore() = default;

    bool pop(Pose& pos) noexcept {
        if (positions_.empty()) {
            return false;
        }
        pos = positions_.back();
        positions_.pop_back();
        return true;
    }

    bool insert(Pose& pos) {
        positions_.push_front(pos);
        return true;
    }

    bool push(Pose& pos) {
        positions_.push_back(pos);
        return true;
    }

    bool dequeue(Pose& pos) noexcept {
        if (positions_.empty()) {
            return false;
        }
        pos = positions_.front();
        positions_.pop_front();
        return true;
    }

    bool top(Pose& pos, const uint8_t index) const noexcept {
        if (index > positions_.size()) {
            return false;
        }
        pos = positions_.at(positions_.size() - index);
        return true;
    }
};


class PositionStoreLegacy : public PositionStore {
protected:
    static std::map<legacy::Behaviour_t*, std::unique_ptr<PositionStoreLegacy>> stores_;
    static std::vector<legacy::Behaviour_t*> stores_vec_;

public:
    using pos_store_t = legacy::Behaviour_t;
    using pos_store_size_t = uint32_t;

    /**
     * Erzeugt einen neuen Positionsspeicher angegebener Groesse
     * @param *owner    Zeiger Verhaltensdatensatz
     * @param *data     NULL oder Zeiger auf Speicher fuer size * sizeof(position_t) Bytes
     * @param size      Groesse des Speichers, <= POS_STORE_SIZE
     * @return          Zeiger auf neuen Positionsspeicher oder NULL
     */
    FLASHMEM static pos_store_t* pos_store_create_size(legacy::Behaviour_t* owner, void* data, const pos_store_size_t size) noexcept;

    /**
     * Erzeugt einen neuen Positionsspeicher maximaler Groesse
     * @param *owner    Zeiger Verhaltensdatensatz
     * @param *data     NULL oder Zeiger auf Speicher fuer POS_STORE_SIZE * sizeof(position_t) Bytes
     * @return          Zeiger auf neuen Positionsspeicher oder NULL
     * @see pos_store_create_size()
     */
    FLASHMEM static auto pos_store_create(legacy::Behaviour_t* owner, void* data) noexcept {
        return pos_store_create_size(owner, data, 0);
    }

    /**
     * Erzeugt einen neuen Positionsspeicher angegebener Groesse
     * @param *owner    Zeiger Verhaltensdatensatz
     * @param size      Groesse des Speichers, <= POS_STORE_SIZE
     * @return          Zeiger auf neuen Positionsspeicher oder NULL
     */
    FLASHMEM static auto pos_store_new_size(legacy::Behaviour_t* owner, const pos_store_size_t size) noexcept {
        return pos_store_create_size(owner, nullptr, size);
    }

    /**
     * Erzeugt einen neuen Positionsspeicher maximaler Groesse
     * @param *owner    Zeiger Verhaltensdatensatz
     * @return          Zeiger auf neuen Positionsspeicher oder NULL
     * @see pos_store_new_size()
     */
    FLASHMEM static auto pos_store_new(legacy::Behaviour_t* owner) noexcept {
        return pos_store_create_size(owner, nullptr, 0);
    }

    /**
     * Ermittelt den Positionsspeicher, der zu einem Verhalten gehoert
     * @param *owner    Zeiger auf Verhaltensdatensatz
     * @return          Zeiger auf Positionsspeicher oder NULL
     */
    static pos_store_t* pos_store_from_beh(legacy::Behaviour_t* owner) noexcept;

    /**
     * Ermittelt den Positionsspeicher, der den gegebenen Index im Array hat
     * @param index     Index des Positionsspeichers im Array
     * @return          Zeiger auf Positionsspeicher oder NULL
     */
    static pos_store_t* pos_store_from_index(const uint8_t index) noexcept;

    /**
     * Ermittelt den Index eines Positionsspeichers
     * @param *store    Zeiger auf Positionsspeicher
     * @return          Index des Positionsspeichers im Array
     */
    static uint8_t pos_store_get_index(pos_store_t* store) noexcept;

    /**
     * Leert den Positionsspeicher
     * @param *store    Zeiger auf Positionsspeicher
     */
    static void pos_store_clear(pos_store_t* store) noexcept;

    /**
     * Loescht einen Positionsspeicher
     * @param *store    Zeiger auf Positionsspeicher
     */
    FLASHMEM static void pos_store_release(pos_store_t* store) noexcept;

    /**
     * Loescht alle Positionsspeicher
     */
    FLASHMEM static void pos_store_release_all() noexcept;

    /**
     * Pop-Routine zur Rueckgabe des letzten auf dem Stack gepushten Punktes
     * @param *store    Zeiger auf Positionsspeicher
     * @param *pos      Zeiger auf Rueckgabe-Speicher der Position
     * @return          False falls Pop nicht erfolgreich, d.h. kein Punkt mehr auf dem Stack, sonst True nach erfolgreichem Pop
     */
    static uint8_t pos_store_pop(pos_store_t* store, legacy::position_t* pos) noexcept;

    /**
     * Speichern einer Koordinate vorne im Speicher
     * @param *store    Zeiger auf Positionsspeicher
     * @param pos       X/Y-Koordinaten des zu sichernden Punktes
     * @return          True wenn erfolgreich sonst False wenn Array voll ist
     */
    static uint8_t pos_store_insert(pos_store_t* store, const legacy::position_t pos) noexcept;

    /**
     * Speichern einer Koordinate auf dem Stack
     * @param *store    Zeiger auf Positionsspeicher
     * @param pos       X/Y-Koordinaten des zu sichernden Punktes
     * @return          True wenn erfolgreich sonst False wenn Array voll ist
     */
    static uint8_t pos_store_push(pos_store_t* store, const legacy::position_t pos) noexcept;

    /**
     * Erweiterung des Stacks zur Queue; Element wird hinten angefuegt, identisch dem Stack-Push
     * @param *store    Zeiger auf Positionsspeicher
     * @param pos       X/Y-Koordinaten des zu sichernden Punktes
     * @return          True wenn erfolgreich sonst False wenn Array voll ist
     */
    static auto pos_store_queue(pos_store_t* store, const legacy::position_t pos) noexcept {
        return pos_store_push(store, pos);
    }

    /**
     * Erweiterung des Stacks zur Queue; Element wird vorn entnommen
     * @param *store    Zeiger auf Positionsspeicher
     * @param *pos      Zeiger auf Rueckgabe-Speicher der Position
     * @return          True wenn Element erfolgreich entnommen werden konnte sonst False falls kein Element mehr enthalten ist
     */
    static uint8_t pos_store_dequeue(pos_store_t* store, legacy::position_t* pos) noexcept;

    /**
     * Gibt das n-letzte Element des Stacks / der Queue zurueck, entfernt es aber nicht.
     * pos_store_top(&store,  &pos, 2) gibt z.B. das vorletzte Element zurueck
     * @param *store    Zeiger auf Positionsspeicher
     * @param *pos      Zeiger auf Rueckgabe-Speicher der Position
     * @param index     Index des gewuenschten Elements vom Ende aus gezaehlt, 1-based
     * @return          True, wenn ein Element im Speicher ist, sonst False
     */
    static uint8_t pos_store_top(pos_store_t* store, legacy::position_t* pos, const uint8_t index) noexcept;
};

} // namespace ctbot
