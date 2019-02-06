/*
 * This file is part of the c't-Bot teensy framework.
 * Copyright (c) 2018 Timo Sandmann
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
 * @file    condition.h
 * @brief   Condition variable implementation
 * @author  Timo Sandmann
 * @date    28.07.2018
 */

#pragma once

#include <cstdint>
#include <list>


namespace ctbot {
class Task;

/**
 * @brief Condition variable
 *
 * @startuml{Condition.png}
 *  !include condition.puml
 *  set namespaceSeparator ::
 *  skinparam classAttributeIconSize 0
 * @enduml
 */
class Condition {
protected:
    std::list<Task*> wait_list_;

public:
    // FIXME: add documentation
    void add_task(Task* p_task);
    void remove_task(Task* p_task);
    bool notify();
    bool notify_all();
    bool task_waiting(const Task* p_task) const;
};

} // namespace ctbot
