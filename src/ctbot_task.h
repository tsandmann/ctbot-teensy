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
 * @file    ctbot_task.h
 * @brief   Task implementation
 * @author  Timo Sandmann
 * @date    28.07.2018
 */

#pragma once

#include <cstdint>
#include <string>
#include <string_view>
#include <functional>
#include <thread>


namespace ctbot {
class Scheduler;
class CommInterface;

/**
 * @brief Task representation
 *
 * @startuml{Task.png}
 *  !include ctbot_task.puml
 *  set namespaceSeparator ::
 *  skinparam classAttributeIconSize 0
 * @enduml
 */
class Task {
    friend class Scheduler;

public:
    using func_t = std::function<void(void)>;

protected:
    uint16_t id_; /**< ID of task */
    uint16_t period_; /**< Execution period of task in ms */
    uint8_t priority_;
    volatile uint8_t state_; /**< Flag indicating the state of the task: runnable (1), blocked (2), finished (0) */ // FIXME: enum class?
    func_t func_; /**< Function wrapper for the task's implementation */
    bool std_thread_;
    bool external_;
    union {
        void* p_freertos_handle;
        std::thread* p_thread; /**< Thread of task */
    } handle_; // FIXME: think about this
    Scheduler& scheduler_; /**< Reference to scheduler for this task */
    std::string name_; /**< Name of task */

public:
    /**
     * @brief Construct a new Task object
     * @param[in] id: ID of task
     * @param[in] name: Name of task
     * @param[in] period: Execution period of task in ms
     * @param[in] func: Function wrapper for the task's implementation
     */
    Task(Scheduler& scheduler, const uint16_t id, const std::string_view& name, const bool external, const uint16_t period, const uint8_t priority,
        func_t&& func);

    /**
     * @brief Destroy the Task object
     */
    ~Task();

    // FIXME: Documentation
    bool resume();

    uint8_t get_priority() const {
        return priority_;
    }

    auto& get_name() const {
        return name_;
    }

    /**
     * @brief Print task information to a CommInterface
     * @param[in] comm: Reference to CommInterface instance to print to
     */
    void print(CommInterface& comm) const;
};

} // namespace ctbot
