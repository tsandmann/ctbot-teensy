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
 * @file    scheduler.h
 * @brief   Task scheduling
 * @author  Timo Sandmann
 * @date    15.04.2018
 */

#pragma once

#include "ctbot_config.h"
#include "ctbot_task.h"

#include <cstdint>
#include <queue>
#include <map>
#include <string>
#include <functional>


namespace ctbot {

class CommInterface;

/**
 * @brief Cooperative scheduler implementation for periodic tasks
 *
 * @startuml{Scheduler.png}
 *  !include scheduler.puml
 *  set namespaceSeparator ::
 *  skinparam classAttributeIconSize 0
 * @enduml
 */
class Scheduler {
protected:
    static constexpr uint8_t DEFAULT_PRIORITY { 4 };

    struct TaskComparator {
        /**
         * @brief Compare operator for Task
         * @param[in] *lhs Pointer to task to compare with other task
         * @param[in] *rhs Pointer other task for comparison
         * @return true, if the other tasks has to be executed before (a lower next runtime)
         */
        bool operator()(Task* lhs, Task* rhs) {
            return lhs->next_runtime_ > rhs->next_runtime_; // lowest next_runtime will be executed first!
        }
    };

    uint16_t next_id_; /**< Next task ID to use */
    std::priority_queue<Task*, std::vector<Task*>, TaskComparator> task_queue_; /**< Queue of all tasks, sorted ascending by next runtime */
    std::map<uint16_t /*ID*/, Task* /*task pointer*/> tasks_; /**< Map containing pointer to the tasks, using task ID as key */

public:
    /**
     * @brief Entry point of scheduler
     * @note Does not return until scheduler is stoppped
     */
    void run();

    /**
     * @brief Stop the scheduler (and its main loop, @see run)
     */
    static void stop();

    static inline void enter_critical_section() {
        // FIXME: disable ints?
    }

    static inline void exit_critical_section() {
        // FIXME: disable ints?
    }

    /**
     * @brief Construct a new Scheduler object
     */
    Scheduler();

    /**
     * @brief Add a tasks to the run queue
     * @param[in] name: Reference to string with name of the task
     * @param[in] period: Execution period of task in ms
     * @param[in] priority: Priority of task
     * @param[in] stack_size: Size of task's stack in byte (dummy)
     * @param[in] func: Function wrapper for the task's implementation
     * @return ID of created task or 0 in case of an error
     */
    uint16_t task_add(const std::string& name, const uint16_t period, const uint8_t priority, const uint32_t stack_size, Task::func_t&& func);

    /**
     * @brief Add a tasks to the run queue
     * @param[in] name: Reference to string with name of the task
     * @param[in] period: Execution period of task in ms
     * @param[in] stack_size: Size of task's stack in byte (dummy)
     * @param[in] func: Function wrapper for the task's implementation
     * @return ID of created task or 0 in case of an error
     */
    uint16_t task_add(const std::string& name, const uint16_t period, const uint32_t stack_size, Task::func_t&& func) {
        return task_add(name, period, DEFAULT_PRIORITY, stack_size, std::move(func));
    }

    /**
     * @brief Add a tasks to the run queue
     * @param[in] name: Reference to string with name of the task
     * @param[in] period: Execution period of task in ms
     * @param[in] func: Function wrapper for the task's implementation
     * @return ID of created task or 0 in case of an error
     */
    uint16_t task_add(const std::string& name, const uint16_t period, Task::func_t&& func) {
        return task_add(name, period, DEFAULT_PRIORITY, 0, std::move(func));
    }

    bool task_remove(const uint16_t task);

    /**
     * @brief Get the ID of a task given by its name
     * @param[in] name: Reference to string with task name
     * @return ID of searched task or 0xffff, if task name is unknown
     */
    uint16_t task_get(const std::string& name) const;

    /**
     * @brief Get a pointer to a tasks' control structure (of type Task)
     * @param[in] id: ID of task to get
     * @return Pointer to task entry
     */
    Task* task_get(const uint16_t id) const;

    /**
     * @brief Suspend a task, task will not be scheduled again until resumed
     * @param[in] id: ID of task to suspend
     * @return true on success
     */
    bool task_suspend(const uint16_t id);

    /**
     * @brief Resume a task, task will be scheduled on next runtime
     * @param[in] id: ID of task to resume
     * @return true on success
     */
    bool task_resume(const uint16_t id);

    // FIXME: Documentation
    bool task_wait_for(const uint16_t id, Condition& cond);

    /**
     * @brief Print a list of all tasks and their current status
     * @param[in] comm: Reference to CommInterface instance used to print the list
     */
    void print_task_list(CommInterface& comm) const;

    /**
     * @brief Print amount of used and free (heap) RAM in byte
     * @param[in] comm: Reference to CommInterface instance used to print with
     */
    void print_ram_usage(CommInterface& comm) const;
};

} // namespace ctbot
