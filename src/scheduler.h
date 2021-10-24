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
#include <vector>
#include <list>
#include <map>
#include <string>
#include <string_view>
#include <functional>
#include <thread>
#include <mutex>


extern "C" {
void vTaskSuspendAll();
long xTaskResumeAll();
}

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
    static constexpr uint8_t DEBUG_LEVEL_ { 2 };

protected:
    static constexpr uint8_t DEFAULT_PRIORITY { 4 };
    static constexpr uint32_t DEFAULT_STACK_SIZE { 2 * 1024 }; // byte

    static TaskHandle_t p_main_task_;

    uint16_t next_id_; /**< Next task ID to use */
    std::map<uint16_t /*ID*/, Task* /*task pointer*/> tasks_; /**< Map containing pointer to the tasks, using task ID as key */
    mutable std::mutex task_mutex_;

public:
    static constexpr uint8_t MAX_PRIORITY { 9 };

    /**
     * @brief Stop (exit) the scheduler
     * @note Calls FreeRTOS' vTaskEndScheduler()
     */
    FLASHMEM static void stop();

    static inline void enter_critical_section() {
        vTaskSuspendAll();
    }

    static inline void exit_critical_section() {
        xTaskResumeAll();
    }

    /**
     * @brief Construct a new Scheduler object
     */
    FLASHMEM Scheduler();

    /**
     * @brief Destroy the Scheduler object
     */
    FLASHMEM ~Scheduler();

    /**
     * @brief Add a tasks to the run queue
     * @param[in] name: Reference to a string_view with name of the task
     * @param[in] period: Execution period of task in ms
     * @param[in] priority: Priority of task
     * @param[in] stack_size: Size of task's stack in byte
     * @param[in] func: Function wrapper for the task's implementation
     * @return ID of created task or 0 in case of an error
     */
    FLASHMEM uint16_t task_add(const std::string_view& name, const uint16_t period, const uint8_t priority, const uint32_t stack_size, Task::func_t&& func);

    /**
     * @brief Add a tasks to the run queue
     * @param[in] name: Reference to a string_view with name of the task
     * @param[in] period: Execution period of task in ms
     * @param[in] stack_size: Size of task's stack in byte
     * @param[in] func: Function wrapper for the task's implementation
     * @return ID of created task or 0 in case of an error
     */
    FLASHMEM uint16_t task_add(const std::string_view& name, const uint16_t period, const uint32_t stack_size, Task::func_t&& func) {
        return task_add(name, period, DEFAULT_PRIORITY, stack_size, std::move(func));
    }

    /**
     * @brief Add a tasks to the run queue
     * @param[in] name: Reference to a string_view with name of the task
     * @param[in] period: Execution period of task in ms
     * @param[in] func: Function wrapper for the task's implementation
     * @return ID of created task or 0 in case of an error
     */
    FLASHMEM uint16_t task_add(const std::string_view& name, const uint16_t period, Task::func_t&& func) {
        return task_add(name, period, DEFAULT_PRIORITY, DEFAULT_STACK_SIZE, std::move(func));
    }

    FLASHMEM uint16_t task_register(const std::string_view& name, const bool external = false);

    FLASHMEM uint16_t task_register(TaskHandle_t task, const bool external);

    FLASHMEM bool task_remove(const uint16_t task);

    /**
     * @brief Get the ID of a task given by its name
     * @param[in] name: Reference to a string_view with task name
     * @return ID of searched task or 0xffff, if task name is unknown
     */
    FLASHMEM uint16_t task_get(const std::string_view& name) const;

    /**
     * @brief Get a pointer to a tasks' control structure (of type Task)
     * @param[in] id: ID of task to get
     * @return Pointer to task entry
     */
    FLASHMEM Task* task_get(const uint16_t id) const;

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

    bool task_join(const uint16_t id);

    FLASHMEM bool task_set_finished(const uint16_t id);

    /**
     * @brief Print a list of all tasks and their current status
     * @param[in] comm: Reference to CommInterface instance used to print the list
     */
    FLASHMEM void print_task_list(CommInterface& comm) const;

    /**
     * @brief Print amount of used and free (heap) RAM in byte
     * @param[in] comm: Reference to CommInterface instance used to print with
     */
    FLASHMEM void print_ram_usage(CommInterface& comm) const;

    FLASHMEM size_t get_free_stack() const;

    FLASHMEM size_t get_free_stack(const uint16_t id) const;

    FLASHMEM std::unique_ptr<std::vector<std::pair<TaskHandle_t, float>>> get_runtime_stats() const;
};

} // namespace ctbot
