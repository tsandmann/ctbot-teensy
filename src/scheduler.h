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

#ifndef SRC_SCHEDULER_H_
#define SRC_SCHEDULER_H_

#include "ctbot_config.h"

#include <cstdint>
#include <vector>
#include <string>


extern "C" {
void vTaskSuspendAll();
long xTaskResumeAll();
}

namespace ctbot {

class CommInterface;

/**
 * @brief Cooperative scheduler implementation for periodic tasks
 */
class Scheduler {
public:
    using task_func_data_t = void*;
    using task_func_t = void (*)(task_func_data_t);

protected:
    static constexpr uint16_t DEFAULT_STACK_SIZE { 2U * 1024U };

    /**
     * @brief Task control block
     */
    struct Task {
        uint16_t id_; /**< ID of task */
        uint16_t period_; /**< Execution period of task in ms */
        bool active_; /**< Flag indicating if task is runnable (true) or blocked (false) */
        task_func_t func_; /**< Function pointer to the task's implementation */
        task_func_data_t func_data_; /**< Pointer to additional data for the task */
        void* handle_; /**< FreeRTOS task handle */
        std::string name_;

        /**
         * @brief Construct a new Task object
         * @param[in] id: ID of task
         * @param[in] name: Name of task
         * @param[in] period: Execution period of task in ms
         * @param[in] func: Function pointer to the task's implementation
         * @param[in] func_data: Pointer to additional data for the task
         */
        Task(const uint16_t id, const std::string& name, const uint16_t period, task_func_t&& func, task_func_data_t&& func_data);

        /**
         * @brief Print task information to a CommInterface
         * @param[in] comm: Reference to CommInterface instance to print to
         */
        void print(CommInterface& comm) const;
    };

    uint16_t next_id_; /**< Next task ID to use */
    std::vector<Task*> task_vector_; /**< Vector containing pointer to the tasks, ordering by task ID */
    void* tast_vector_mutex_;

public:
    /**
     * @brief Stop (exit) the scheduler
     * @note Calls FreeRTOS' vTaskEndScheduler()
     */
    static void stop();

    static inline void enter_critical_section() {
        vTaskSuspendAll();
    }

    static inline void exit_critical_section() {
        xTaskResumeAll();
    }

    /**
     * @brief Construct a new Scheduler object
     */
    Scheduler();

    /**
     * @brief Destroy the Scheduler object
     */
    ~Scheduler();

    /**
     * @brief Add a tasks to the run queue
     * @param[in] name: Reference to string with name of the task
     * @param[in] period: Execution period of task in ms
     * @param[in] stack_size: Size of task's stack in byte
     * @param[in] func: Function pointer to the task's implementation
     * @param[in] func_data: Pointer to additional data for the task
     * @return ID of created task or 0 in case of an error
     */
    uint16_t task_add(const std::string& name, const uint16_t period, const uint32_t stack_size, task_func_t&& func, task_func_data_t&& func_data);

    /**
     * @brief Add a tasks to the run queue
     * @param[in] name: Reference to string with name of the task
     * @param[in] period: Execution period of task in ms
     * @param[in] func: Function pointer to the task's implementation
     * @param[in] func_data: Pointer to additional data for the task
     * @return ID of created task or 0 in case of an error
     */
    uint16_t task_add(const std::string& name, const uint16_t period, task_func_t&& func, task_func_data_t&& func_data) {
        return task_add(name, period, DEFAULT_STACK_SIZE, std::move(func), std::move(func_data));
    }

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
    const Task* task_get(const uint16_t id) const;

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

    /**
     * @brief Print a list of all tasks and their current status
     * @param[in] comm: Reference to CommInterface instance used to print the list
     */
    void print_task_list(CommInterface& comm) const;

    /**
     * @brief Print number of free (heap) RAM in byte
     * @param[in] comm: Reference to CommInterface instance used to print with
     */
    void print_free_ram(CommInterface& comm) const;
};

} // namespace ctbot

#endif /* SRC_SCHEDULER_H_ */
