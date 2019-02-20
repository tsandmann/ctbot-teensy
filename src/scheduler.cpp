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
 * @file    scheduler.cpp
 * @brief   Task scheduling
 * @author  Timo Sandmann
 * @date    15.04.2018
 */

#include "scheduler.h"
#include "condition.h"
#include "ctbot.h"
#include "timer.h"
#include "comm_interface.h"

#include "pprintpp.hpp"
#include "arduino_fixed.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "portable/teensy.h"


namespace ctbot {

Scheduler::Scheduler() : next_id_ {}, tasks_mutex_ { xSemaphoreCreateMutex() } {
    Task* p_idle_task { new Task(*this, next_id_++, configIDLE_TASK_NAME, 0, nullptr) };
    p_idle_task->handle_ = xTaskGetIdleTaskHandle();

    vTaskSuspendAll();
    tasks_[0] = p_idle_task;
    xTaskResumeAll();
}

Scheduler::~Scheduler() {
    // Serial.println("Scheduler::~Scheduler() entered.");
    enter_critical_section();
    for (auto& t : tasks_) {
        if (t.second->name_ != "main") {
            // Serial.print("Scheduler::~Scheduler(): task 0x");
            // Serial.print(t.second->id_, 16);
            // Serial.print(" @ 0x");
            // Serial.print(reinterpret_cast<uintptr_t>(t.second), 16);
            // Serial.print(" will be blocked...");
            t.second->state_ = 0;
            // Serial.println(" done.");
        }
    }
    // Serial.println("Scheduler::~Scheduler(): all tasks blocked.");

    for (auto& t : tasks_) {
        if (t.second->name_ != "main") {
            // Serial.print("Scheduler::~Scheduler(): task \"");
            // Serial.print(t.second->name_.c_str());
            // Serial.print("\" will be deleted...");
            delete t.second;
            // Serial.println(" done.");
        }
    }

    vSemaphoreDelete(tasks_mutex_);
    exit_critical_section();
    // serial_puts("Scheduler::~Scheduler(): all tasks deleted.");
}

void Scheduler::stop() {
    vTaskEndScheduler();
}

size_t Scheduler::get_free_stack() {
#if INCLUDE_uxTaskGetStackHighWaterMark == 1
    return ::uxTaskGetStackHighWaterMark(nullptr) * sizeof(StackType_t);
#else
    return 0xffffff;
#endif
}

uint16_t Scheduler::task_add(const std::string& name, const uint16_t period, const uint8_t priority, const uint32_t stack_size, Task::func_t&& func) {
    Task* p_task { new Task(*this, next_id_, name, period, std::move(func)) };

    if (tasks_mutex_ && xSemaphoreTake(tasks_mutex_, portMAX_DELAY)) {
        tasks_[p_task->id_] = p_task;
        xSemaphoreGive(tasks_mutex_);
    } else {
        delete p_task;
        return 0;
    }

    xTaskCreate(
        [](void* p_data) {
            Task* p_task { reinterpret_cast<Task*>(p_data) };

            while (p_task->state_) {
                Timer::delay_ms(p_task->period_); // FIXME: use vTaskDelayUntil() instead?
                if (p_task->state_ == 1) {
                    p_task->func_();
                }
            }
        },
        p_task->name_.c_str(), stack_size / sizeof(StackType_t), p_task, priority, &p_task->handle_);

    return next_id_++;
}

uint16_t Scheduler::task_register(const std::string& name) {
    auto task { ::xTaskGetHandle(name.c_str()) };
    return task_register(task);
}

uint16_t Scheduler::task_register(void* task) {
    Task* p_task { new Task(*this, next_id_, ::pcTaskGetName(task), 0, nullptr) };
    p_task->handle_ = task;

    if (p_task->handle_) {
        if (tasks_mutex_ && xSemaphoreTake(tasks_mutex_, portMAX_DELAY)) {
            tasks_[p_task->id_] = p_task;
            xSemaphoreGive(tasks_mutex_);
            return next_id_++;
        }
    }
    delete p_task;
    return 0;
}

bool Scheduler::task_remove(const uint16_t task_id) {
    if (tasks_mutex_ && xSemaphoreTake(tasks_mutex_, portMAX_DELAY)) {
        Task* p_task { tasks_.at(task_id) };
        tasks_.erase(task_id);
        xSemaphoreGive(tasks_mutex_);
        delete p_task;
    } else {
        return false;
    }

    return true;
}

uint16_t Scheduler::task_get(const std::string& name) const {
    for (const auto& t : tasks_) {
        if (t.second->name_ == name) {
            return t.first;
        }
    }

    return 0xffff;
}

Task* Scheduler::task_get(const uint16_t id) const {
    return tasks_.at(id);
}

bool Scheduler::task_suspend(const uint16_t id) {
    if (tasks_.find(id) == tasks_.end()) {
        return false;
    }

    Task& task { *tasks_[id] };
    task.state_ = 2;
    vTaskSuspend(task.handle_);

    return true;
}

bool Scheduler::task_resume(const uint16_t id) {
    if (tasks_.find(id) == tasks_.end()) {
        return false;
    }

    Task& task { *tasks_[id] };
    task.state_ = 1;
    vTaskResume(task.handle_);

    return true;
}

bool Scheduler::task_wait_for(const uint16_t id, Condition& cond) {
    cond.add_task(task_get(id));

    if (!task_suspend(id)) {
        cond.remove_task(task_get(id));
        return false;
    }

    return true;
}

void Scheduler::print_task_list(CommInterface& comm) const {
    for (const auto& p_task : tasks_) {
        p_task.second->print(comm);
    }
}

void Scheduler::print_ram_usage(CommInterface& comm) const {
    const auto info { freertos::ram_usage() };

    comm.debug_printf<true>(
        PP_ARGS("free RAM: {} KB, used heap: {} KB, system free: {} KB", std::get<0>(info) / 1024UL, std::get<1>(info) / 1024UL, std::get<2>(info) / 1024UL));
}
} // namespace ctbot
