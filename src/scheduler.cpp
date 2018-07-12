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
#include "ctbot.h"
#include "timer.h"
#include "comm_interface.h"

#include <arduino_fixed.h>
#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <portable/teensy.h>


namespace ctbot {

Scheduler::Task::Task(const uint16_t id, const std::string& name, const uint16_t period, task_func_t&& func, task_func_data_t&& func_data)
    : id_ { id }, period_ { period }, active_ { true }, func_ { std::move(func) }, func_data_ { std::move(func_data) }, handle_ { nullptr }, name_ { name } {}

void Scheduler::Task::print(CommInterface& comm) const {
    comm.debug_print(" \"");
    comm.debug_print(name_);
    comm.debug_print("\":\t");
    comm.debug_print("0x");
    comm.debug_print(id_, PrintBase::HEX);
    comm.debug_print('\t');
    comm.debug_print(active_ ? "  ACTIVE" : "INACTIVE");
    comm.debug_print('\t');
    if (period_) {
        comm.debug_print(period_, PrintBase::DEC);
        comm.debug_print(" ms");
    }
    comm.debug_print("\tstack free: ");
    comm.debug_print(uxTaskGetStackHighWaterMark(handle_) * sizeof(StackType_t), PrintBase::DEC);
    comm.debug_print(" byte\n");
}

Scheduler::Scheduler() : next_id_ { 0 }, tast_vector_mutex_ { xSemaphoreCreateMutex() } {
    Task* p_task { new Task(next_id_++, configIDLE_TASK_NAME, 0, nullptr, nullptr) };
    p_task->handle_ = xTaskGetIdleTaskHandle();

    vTaskSuspendAll();
    task_vector_.push_back(p_task);
    xTaskResumeAll();
}

Scheduler::~Scheduler() {
    vTaskSuspendAll();
    for (uint16_t i { 0 }; i < next_id_; ++i) {
        task_suspend(i);
    }
    xTaskResumeAll();
}

void Scheduler::stop() {
    vTaskEndScheduler();
}

uint16_t Scheduler::task_add(const std::string& name, const uint16_t period, const uint32_t stack_size, task_func_t&& func, task_func_data_t&& func_data) {
    Task* p_task { new Task(next_id_, name, period, std::move(func), std::move(func_data)) };

    if (tast_vector_mutex_ && xSemaphoreTake(tast_vector_mutex_, portMAX_DELAY)) {
        task_vector_.push_back(p_task);
        xSemaphoreGive(tast_vector_mutex_);
    } else {
        return 0;
    }

    xTaskCreate(
        [](void* p_data) {
            Task* p_task { reinterpret_cast<Task*>(p_data) };

            while (true) {
                Timer::delay_ms(p_task->period_);
                if (p_task->active_) { // FIXME: optimize?
                    p_task->func_(p_task->func_data_);
                }
            }
        },
        p_task->name_.c_str(), stack_size / sizeof(StackType_t), p_task, configMAX_PRIORITIES - 2, &p_task->handle_);

    return next_id_++;
}

uint16_t Scheduler::task_get(const std::string& name) const {
    for (uint16_t i { 0U }; i < next_id_; ++i) {
        if (task_vector_[i]->name_ == name) {
            return i;
        }
    }

    return 0xffff;
}

const Scheduler::Task* Scheduler::task_get(const uint16_t id) const {
    return task_vector_[id];
}

bool Scheduler::task_suspend(const uint16_t id) {
    if (id >= task_vector_.size()) {
        return false;
    }

    auto& task { *task_vector_[id] };
    task.active_ = false;
    vTaskSuspend(task.handle_);

    return true;
}

bool Scheduler::task_resume(const uint16_t id) {
    if (id >= task_vector_.size()) {
        return false;
    }

    auto& task { *task_vector_[id] };
    task.active_ = true;
    vTaskResume(task.handle_);

    return true;
}

void Scheduler::print_task_list(CommInterface& comm) const {
    for (const auto& p_task : task_vector_) {
        p_task->print(comm);
    }
}

void Scheduler::print_free_ram(CommInterface& comm) const {
    comm.debug_print("free RAM: ");
    comm.debug_print(freertos::free_ram() / 1024UL, PrintBase::DEC);
    comm.debug_print(" KB");
}

} // namespace ctbot
