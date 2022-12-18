/*
 * This file is part of the ct-Bot teensy framework.
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
#include "comm_interface.h"
#include "timer.h"

#include "driver/serial_io.h"
#include "driver/serial_t3.h"
#include "driver/serial_t4.h"

#include "pprintpp.hpp"
#include "arduino_freertos.h"


namespace ctbot {

decltype(Scheduler::p_main_task_) Scheduler::p_main_task_ {};

Scheduler::Scheduler() : next_id_ {} {
    Task* p_idle_task { new Task {
        *this, next_id_++, configIDLE_TASK_NAME, false, 0, static_cast<uint8_t>(::uxTaskPriorityGet(::xTaskGetIdleTaskHandle())), nullptr } };
    p_idle_task->std_thread_ = false;
    p_idle_task->external_ = true;
    p_idle_task->handle_.p_freertos_handle = ::xTaskGetIdleTaskHandle();

    ::vTaskSuspendAll();
    tasks_[0] = p_idle_task;
    ::xTaskResumeAll();
}

Scheduler::~Scheduler() {
    if (DEBUG_LEVEL_ > 2) {
        ::serialport_puts(PSTR("Scheduler::~Scheduler() entered.\r\n"));
    }
    enter_critical_section();
    for (auto& t : tasks_) {
        if (t.second->name_ != PSTR("main")) {
            if (DEBUG_LEVEL_ > 3) {
                ::serialport_puts(PSTR("Scheduler::~Scheduler(): task "));
                ::serialport_puts(std::to_string(t.second->id_).c_str());
                ::serialport_puts(PSTR(" @ "));
                ::serialport_puts(std::to_string(reinterpret_cast<uintptr_t>(t.second)).c_str()); // FIXME: print as hex
                ::serialport_puts(PSTR(" will be blocked..."));
            }
            t.second->state_ = 0;
            if (DEBUG_LEVEL_ > 3) {
                ::serialport_puts(PSTR(" done.\r\n"));
            }
        } else {
#ifdef __arm__
            p_main_task_ = t.second->handle_.p_thread->native_handle().get_native_handle();
#endif
        }
    }
    exit_critical_section();
    if (DEBUG_LEVEL_ > 2) {
        ::serialport_puts(PSTR("Scheduler::~Scheduler(): all tasks blocked.\r\n"));
    }

    for (auto& t : tasks_) {
        if (t.second->name_ != PSTR("main") && t.second->name_ != PSTR("YIELD")) {
            if (DEBUG_LEVEL_ > 3) {
                ::serialport_puts(PSTR("Scheduler::~Scheduler(): task \""));
                ::serialport_puts(t.second->name_.c_str());
                ::serialport_puts("\" will be deleted...\r\n");
            }
            delete t.second;
            freertos::delay_ms(20); // FIXME: necessary to wait here?
            if (DEBUG_LEVEL_ > 3) {
                ::serialport_puts("Scheduler::~Scheduler(): task delete done.\r\n");
            }
        }
    }
    if (DEBUG_LEVEL_ > 2) {
        ::serialport_puts(PSTR("Scheduler::~Scheduler(): all tasks deleted.\r\n"));
    }
}

void Scheduler::stop() {
    auto& serial { arduino::get_serial(CtBotConfig::UART_WIFI_FOR_CMD ? CtBotConfig::UART_WIFI : 0) };
    if (DEBUG_LEVEL_ > 2) {
        serial.write_direct(PSTR("Scheduler::stop()\r\n"));
    }

    if (p_main_task_) {
        if (DEBUG_LEVEL_ > 2) {
            serial.write_direct(PSTR("Scheduler::stop(): p_main_task has to be deleted...\r\n"));
        }

        ::vTaskSuspend(p_main_task_);
        if (DEBUG_LEVEL_ > 2) {
            serial.write_direct(PSTR("Scheduler::stop(): vTaskSuspend(p_main_task_) done.\r\n"));
        }

        ::vTaskDelete(p_main_task_);
        if (DEBUG_LEVEL_ > 2) {
            serial.write_direct(PSTR("Scheduler::stop(): vTaskDelete(p_main_task_) done.\r\n"));
        }
    }

    if (DEBUG_LEVEL_ > 2) {
        freertos::print_ram_usage();
        serial.write_direct(PSTR("free stack: "));
        serial.write_direct(std::to_string(::uxTaskGetStackHighWaterMark(nullptr) * sizeof(StackType_t)).c_str());
        serial.write_direct(PSTR("\r\n"));
    }

    if (CtBotConfig::IS_SIMULATED) {
        if (DEBUG_LEVEL_ > 2) {
            size_t num_tasks { ::uxTaskGetNumberOfTasks() };
            std::vector<TaskStatus_t> task_data { num_tasks };
            uint32_t total_runtime;
            num_tasks = ::uxTaskGetSystemState(task_data.data(), num_tasks, &total_runtime);
            for (size_t i {}; i < num_tasks; ++i) {
                serial.write_direct(PSTR("Task \""));
                serial.write_direct(task_data.at(i).pcTaskName);
                serial.write_direct(PSTR("\" ("));
                serial.write_direct(std::to_string(task_data.at(i).uxCurrentPriority).c_str());
                serial.write_direct(PSTR(") in state "));
                serial.write_direct(std::to_string(task_data.at(i).eCurrentState).c_str());
                serial.write_direct(PSTR("\r\n"));
            }
        }
        arduino::Serial.end();
    }

    if (DEBUG_LEVEL_ > 1) {
        serial.write_direct(PSTR("Scheduler::stop(): calling vTaskEndScheduler()...\r\n"));
    }
    ::vTaskEndScheduler();
}

size_t Scheduler::get_free_stack() const {
    return ::uxTaskGetStackHighWaterMark(nullptr) * sizeof(StackType_t);
}

size_t Scheduler::get_free_stack(uint16_t id) const {
    auto p_task { task_get(id) };
    return ::uxTaskGetStackHighWaterMark(
               p_task->std_thread_ ? free_rtos_std::gthr_freertos::get_freertos_handle(p_task->handle_.p_thread) : p_task->handle_.p_freertos_handle)
        * sizeof(StackType_t);
}

uint16_t Scheduler::task_add(const std::string_view& name, uint16_t period, uint8_t priority, uint32_t stack_size, Task::func_t&& func) {
    Task* p_task;
    {
        std::unique_lock<std::mutex> lock(task_mutex_);
        p_task = new Task { *this, next_id_++, name, false, period, priority, std::move(func) };
        tasks_[p_task->id_] = p_task;
    }

    if (stack_size / sizeof(StackType_t) < configMINIMAL_STACK_SIZE) {
        stack_size = configMINIMAL_STACK_SIZE * sizeof(StackType_t);
    }

    p_task->std_thread_ = true;
    const auto last { free_rtos_std::gthr_freertos::set_next_stacksize(stack_size) };
    p_task->handle_.p_thread = new std::thread { [p_task]() {
        using namespace std::chrono;
        auto last_exec { system_clock::now() };
        while (p_task->state_) {
            std::this_thread::sleep_until(last_exec + milliseconds(p_task->period_));
            last_exec = system_clock::now();
            if (p_task->state_ == 1) {
                p_task->func_();
            }
        }
        p_task->state_ = 10; // FIXME: necessary?
    } };
    configASSERT(p_task->handle_.p_thread);
    free_rtos_std::gthr_freertos::set_next_stacksize(last);

    free_rtos_std::gthr_freertos::set_priority(p_task->handle_.p_thread, priority);
    free_rtos_std::gthr_freertos::set_name(p_task->handle_.p_thread, p_task->name_.c_str());

    return p_task->id_;
}

uint16_t Scheduler::task_register(const std::string_view& name, const bool external) {
    auto task { ::xTaskGetHandle(name.data()) };
    return task_register(task, external);
}

uint16_t Scheduler::task_register(TaskHandle_t task, const bool external) {
    if (!task) {
        return 0;
    }

    Task* p_task;
    {
        std::unique_lock<std::mutex> lock(task_mutex_);
        p_task = new Task { *this, next_id_++, ::pcTaskGetName(task), external, 0, static_cast<uint8_t>(::uxTaskPriorityGet(task)), nullptr };
        p_task->std_thread_ = false; // FIXME: could also be a std::thread
        p_task->handle_.p_freertos_handle = task;

        if (p_task->handle_.p_freertos_handle) {
            tasks_[p_task->id_] = p_task;
            return p_task->id_;
        } else {
            --next_id_;
        }
    }

    delete p_task;
    return 0;
}

bool Scheduler::task_remove(const uint16_t task_id) {
    if (DEBUG_LEVEL_ > 3) {
        ::serialport_puts(PSTR("Scheduler::task_remove():\r\n"));
    }

    Task* p_task;
    {
        std::unique_lock<std::mutex> lock(task_mutex_);
        if (tasks_.count(task_id)) {
            if (DEBUG_LEVEL_ > 3) {
                ::serialport_puts(PSTR("Scheduler::task_remove(): task_id found.\r\n"));
            }
            p_task = tasks_[task_id];
            tasks_.erase(task_id);
        } else {
            return false;
        }
    }
    delete p_task;

    if (DEBUG_LEVEL_ > 3) {
        ::serialport_puts(PSTR("Scheduler::task_remove() done.\r\n"));
    }
    return true;
}

uint16_t Scheduler::task_get(const std::string_view& name) const {
    std::unique_lock<std::mutex> lock(task_mutex_); // FIXME: lock necessary?
    for (const auto& t : tasks_) {
        if (t.second->name_ == name) {
            return t.first;
        }
    }

    return 0xffff;
}

Task* Scheduler::task_get(const uint16_t id) const {
    std::unique_lock<std::mutex> lock(task_mutex_); // FIXME: lock necessary?
    if (!tasks_.count(id)) {
        return nullptr;
    }

    return tasks_.at(id);
}

bool Scheduler::task_suspend(const uint16_t id) {
    std::unique_lock<std::mutex> lock(task_mutex_); // FIXME: lock necessary?
    if (!tasks_.count(id)) {
        return false;
    }

    Task& task { *tasks_[id] };
    task.state_ = 2;
    free_rtos_std::gthr_freertos::suspend(task.handle_.p_thread);

    return true;
}

bool Scheduler::task_resume(const uint16_t id) {
    std::unique_lock<std::mutex> lock(task_mutex_); // FIXME: lock necessary?
    if (!tasks_.count(id)) {
        return false;
    }

    Task& task { *tasks_[id] };
    task.state_ = 1;
    free_rtos_std::gthr_freertos::resume(task.handle_.p_thread);

    return true;
}

bool Scheduler::task_join(const uint16_t id) {
    std::unique_lock<std::mutex> lock(task_mutex_); // FIXME: lock necessary?
    if (!tasks_.count(id)) {
        return false;
    }

    tasks_[id]->handle_.p_thread->join();
    return true;
}

bool Scheduler::task_set_finished(const uint16_t id) {
    std::unique_lock<std::mutex> lock(task_mutex_); // FIXME: lock necessary?
    if (!tasks_.count(id)) {
        return false;
    }

    tasks_[id]->state_ = 0;
    return true;
}

void Scheduler::print_task_list(CommInterface& comm) const {
    for (const auto& p_task : tasks_) {
        p_task.second->print(comm);
    }
}

void Scheduler::print_ram_usage(CommInterface& comm) const {
    const auto info1 { freertos::ram1_usage() };
    const auto info2 { freertos::ram2_usage() };
    const auto info3 { freertos::ram3_usage() };

    comm.debug_printf<true>(PP_ARGS("RAM1 size: \x1b[1m{} KB\x1b[0m\x1b[37;40m, free RAM1: \x1b[1m\x1b[32;40m{} KB\x1b[0m\x1b[37;40m, ",
        std::get<6>(info1) / 1024UL, std::get<0>(info1) / 1024UL));
    comm.debug_printf<true>(PP_ARGS("data used: \x1b[1m\x1b[31;40m{} KB\x1b[0m\x1b[37;40m, ", std::get<1>(info1) / 1'024UL));
    comm.debug_printf<true>(PP_ARGS("bss used: \x1b[1m{} KB\x1b[0m\x1b[37;40m,\r\n     heap used: \x1b[1m{} KB\x1b[0m\x1b[37;40m", std::get<2>(info1) / 1'024UL,
        std::get<3>(info1) / 1'024UL));
    comm.debug_printf<true>(PP_ARGS(", system free: \x1b[1m{} KB\x1b[0m\x1b[37;40m\r\n", std::get<4>(info1) / 1'024UL));

    comm.debug_printf<true>(PP_ARGS("RAM2 size: \x1b[1m{} KB\x1b[0m\x1b[37;40m, free RAM2: \x1b[1m\x1b[32;40m{} KB\x1b[0m\x1b[37;40m, ",
        std::get<1>(info2) / 1024UL, std::get<0>(info2) / 1024UL));
    comm.debug_printf<true>(PP_ARGS("used RAM2: \x1b[1m\x1b[31;40m{} KB\x1b[0m\x1b[37;40m\r\n", (std::get<1>(info2) - std::get<0>(info2)) / 1'024UL));

#ifdef ARDUINO_TEENSY41
    if (std::get<1>(info3)) {
        comm.debug_printf<true>(PP_ARGS("eRAM size: \x1b[1m{} KB\x1b[0m\x1b[37;40m, free eRAM: \x1b[1m\x1b[32;40m{} KB\x1b[0m\x1b[37;40m, ",
            std::get<1>(info3) / 1024UL, std::get<0>(info3) / 1024UL));
        comm.debug_printf<true>(PP_ARGS("used eRAM: \x1b[1m\x1b[31;40m{} KB\x1b[0m\x1b[37;40m\r\n", (std::get<1>(info3) - std::get<0>(info3)) / 1'024UL));
    }
#endif // ARDUINO_TEENSY41
}

std::unique_ptr<std::vector<std::pair<TaskHandle_t, float>>> Scheduler::get_runtime_stats(SchedulerStatContext* p_context, bool sort) const {
    enter_critical_section();
    size_t num_tasks { ::uxTaskGetNumberOfTasks() };
    std::vector<TaskStatus_t> task_data;
    task_data.resize(num_tasks);
    uint32_t total_runtime;
    num_tasks = ::uxTaskGetSystemState(task_data.data(), num_tasks, &total_runtime);
    exit_critical_section();

    auto current_runtimes { std::make_unique<std::vector<std::pair<TaskHandle_t, float>>>() };

    for (size_t i {}; i < num_tasks; ++i) {
        current_runtimes->push_back(std::make_pair(task_data[i].xHandle,
            (task_data[i].ulRunTimeCounter - p_context->last_runtimes_[task_data[i].xHandle]) * 100.f / (total_runtime - p_context->last_total_runtime_)));
        p_context->last_runtimes_[task_data[i].xHandle] = task_data[i].ulRunTimeCounter;
    }
    p_context->last_total_runtime_ = total_runtime;

    if (sort) {
        auto cmp = [](std::pair<TaskHandle_t, float> const& a, std::pair<void*, float> const& b) { return a.second >= b.second; };
        std::sort(current_runtimes->begin(), current_runtimes->end(), cmp);
    }

    return current_runtimes;
}

} // namespace ctbot
