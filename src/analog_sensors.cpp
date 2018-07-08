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
 * @file    analog_sensors.cpp
 * @brief   Analog sensor processing
 * @author  Timo Sandmann
 * @date    13.05.2018
 */

#include "analog_sensors.h"
#include "scheduler.h"
#include "timer.h"

#include <arduino_fixed.h>


namespace ctbot {

AnalogSensors::AnalogSensors()
    : last_dist_update_ { Timer::get_ms() }, last_adc_res_ { 0 }, distance_ { 0, 0 }, line_ { 0, 0 }, ldr_ { 0, 0 }, border_ { 0, 0 } {
    Scheduler::enter_critical_section();
    arduino::pinMode(CtBotConfig::DISTANCE_L_PIN, arduino::INPUT);
    arduino::pinMode(CtBotConfig::DISTANCE_R_PIN, arduino::INPUT);
    arduino::pinMode(CtBotConfig::LINE_L_PIN, arduino::INPUT);
    arduino::pinMode(CtBotConfig::LINE_R_PIN, arduino::INPUT);
    arduino::pinMode(CtBotConfig::LDR_L_PIN, arduino::INPUT);
    arduino::pinMode(CtBotConfig::LDR_R_PIN, arduino::INPUT);
    arduino::pinMode(CtBotConfig::BORDER_L_PIN, arduino::INPUT);
    arduino::pinMode(CtBotConfig::BORDER_R_PIN, arduino::INPUT);
    Scheduler::exit_critical_section();
}

void AnalogSensors::update() {
    const uint32_t now { Timer::get_ms() };
    if (now >= last_dist_update_ + 50) {
        distance_[0] = analog_read(CtBotConfig::DISTANCE_L_PIN, 10, 8);
        distance_[1] = analog_read(CtBotConfig::DISTANCE_R_PIN, 10, 8);
        last_dist_update_ = now;
    }
    line_[0] = analog_read(CtBotConfig::LINE_L_PIN, 10);
    line_[1] = analog_read(CtBotConfig::LINE_R_PIN, 10);
    ldr_[0] = analog_read(CtBotConfig::LDR_L_PIN, 10, 4);
    ldr_[1] = analog_read(CtBotConfig::LDR_R_PIN, 10, 4);
    border_[0] = analog_read(CtBotConfig::BORDER_L_PIN, 10);
    border_[1] = analog_read(CtBotConfig::BORDER_R_PIN, 10);
}

int16_t AnalogSensors::analog_read(const uint8_t pin, const uint8_t resolution, const uint8_t avg_num) {
    Scheduler::enter_critical_section();
    if (last_adc_res_ != resolution && resolution >= 8 && resolution <= 16) {
        last_adc_res_ = resolution;
        arduino::analogReadResolution(resolution);
    }
    arduino::analogReadAveraging(avg_num);
    const int16_t ret { static_cast<int16_t>(arduino::analogRead(pin)) };
    Scheduler::exit_critical_section();
    return ret;
}

} // namespace ctbot
