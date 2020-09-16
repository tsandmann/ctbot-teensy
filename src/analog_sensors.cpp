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

#include "arduino_freertos.h"


namespace ctbot {

AnalogSensors::AnalogSensors() : last_adc_res_ {}, line_ { 0, 0 }, ldr_ { 0, 0 }, border_ { 0, 0 }, bat_voltage_ {} {
    Scheduler::enter_critical_section();
    arduino::pinMode(CtBotConfig::LINE_L_PIN, arduino::INPUT);
    arduino::pinMode(CtBotConfig::LINE_R_PIN, arduino::INPUT);
    if (CtBotConfig::LDR_L_PIN != 255) {
        arduino::pinMode(CtBotConfig::LDR_L_PIN, arduino::INPUT);
    }
    if (CtBotConfig::LDR_R_PIN != 255) {
        arduino::pinMode(CtBotConfig::LDR_R_PIN, arduino::INPUT);
    }
    arduino::pinMode(CtBotConfig::BORDER_L_PIN, arduino::INPUT);
    arduino::pinMode(CtBotConfig::BORDER_R_PIN, arduino::INPUT);
    arduino::pinMode(CtBotConfig::BAT_VOLTAGE_PIN, arduino::INPUT);
    arduino::analogReference(0);
    Scheduler::exit_critical_section();
}

void AnalogSensors::update() {
    line_[0] = analog_read(CtBotConfig::LINE_L_PIN, 10);
    line_[1] = analog_read(CtBotConfig::LINE_R_PIN, 10);
    ldr_[0] = analog_read(CtBotConfig::LDR_L_PIN, 10, 4);
    ldr_[1] = analog_read(CtBotConfig::LDR_R_PIN, 10, 4);
    border_[0] = analog_read(CtBotConfig::BORDER_L_PIN, 10);
    border_[1] = analog_read(CtBotConfig::BORDER_R_PIN, 10);
    bat_voltage_ = analog_read(CtBotConfig::BAT_VOLTAGE_PIN, BAT_ADC_RES, 16)
        * (3.33f * (static_cast<float>(BAT_VOLTAGE_R2 + BAT_VOLTAGE_R1) / BAT_VOLTAGE_R2) / static_cast<float>((1 << BAT_ADC_RES) - 1));
}

uint16_t AnalogSensors::analog_read(const uint8_t pin, const uint8_t resolution, const uint8_t avg_num) {
    if (pin >= 128) {
        return 0;
    }
    if (last_adc_res_ != resolution && resolution >= 8 && resolution <= 16) {
        last_adc_res_ = resolution;
        Scheduler::enter_critical_section();
        arduino::analogReadResolution(resolution);
    } else {
        Scheduler::enter_critical_section();
    }
    arduino::analogReadAveraging(avg_num);
    const uint16_t ret { static_cast<uint16_t>(arduino::analogRead(pin)) };
    Scheduler::exit_critical_section();
    return ret;
}

} // namespace ctbot
