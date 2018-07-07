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

#include <arduino_fixed.h>


namespace ctbot {

AnalogSensors::AnalogSensors() {
    arduino::pinMode(CtBotConfig::BORDER_L_PIN, INPUT);
    arduino::pinMode(CtBotConfig::BORDER_R_PIN, INPUT);
    arduino::pinMode(CtBotConfig::DISTANCE_L_PIN, INPUT);
    arduino::pinMode(CtBotConfig::DISTANCE_R_PIN, INPUT);
    arduino::pinMode(CtBotConfig::LDR_L_PIN, INPUT);
    arduino::pinMode(CtBotConfig::LDR_R_PIN, INPUT);
    arduino::pinMode(CtBotConfig::LINE_L_PIN, INPUT);
    arduino::pinMode(CtBotConfig::LINE_R_PIN, INPUT);
}

void AnalogSensors::update() {
    data_.border[0] = analog_read(CtBotConfig::BORDER_L_PIN);
    data_.border[1] = analog_read(CtBotConfig::BORDER_R_PIN);
    data_.distance[0] = analog_read(CtBotConfig::DISTANCE_L_PIN, 8); // FIXME: every 50 ms
    data_.distance[1] = analog_read(CtBotConfig::DISTANCE_R_PIN, 8); // FIXME: every 50 ms
    data_.ldr[0] = analog_read(CtBotConfig::LDR_L_PIN, 4);
    data_.ldr[1] = analog_read(CtBotConfig::LDR_R_PIN, 4);
    data_.line[0] = analog_read(CtBotConfig::LINE_L_PIN);
    data_.line[1] = analog_read(CtBotConfig::LINE_R_PIN);
}

int16_t AnalogSensors::analog_read(const uint8_t pin, const uint8_t avg_num) const {
    arduino::analogReadAveraging(avg_num);
    return static_cast<int16_t>(arduino::analogRead(pin));
}

} // namespace ctbot
