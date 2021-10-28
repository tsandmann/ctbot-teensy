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
 * @file    sensors.h
 * @brief   Sensor abstraction layer
 * @author  Timo Sandmann
 * @date    13.05.2018
 */

#include "sensors.h"
#include "ctbot.h"

#include <chrono>


namespace ctbot {

Sensors::Sensors(CtBot& ctbot) : DigitalSensors { ctbot }, ctbot_ { ctbot } {
    ctbot_.get_ena()->on(AnalogSensors::ENA_MASK_INIT | DigitalSensors::ENA_MASK_INIT); // enable always-on ENA ports
    ctbot_.get_ena()->off(AnalogSensors::ENA_MASK | DigitalSensors::ENA_MASK); // disable ENA sensors, that aren't always-on
    ctbot_.get_ena_pwm()->on(AnalogSensors::ENA_MASK_PWM_INIT | DigitalSensors::ENA_MASK_PWM_INIT); // enable always-on PWM sensors (wheel encoder)
    ctbot_.get_ena_pwm()->off(AnalogSensors::ENA_MASK_PWM | DigitalSensors::ENA_MASK_PWM); // disable PWM sensors, that aren't always-on
}

bool Sensors::update() {
    bool ret { true };

    DigitalSensors::update();
    AnalogSensors::update();

    return ret;
}

bool Sensors::enable_sensors() {
    bool ret { true };

    ret &= ctbot_.get_ena()->on(AnalogSensors::ENA_MASK | DigitalSensors::ENA_MASK);
    if (DEBUG_ && !ret) {
        CtBot::get_instance().get_comm()->debug_print(PSTR("Sensors::enable_sensors(): get_ena()->on() failed.\r\n"), true);
    }

    ret &= ctbot_.get_ena_pwm()->on(AnalogSensors::ENA_MASK_PWM | DigitalSensors::ENA_MASK_PWM);
    if (DEBUG_ && !ret) {
        CtBot::get_instance().get_comm()->debug_print(PSTR("Sensors::enable_sensors(): get_ena_pwm()->on() failed.\r\n"), true);
    }

    return ret;
}

bool Sensors::disable_sensors() {
    bool ret { true };

    ret &= ctbot_.get_ena()->off(AnalogSensors::ENA_MASK | DigitalSensors::ENA_MASK);
    if (DEBUG_ && !ret) {
        CtBot::get_instance().get_comm()->debug_print(PSTR("Sensors::enable_sensors(): get_ena()->off() failed.\r\n"), true);
    }

    ret &= ctbot_.get_ena_pwm()->off(AnalogSensors::ENA_MASK_PWM | DigitalSensors::ENA_MASK_PWM);
    if (DEBUG_ && !ret) {
        CtBot::get_instance().get_comm()->debug_print(PSTR("Sensors::enable_sensors(): get_ena_pwm()->off() failed.\r\n"), true);
    }

    return ret;
}

bool Sensors::disable_all() {
    bool ret { true };

    ret &= ctbot_.get_ena()->off(CtBotConfig::ESP32_CONTROL_AVAILABLE ? static_cast<EnaI2cTypes>(0xff) & ~(EnaI2cTypes::ESP32_RESET | EnaI2cTypes::ESP32_PROG) :
                                                                        static_cast<EnaI2cTypes>(0xff));
    if (DEBUG_ && !ret) {
        CtBot::get_instance().get_comm()->debug_print(PSTR("Sensors::disable_all(): get_ena()->off() failed.\r\n"), true);
    }

    ret &= ctbot_.get_ena_pwm()->off(static_cast<LedTypesEna>(0xff));
    if (DEBUG_ && !ret) {
        CtBot::get_instance().get_comm()->debug_print(PSTR("Sensors::disable_all(): get_ena_pwm()->off() failed.\r\n"), true);
    }

    return ret;
}

uint32_t Sensors::get_time() const {
    using namespace std::chrono;
    return duration_cast<microseconds>(system_clock::now().time_since_epoch()).count();
}

} // namespace ctbot
