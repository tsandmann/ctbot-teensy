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

#pragma once

#include "analog_sensors.h"
#include "digital_sensors.h"
#include "i2c_service.h"


namespace ctbot {

class CtBot;

/**
 * @brief Collection of all sensors
 *
 * @startuml{Sensors.png}
 *  !include sensors.puml
 *  set namespaceSeparator ::
 *  skinparam classAttributeIconSize 0
 * @enduml
 */
class Sensors : public DigitalSensors, public AnalogSensors {
    static constexpr bool DEBUG_ { false };

protected:
    CtBot& ctbot_;

public:
    /**
     * @brief Construct a new Sensors object
     * @param[in] ctbot: Reference to CtBot instance
     */
    FLASHMEM Sensors(CtBot& ctbot, I2C_Service* p_i2c_1_svc, I2C_Service* p_i2c_2_svc = nullptr);

    /**
     * @brief Update all sensors by calling the underlying update()-methods
     */
    bool update();

    /**
     * @brief Enable sensors that can be disabled for power-saving reasons
     */
    bool enable_sensors();

    /**
     * @brief Disable sensors that can be disabled for power-saving reasons
     */
    bool disable_sensors();

    /**
     * @brief Disable/shutdown all sensors
     */
    bool disable_all();

    /**
     * @return The current time in ms
     */
    uint32_t get_time() const;
};

} // namespace ctbot
