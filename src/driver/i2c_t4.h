/*
 * This file is part of the ct-Bot teensy framework.
 * Copyright (c) 2021 Timo Sandmann
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
 * @file    i2c_t4.h
 * @brief   Teensy 4.x I2C FreeRTOS driver
 * @author  Timo Sandmann
 * @date    25.03.2021
 */

#pragma once

#if defined ARDUINO_TEENSY40 || defined ARDUINO_TEENSY41
#include "arduino_freertos.h"

#include <cstdint>


namespace arduino {
class I2CT4 : public TwoWire {
    static constexpr uint8_t DEBUG_LEVEL_ { 1 }; // 0: off; 1: errors; 2: warnings; 3: info; 4: verbose

protected:
    static constexpr uint32_t RX_WATERMARK_ { 3 };
    static constexpr uint32_t TX_WATERMARK_ { 2 };

    static constexpr IMXRT_LPI2C_t* get_port(const uint8_t bus) {
        return (IMXRT_LPI2C_t*) (bus == 0 ? 0x403F0000 : (bus == 1 ? 0x403F8000 : 0x403FC000));
    }

    static constexpr const I2C_Hardware_t* get_hardware(const uint8_t bus) {
        return bus == 0 ? &TwoWire::i2c1_hardware : (bus == 1 ? &TwoWire::i2c3_hardware : &TwoWire::i2c4_hardware);
    }

    static TaskHandle_t caller_[3];

    static void isr1();
    static void isr2();
    static void isr3();

    TaskHandle_t* p_caller_;

    bool wait_idle();

public:
    I2CT4(const uint8_t bus);
    void setClock(uint32_t frequency);
    uint8_t endTransmission(uint8_t sendStop);
    uint8_t requestFrom(uint8_t address, uint8_t quantity, uint8_t sendStop);
};

namespace teensy4 {
extern I2CT4 Wire;
extern I2CT4 Wire1;
extern I2CT4 Wire2;
} // namespace teensy4

} // namespace arduino

#endif // ARDUINO_TEENSY40 || ARDUINO_TEENSY41
