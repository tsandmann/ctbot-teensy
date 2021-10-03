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
 * @file    i2c_service.h
 * @brief   I2C FreeRTOS service
 * @author  Timo Sandmann
 * @date    25.03.2021
 */

#pragma once

#include "arduino_freertos.h"

#include <array>
#include <functional>
#include <concepts>


struct QueueDefinition;
typedef struct QueueDefinition* QueueHandle_t;

namespace arduino {
class I2CT4;
} // namespace arduino

class I2C_Service {
    static constexpr bool DEBUG_ { false };

#if defined ARDUINO_TEENSY40 || defined ARDUINO_TEENSY41
    using I2C_t = arduino::I2CT4;
#else
    using I2C_t = arduino::TwoWire;
#endif

    static constexpr uint8_t NUM_BUSSES {
#ifdef WIRE_IMPLEMENT_WIRE3
        4
#else
        3
#endif
    };

public:
    struct I2C_Transfer {
        uint16_t addr;
        uint8_t size1 : 7;
        uint8_t type1 : 1; // 1: read
        uint8_t size2 : 7;
        uint8_t type2 : 1; // 1: read
        union {
            uint32_t data;
            uint8_t* ptr;
        } data1;
        union {
            uint32_t data;
            uint8_t* ptr;
        } data2;
        uint8_t error;
        TaskHandle_t caller;
        std::function<void(const bool, I2C_Transfer**)> callback;

        I2C_Transfer(const uint16_t slave_addr = 0)
            : addr { slave_addr }, size1 { 0 }, type1 { 0 }, size2 { 0 }, type2 { 0 }, error {}, caller {}, callback {} {}
        I2C_Transfer(const uint16_t slave_addr, const uint8_t _size1, const bool _type1, const uint8_t _size2, const bool _type2)
            : addr { slave_addr }, size1 { _size1 }, type1 { _type1 }, size2 { _size2 }, type2 { _type2 }, error {}, caller {}, callback {} {}
    };

protected:
    static std::array<TaskHandle_t, NUM_BUSSES> i2c_task_;
    static std::array<QueueHandle_t, NUM_BUSSES> i2c_queue_;
    static std::array<I2C_t*, NUM_BUSSES> p_i2c_;
    static std::array<bool, NUM_BUSSES> init_;
    static std::array<uint32_t, NUM_BUSSES> freq_;

    static void finish_transfer(const bool success, I2C_Transfer* transfer);
    static void finish_transfer(const bool success, I2C_Transfer* transfer, const uint32_t id) {
        if (DEBUG_ && transfer->callback) {
            (void) id;
            printf_debug(PSTR("I2C_Service::finish_transfer(): callback for transfer %u addr=0x%x, err=%u\r\n"), id, transfer->addr, transfer->error);
        }
        finish_transfer(success, transfer);
    }

    static void run(void* param);

    const uint8_t bus_;

    template <typename DATA>
    uint8_t set_bit_internal(const uint16_t addr, std::unsigned_integral auto const reg, const uint8_t bit, const bool value) const;

public:
    static bool init(const uint8_t bus, const uint32_t freq, const uint8_t pin_sda, const uint8_t pin_scl);

    I2C_Service(const uint8_t bus, const uint32_t freq, const uint8_t pin_sda = 255, const uint8_t pin_scl = 255);

    uint8_t get_bus() const {
        return bus_;
    }

    uint32_t get_freq() const {
        return freq_[bus_];
    }

    uint8_t read_reg(const uint16_t addr, std::unsigned_integral auto const reg, std::integral auto& data,
        std::function<void(const bool, I2C_Transfer**)> callback = nullptr) const;
    uint8_t read_bytes(const uint16_t addr, std::unsigned_integral auto const reg_addr, uint8_t* p_data, const uint8_t length,
        std::function<void(const bool, I2C_Transfer**)> callback = nullptr) const;

    uint8_t write_reg(const uint16_t addr, std::unsigned_integral auto const reg, std::integral auto const data,
        std::function<void(const bool, I2C_Transfer**)> callback = nullptr) const;
    uint8_t write_bytes(const uint16_t addr, std::unsigned_integral auto const reg_addr, const uint8_t* p_data, const uint8_t length,
        std::function<void(const bool, I2C_Transfer**)> callback = nullptr) const;

    uint8_t set_bit(const uint16_t addr, std::unsigned_integral auto const reg, const uint8_t bit, const bool value) const;

    void test(const uint16_t addr, const uint32_t rx, const uint32_t tx, std::function<void(const bool, I2C_Transfer**)> callback = nullptr) const;
};
