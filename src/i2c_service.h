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
#include <concepts>
#include <functional>


struct QueueDefinition;
typedef struct QueueDefinition* QueueHandle_t;

namespace freertos {
class I2CT4;
} // namespace freertos

class I2C_Service {
    static constexpr bool DEBUG_ { false };

#if defined ARDUINO_TEENSY40 || defined ARDUINO_TEENSY41
    using I2C_t = freertos::I2CT4;
#else
    using I2C_t = arduino::TwoWire;
#endif

    static constexpr uint8_t NUM_BUSSES_ {
#ifdef WIRE_IMPLEMENT_WIRE3
        4
#else
        3
#endif
    };

public:
    enum class I2C_Error : uint8_t {
        SUCCESS = 0,
        QUEUE_FULL,
        TIMEOUT,
        WRITE_1_FAILURE,
        WRITE_2_FAILURE,
        END_TRANSMISSION_1_FAILURE,
        END_TRANSMISSION_2_FAILURE,
        REQUEST_FROM_FAILURE,
        READ_FAILURE,
        INVALID_PARAMETERS,
        NOT_IMPLEMENTED,
    };

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
        I2C_Error error;
        TaskHandle_t caller;
        std::function<void(const bool, I2C_Transfer**)> callback;

        I2C_Transfer(uint16_t slave_addr = 0) : addr { slave_addr }, size1 {}, type1 {}, size2 {}, type2 {}, error {}, caller {}, callback {} {}
        I2C_Transfer(uint16_t slave_addr, uint8_t _size1, bool _type1, uint8_t _size2, bool _type2)
            : addr { slave_addr }, size1 { _size1 }, type1 { _type1 }, size2 { _size2 }, type2 { _type2 }, error {}, caller {}, callback {} {}
    };

protected:
    static constexpr size_t TRANSFER_QUEUE_SIZE_ { 16 };
    static constexpr uint32_t TRANSFER_QUEUE_TIMEOUT_MS_ { 200 };
    static constexpr uint32_t DEFAULT_CALLBACK_TIMEOUT_MS_ { 20 };
    static constexpr uint32_t READ_BYTES_TIMEOUT_MS_PER_BYTE_ { 10 };
    static constexpr uint32_t WRITE_BYTES_TIMEOUT_MS_PER_BYTE_ { 10 };
    static constexpr uint32_t TEST_TIMEOUT_MS_ { 50 };

    static std::array<TaskHandle_t, NUM_BUSSES_> i2c_task_;
    static std::array<QueueHandle_t, NUM_BUSSES_> i2c_queue_;
    static std::array<I2C_t*, NUM_BUSSES_> p_i2c_;
    static std::array<bool, NUM_BUSSES_> init_;
    static std::array<uint32_t, NUM_BUSSES_> freq_;

    static void finish_transfer(bool success, I2C_Transfer* transfer);
    static void finish_transfer(bool success, I2C_Transfer* transfer, [[maybe_unused]] const uint32_t id);

    static void reset(uint8_t bus);

    static void run(void* param);

    const uint8_t bus_;

    template <typename DATA>
    I2C_Error set_bits_internal(uint16_t addr, std::unsigned_integral auto reg, uint32_t bit_mask, uint32_t values) const;

public:
    static bool init(uint8_t bus, uint32_t freq, uint8_t pin_sda, uint8_t pin_scl);

    I2C_Service(uint8_t bus, uint32_t freq, uint8_t pin_sda = 255, uint8_t pin_scl = 255);

    uint8_t get_bus() const {
        return bus_;
    }

    uint32_t get_freq() const {
        return freq_[bus_];
    }

    void reset() const {
        reset(bus_);
    }

    I2C_Error read_reg(
        uint16_t addr, std::unsigned_integral auto reg, std::integral auto& data, std::function<void(bool, I2C_Transfer**)> callback = nullptr) const;
    I2C_Error read_bytes(uint16_t addr, std::unsigned_integral auto reg_addr, uint8_t* p_data, uint8_t length,
        std::function<void(bool, I2C_Transfer**)> callback = nullptr) const;

    I2C_Error write_reg(
        uint16_t addr, std::unsigned_integral auto reg, std::integral auto data, std::function<void(bool, I2C_Transfer**)> callback = nullptr) const;
    I2C_Error write_bytes(uint16_t addr, std::unsigned_integral auto reg_addr, const uint8_t* p_data, uint8_t length,
        std::function<void(bool, I2C_Transfer**)> callback = nullptr) const;

    I2C_Error set_bit(uint16_t addr, std::unsigned_integral auto reg, uint8_t bit, bool value) const;

    I2C_Error set_bits(uint16_t addr, std::unsigned_integral auto reg, uint32_t bit_mask, uint32_t values) const;

    I2C_Error test(uint16_t addr, std::function<void(bool, I2C_Transfer**)> callback = nullptr) const;
};
