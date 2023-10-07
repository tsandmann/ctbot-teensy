/* Wire Library for Teensy 4.x
 * Copyright (c) 2014-2017, Paul Stoffregen, paul@pjrc.com
 * Copyright (c) 2023, Timo Sandmann (Teensy 4.x I2C FreeRTOS driver)
 *
 * Development of this I2C library was funded by PJRC.COM, LLC by sales of
 * Teensy and related products.  Please support PJRC's efforts to develop
 * open source software by purchasing Teensy or other PJRC products.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice, development funding notice, and this permission
 * notice shall be included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
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


namespace freertos {
class I2CT4;

namespace teensy4 {
template <uint8_t>
I2CT4* get_wire();
} // namespace teensy4

class I2CT4 {
    static constexpr uint8_t DEBUG_LEVEL_ { 1 }; // 0: off; 1: errors; 2: warnings; 3: info; 4: verbose

public:
    static constexpr uint8_t MAX_BUS_ { 2 };

protected:
    static constexpr uint32_t RX_WATERMARK_ { 3 };
    static constexpr uint32_t TX_WATERMARK_ { 2 };
    static constexpr size_t BUFFER_LENGTH_ { 136 };
    static constexpr uint32_t CLOCK_STRETCH_TIMEOUT_US_ { 15'000 };
    static constexpr uint8_t CNT_SDA_PINS_ { 2 };
    static constexpr uint8_t CNT_SCL_PINS_ { 2 };
    static constexpr uint32_t PINCONFIG_ { IOMUXC_PAD_ODE | IOMUXC_PAD_SRE | IOMUXC_PAD_DSE(4) | IOMUXC_PAD_SPEED(1) | IOMUXC_PAD_PKE | IOMUXC_PAD_PUE
        | IOMUXC_PAD_PUS(3) | IOMUXC_PAD_HYS };

    struct pin_info_t {
        uint32_t mux_val;
        volatile uint32_t* select_input_reg;
        uint32_t select_val;
        uint8_t pin;
    } __attribute__((packed));

    struct hardware_t {
        uintptr_t port_addr;
        volatile uint32_t* clock_gate_reg;
        uint32_t clock_gate_mask;
        void (*irq_function)();
        uint8_t irq_number;
        pin_info_t sda_pins[CNT_SDA_PINS_];
        pin_info_t scl_pins[CNT_SCL_PINS_];
    } __attribute__((packed));

    static constexpr const hardware_t* get_hardware(uint8_t bus) {
        switch (bus) {
            case 0: return &i2c1_hw_;

            case 1: return &i2c3_hw_;

            case 2: return &i2c4_hw_;

            default: return nullptr;
        }
    }

    static I2CT4* p_instances_[MAX_BUS_ + 1];

    template <uint8_t BUS>
    FASTRUN static void isr_wire() {
        static_assert(BUS <= MAX_BUS_, "invalid BUS.");

        p_instances_[BUS]->isr();
    }

    static constinit const hardware_t i2c1_hw_;
    static constinit const hardware_t i2c3_hw_;
    static constinit const hardware_t i2c4_hw_;

    const hardware_t* const p_hardware_;
    IMXRT_LPI2C_t* const p_port_;
    uint8_t pin_sda_;
    uint8_t pin_scl_;
    TaskHandle_t caller_;
    uint8_t rx_buffer_[BUFFER_LENGTH_] = {};
    uint8_t rx_buffer_index_ {};
    uint8_t rx_buffer_length_ {};
    bool transmitting_ {};
    uint8_t tx_buffer_[BUFFER_LENGTH_ + 1] = {};
    uint8_t tx_buffer_length_ {};

    void configure_pin(const pin_info_t& pin_info) const;
    bool force_clock();
    bool wait_idle();
    void isr();

public:
    I2CT4(uint8_t bus);
    void begin();
    void setClock(uint32_t frequency);
    void setSDA(uint8_t pin);
    void setSCL(uint8_t pin);
    void beginTransmission(uint8_t address) {
        tx_buffer_[0] = (address << 1u);
        tx_buffer_length_ = 1;
        transmitting_ = true;
    }
    uint8_t endTransmission(bool sendStop);
    uint8_t requestFrom(uint8_t address, uint8_t quantity, bool sendStop);

    size_t write(uint8_t data);
    size_t write(const uint8_t* data, size_t quantity);
    int available();
    int read();
    int readBytes(uint8_t* data, size_t quantity);
    int peek();
    void flush();
};

template <uint8_t BUS>
I2CT4* get_wire() {
    static_assert(BUS <= I2CT4::MAX_BUS_, "invalid BUS.");

    static I2CT4 instance { BUS };
    return &instance;
};

} // namespace freertos

#endif // ARDUINO_TEENSY40 || ARDUINO_TEENSY41
