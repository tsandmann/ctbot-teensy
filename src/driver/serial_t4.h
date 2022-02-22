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
 * @file    serial_t4.h
 * @brief   Teensy 4.x serialport FreeRTOS driver
 * @author  Timo Sandmann
 * @date    16.10.2021
 */

#pragma once

#if defined ARDUINO_TEENSY40 || defined ARDUINO_TEENSY41
#include "serial_io.h"

#include "stream_buffer.h"

#include "imxrt.h"


namespace arduino {
class SerialT4;

namespace teensy4 {
#if defined ARDUINO_TEENSY40 || defined ARDUINO_TEENSY41
extern SerialT4 Serial1;
extern SerialT4 Serial2;
extern SerialT4 Serial3;
extern SerialT4 Serial4;
extern SerialT4 Serial5;
extern SerialT4 Serial6;
extern SerialT4 Serial7;
#ifdef ARDUINO_TEENSY41
extern SerialT4 Serial8;
#endif
extern SerialIOStreamAdapter Serial;
#endif // ARDUINO_TEENSY40 || ARDUINO_TEENSY41
} // namespace teensy4


class SerialT4 : public SerialIO {
    static constexpr uint8_t DEBUG_LEVEL_ { 3 }; // 0: off; 1: errors; 2: warnings; 3: info; 4: verbose

protected:
    static constexpr uint32_t RX_DFLT_BUF_SIZE_ { 64 };
    static constexpr uint32_t TX_DFLT_BUF_SIZE_ { 64 };
    static constexpr uint8_t IRQ_PRIORITY_ { 64 };
    static constexpr uint32_t UART_CLOCK_ { 24'000'000 };

    static constexpr uint8_t CNT_RX_PINS_ { 2 };
    static constexpr uint8_t CNT_TX_PINS_ { 2 };
    static constexpr uint32_t CTRL_ENABLE_ { LPUART_CTRL_TE | LPUART_CTRL_RE | LPUART_CTRL_RIE | LPUART_CTRL_ILIE };
    static constexpr uint32_t CTRL_TX_ACTIVE_ { CTRL_ENABLE_ | LPUART_CTRL_TIE };
    static constexpr uint32_t CTRL_TX_COMPLETING_ { CTRL_ENABLE_ | LPUART_CTRL_TCIE };
    static constexpr uint32_t CTRL_TX_INACTIVE_ { CTRL_ENABLE_ };

    class StreamHelper : public Stream {
    protected:
        friend class SerialT4;

        StreamHelper(SerialIO& serial_io) : io_ { serial_io } {}
        virtual int available() override;
        virtual int read() override;
        virtual int peek() override;
        virtual size_t write(uint8_t b) override;

        SerialIO& io_;
    };

    struct hardware_t {
        uint8_t index; // 0-based
        IRQ_NUMBER_t irq;
        void (*irq_handler)();
        volatile uint32_t& ccm_register;
        const uint32_t ccm_value;
        HardwareSerial::pin_info_t rx_pins[CNT_RX_PINS_];
        HardwareSerial::pin_info_t tx_pins[CNT_TX_PINS_];
        const uint16_t irq_priority;
        const uint8_t xbar_out_lpuartX_trig_input;
    };

    static constexpr const hardware_t* get_hardware(const uint8_t index) {
        switch (index) {
            case 0: return &Serial1_hw; // Serial1
            case 1: return &Serial2_hw; // Serial2
            case 2: return &Serial3_hw; // Serial3
            case 3: return &Serial4_hw; // Serial4
            case 4: return &Serial5_hw; // Serial5
            case 5: return &Serial6_hw; // Serial6
            case 6: return &Serial7_hw; // Serial7
#ifdef ARDUINO_TEENSY41
            case 7: return &Serial8_hw; // Serial8
#endif
        }
        return nullptr;
    }

    static constexpr IMXRT_LPUART_t* get_port(const uint8_t index) {
        switch (index) {
            case 0: return (IMXRT_LPUART_t*) IMXRT_LPUART6_ADDRESS; // Serial1
            case 1: return (IMXRT_LPUART_t*) IMXRT_LPUART4_ADDRESS; // Serial2
            case 2: return (IMXRT_LPUART_t*) IMXRT_LPUART2_ADDRESS; // Serial3
            case 3: return (IMXRT_LPUART_t*) IMXRT_LPUART3_ADDRESS; // Serial4
            case 4: return (IMXRT_LPUART_t*) IMXRT_LPUART8_ADDRESS; // Serial5
            case 5: return (IMXRT_LPUART_t*) IMXRT_LPUART1_ADDRESS; // Serial6
            case 6: return (IMXRT_LPUART_t*) IMXRT_LPUART7_ADDRESS; // Serial7
#ifdef ARDUINO_TEENSY41
            case 7: return (IMXRT_LPUART_t*) IMXRT_LPUART5_ADDRESS; // Serial8
#endif
        }
        return nullptr;
    }

    static constexpr uint16_t calc_fifo_size(const uint32_t reg) {
        uint16_t tmp { static_cast<uint16_t>(1 << (((reg >> 4) & 7) + 1)) };
        if (tmp == 2) {
            tmp = 1;
        }
        return tmp;
    }

    PROGMEM static const SerialT4::hardware_t Serial1_hw;
    static void isr_Serial1() {
        teensy4::Serial1.isr();
    }
    PROGMEM static const SerialT4::hardware_t Serial2_hw;
    static void isr_Serial2() {
        teensy4::Serial2.isr();
    }
    PROGMEM static const SerialT4::hardware_t Serial3_hw;
    static void isr_Serial3() {
        teensy4::Serial3.isr();
    }
    PROGMEM static const SerialT4::hardware_t Serial4_hw;
    static void isr_Serial4() {
        teensy4::Serial4.isr();
    }
    PROGMEM static const SerialT4::hardware_t Serial5_hw;
    static void isr_Serial5() {
        teensy4::Serial5.isr();
    }
    PROGMEM static const SerialT4::hardware_t Serial6_hw;
    static void isr_Serial6() {
        teensy4::Serial6.isr();
    }
    PROGMEM static const SerialT4::hardware_t Serial7_hw;
    static void isr_Serial7() {
        teensy4::Serial7.isr();
    }
#ifdef ARDUINO_TEENSY41
    PROGMEM static const SerialT4::hardware_t Serial8_hw;
    static void isr_Serial8() {
        teensy4::Serial8.isr();
    }
#endif

    const hardware_t* const p_hardware_;
    IMXRT_LPUART_t* const p_port_;
    const uint16_t tx_fifo_size_;
    volatile bool transmitting_;
    uint8_t rx_pin_index_;
    uint8_t tx_pin_index_;

    StreamBufferHandle_t rx_buffer_;
    StreamBufferHandle_t tx_buffer_;
    bool rx_overflow_;
    mutable TaskHandle_t rx_last_caller_;
    mutable TaskHandle_t tx_last_caller_;
    StreamHelper stream_helper_;

    void isr();

public:
    SerialT4(const uint8_t index);
    virtual ~SerialT4();

    virtual bool begin(const uint32_t baud, const uint16_t format, const size_t rx_buf_size, const size_t tx_buf_size) override;

    bool setRX(const uint8_t pin);

    bool setTX(const uint8_t pin, const bool opendrain = false);
    void end();

    virtual size_t available() const override;

    virtual int peek() const override;

    virtual int read(const bool blocking) const override {
        uint8_t x;
        if (read(&x, 1, blocking) == 1) {
            return static_cast<int>(x);
        }

        return -1;
    }

    virtual size_t read(void* p_data, const size_t length, const bool blocking) const override;

    bool get_rx_overflow(const bool clear) {
        const auto overflow { rx_overflow_ };
        if (clear) {
            rx_overflow_ = false;
        }
        return overflow;
    }

    virtual size_t availableForWrite() const override;

    virtual size_t write(const void* p_data, const size_t length, const bool blocking) override;

    virtual void write_direct(const uint8_t c) const override;

    virtual void flush() const override;

    virtual void flush_direct() const override;

    virtual void clear() override;

    virtual Stream& get_stream() override {
        return stream_helper_;
    }
};

/**
 * @brief Get the serial port object
 * @param[in] port: Number of serial port
 * @return Reference to SerialIO
 */
static inline constexpr SerialIO& get_serial(const uint8_t port) {
    if (port == 1) {
        return arduino::teensy4::Serial1;
    } else if (port == 2) {
        return arduino::teensy4::Serial2;
    } else if (port == 3) {
        return arduino::teensy4::Serial3;
    } else if (port == 4) {
        return arduino::teensy4::Serial4;
    } else if (port == 5) {
        return arduino::teensy4::Serial5;
    } else if (port == 6) {
        return arduino::teensy4::Serial6;
    } else if (port == 7) {
        return arduino::teensy4::Serial7;
#ifdef ARDUINO_TEENSY41
    } else if (port == 8) {
        return arduino::teensy4::Serial8;
#endif
    } else {
        return arduino::teensy4::Serial;
    }
}
} // namespace arduino

#endif // ARDUINO_TEENSY40 || ARDUINO_TEENSY41
