/* Teensyduino Core Library
 * http://www.pjrc.com/teensy/
 * Copyright (c) 2019 PJRC.COM, LLC.
 * Copyright (c) 2023, Timo Sandmann (Teensy 4.x serialport FreeRTOS driver)
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * 1. The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * 2. If the Software is incorporated into a build system that allows
 * selection among a list of target devices, then similar target
 * devices manufactured by PJRC.COM must be included in the list of
 * target devices and selectable in the same manner.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
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


namespace freertos {
namespace teensy4 {
template <class T, uint8_t>
void get_serial(T**);
} // namespace teensy4


class SerialT4 : public SerialIO {
public:
    static constexpr uint8_t MAX_PORTS_ {
#ifdef ARDUINO_TEENSY41
        8
#else
        7
#endif // ARDUINO_TEENSY41
    };

    static constexpr uint8_t DEBUG_LEVEL_ { 3 }; // 0: off; 1: errors; 2: warnings; 3: info; 4: verbose

protected:
    static constexpr uint32_t RX_DFLT_BUF_SIZE_ { 64 };
    static constexpr uint32_t TX_DFLT_BUF_SIZE_ { 64 };
    static constexpr uint8_t IRQ_PRIORITY_ { 64 };
    static constexpr uint32_t UART_CLOCK_ { 24'000'000 };

    static constexpr uint8_t RX_TX_FIFO_SIZE_ { 4 }; // according to IMXRT1060RM.pdf, page 2875, section 49.4.1.12.3 diagram
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

    struct pin_info_t {
        uint32_t mux_val;
        volatile uint32_t* select_in_reg;
        uint32_t select_val;
        uint8_t pin;
    } __attribute__((packed));

    struct hardware_t {
        uint8_t index; // 0-based
        uint8_t irq;
        uint16_t irq_priority;
        uintptr_t port_addr;
        void (*irq_handler)();
        volatile uint32_t* ccm_register;
        uint32_t ccm_value;
        uint8_t xbar_out_trig;
        pin_info_t rx_pins[CNT_RX_PINS_];
        pin_info_t tx_pins[CNT_TX_PINS_];
    } __attribute__((packed));

    static constexpr const hardware_t* get_hardware(uint8_t index) {
        switch (index) {
            case 0: return &serial1_hw_;
            case 1: return &serial2_hw_;
            case 2: return &serial3_hw_;
            case 3: return &serial4_hw_;
            case 4: return &serial5_hw_;
            case 5: return &serial6_hw_;
            case 6: return &serial7_hw_;
#ifdef ARDUINO_TEENSY41
            case 7: return &serial8_hw_;
#endif
        }
        return nullptr;
    }

    // static constexpr uint16_t calc_fifo_size(uint32_t reg) {
    //     uint16_t tmp { static_cast<uint16_t>(1 << (((reg >> 4) & 7) + 1)) };
    //     if (tmp == 2) {
    //         tmp = 1;
    //     }
    //     return tmp;
    // }

    static SerialT4* p_instances_[MAX_PORTS_];

    template <uint8_t PORT>
    FASTRUN static void isr_serial() {
        static_assert(PORT > 0 && PORT <= MAX_PORTS_, "invalid PORT.");

        SerialT4* p_serial { p_instances_[PORT - 1] };
        p_serial->isr();
    }

    static const hardware_t serial1_hw_;
    static const hardware_t serial2_hw_;
    static const hardware_t serial3_hw_;
    static const hardware_t serial4_hw_;
    static const hardware_t serial5_hw_;
    static const hardware_t serial6_hw_;
    static const hardware_t serial7_hw_;
#ifdef ARDUINO_TEENSY41
    static const hardware_t serial8_hw_;
#endif // ARDUINO_TEENSY41

    const hardware_t* const p_hardware_;
    IMXRT_LPUART_t* const p_port_;
    static constexpr uint16_t rx_fifo_size_ { RX_TX_FIFO_SIZE_ };
    static constexpr uint16_t tx_fifo_size_ { RX_TX_FIFO_SIZE_ };
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
    SerialT4(uint8_t index);
    virtual ~SerialT4();

    virtual bool begin(uint32_t baud, uint16_t format, size_t rx_buf_size, size_t tx_buf_size) override;

    bool setRX(uint8_t pin);

    bool setTX(uint8_t pin, bool opendrain = false);
    void end();

    virtual size_t available() const override;

    virtual int peek() const override;

    virtual int read(bool blocking) const override {
        uint8_t x;
        if (read(&x, 1, blocking) == 1) {
            return static_cast<int>(x);
        }

        return -1;
    }

    virtual size_t read(void* p_data, size_t length, bool blocking) const override;

    bool get_rx_overflow(bool clear) override {
        const auto overflow { rx_overflow_ };
        if (clear) {
            rx_overflow_ = false;
        }
        return overflow;
    }

    virtual size_t availableForWrite() const override;

    virtual size_t write(const void* p_data, size_t length, bool blocking) override;

    virtual void write_direct(uint8_t c) const override;

    virtual void flush() const override;

    virtual void flush_direct() const override;

    virtual void clear() override;

    virtual Stream& get_stream() override {
        return stream_helper_;
    }
};

namespace teensy4 {
template <class T, uint8_t PORT>
void get_serial(T** p_serial) {
    static_assert(PORT <= SerialT4::MAX_PORTS_, "invalid PORT.");

    if constexpr (PORT == 0) {
        static SerialIOStreamAdapter instance { arduino::Serial };
        *p_serial = &instance;
    } else {
        static SerialT4 instance { PORT - 1 };
        *p_serial = &instance;
    }
};
} // namespace teensy4

/**
 * @brief Get the serial port object
 * @tparam PORT: Number of serial port
 * @return Reference to SerialIO
 */
template <uint8_t PORT>
static constexpr SerialIO& get_serial() {
    SerialIO* p_serial;
    if constexpr (PORT == 0) {
        teensy4::get_serial<SerialIOStreamAdapter, PORT>(reinterpret_cast<SerialIOStreamAdapter**>(&p_serial));
    } else {
        teensy4::get_serial<SerialT4, PORT>(reinterpret_cast<SerialT4**>(&p_serial));
    }
    return *p_serial;
}
} // namespace freertos

typedef struct _pin_to_xbar_info {
    const uint8_t pin;
    const uint8_t xbar_in_index;
    const uint32_t mux_val;
    volatile uint32_t* select_input_register;
    const uint32_t select_val;
} pin_to_xbar_info_t;

extern const pin_to_xbar_info_t pin_to_xbar_info[];
extern const uint8_t count_pin_to_xbar_info;

#endif // ARDUINO_TEENSY40 || ARDUINO_TEENSY41
