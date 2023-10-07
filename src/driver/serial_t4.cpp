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
 * @file    serial_t4.cpp
 * @brief   Teensy 4.x serialport FreeRTOS driver
 * @author  Timo Sandmann
 * @date    16.10.2021
 */

#if defined ARDUINO_TEENSY40 || defined ARDUINO_TEENSY41
#include "serial_t4.h"


extern "C" void xbar_connect(unsigned int input, unsigned int output);


namespace freertos {

SerialT4* SerialT4::p_instances_[SerialT4::MAX_PORTS_] {};

PROGMEM constinit const SerialT4::hardware_t SerialT4::serial1_hw_ = {
    .index = 0,
    .irq = IRQ_LPUART6,
    .irq_priority = IRQ_PRIORITY_,
    .port_addr = IMXRT_LPUART6_ADDRESS,
    .irq_handler = isr_serial<1>,
    .ccm_register = &CCM_CCGR3,
    .ccm_value = CCM_CCGR3_LPUART6(CCM_CCGR_ON),
    .xbar_out_trig = XBARA1_OUT_LPUART6_TRG_INPUT,
#ifdef ARDUINO_TEENSY41
    .rx_pins = { { .mux_val = 2, .select_in_reg = &IOMUXC_LPUART6_RX_SELECT_INPUT, .select_val = 1, .pin = 0 },
        { .mux_val = 2, .select_in_reg = &IOMUXC_LPUART6_RX_SELECT_INPUT, .select_val = 0, .pin = 52 } },
    .tx_pins = { { .mux_val = 2, .select_in_reg = &IOMUXC_LPUART6_TX_SELECT_INPUT, .select_val = 1, .pin = 1 },
        { .mux_val = 2, .select_in_reg = &IOMUXC_LPUART6_TX_SELECT_INPUT, .select_val = 0, .pin = 53 } },
#else
    .rx_pins = { { .mux_val = 2, .select_in_reg = &IOMUXC_LPUART6_RX_SELECT_INPUT, .select_val = 1, .pin = 0 },
        { .mux_val = 0xff, .select_in_reg = nullptr, .select_val = 0, .pin = 0xff } },
    .tx_pins = { { .mux_val = 2, .select_in_reg = &IOMUXC_LPUART6_TX_SELECT_INPUT, .select_val = 1, .pin = 1 },
        { .mux_val = 0xff, .select_in_reg = nullptr, .select_val = 0, .pin = 0xff } },
#endif // ARDUINO_TEENSY41
};

PROGMEM constinit const SerialT4::hardware_t SerialT4::serial2_hw_ = {
    .index = 1,
    .irq = IRQ_LPUART4,
    .irq_priority = IRQ_PRIORITY_,
    .port_addr = IMXRT_LPUART4_ADDRESS,
    .irq_handler = isr_serial<2>,
    .ccm_register = &CCM_CCGR1,
    .ccm_value = CCM_CCGR1_LPUART4(CCM_CCGR_ON),
    .xbar_out_trig = XBARA1_OUT_LPUART4_TRG_INPUT,
    .rx_pins = { { .mux_val = 2, .select_in_reg = &IOMUXC_LPUART4_RX_SELECT_INPUT, .select_val = 2, .pin = 7 },
        { .mux_val = 0xff, .select_in_reg = nullptr, .select_val = 0, .pin = 0xff } },
    .tx_pins = { { .mux_val = 2, .select_in_reg = &IOMUXC_LPUART4_TX_SELECT_INPUT, .select_val = 2, .pin = 8 },
        { .mux_val = 0xff, .select_in_reg = nullptr, .select_val = 0, .pin = 0xff } },
};

PROGMEM constinit const SerialT4::hardware_t SerialT4::serial3_hw_ = {
    .index = 2,
    .irq = IRQ_LPUART2,
    .irq_priority = IRQ_PRIORITY_,
    .port_addr = IMXRT_LPUART2_ADDRESS,
    .irq_handler = isr_serial<3>,
    .ccm_register = &CCM_CCGR0,
    .ccm_value = CCM_CCGR0_LPUART2(CCM_CCGR_ON),
    .xbar_out_trig = XBARA1_OUT_LPUART2_TRG_INPUT,
    .rx_pins = { { .mux_val = 2, .select_in_reg = &IOMUXC_LPUART2_RX_SELECT_INPUT, .select_val = 1, .pin = 15 },
        { .mux_val = 0xff, .select_in_reg = nullptr, .select_val = 0, .pin = 0xff } },
    .tx_pins = { { .mux_val = 2, .select_in_reg = &IOMUXC_LPUART2_TX_SELECT_INPUT, .select_val = 1, .pin = 14 },
        { .mux_val = 0xff, .select_in_reg = nullptr, .select_val = 0, .pin = 0xff } },
};

PROGMEM constinit const SerialT4::hardware_t SerialT4::serial4_hw_ = {
    .index = 3,
    .irq = IRQ_LPUART3,
    .irq_priority = IRQ_PRIORITY_,
    .port_addr = IMXRT_LPUART3_ADDRESS,
    .irq_handler = isr_serial<4>,
    .ccm_register = &CCM_CCGR0,
    .ccm_value = CCM_CCGR0_LPUART3(CCM_CCGR_ON),
    .xbar_out_trig = XBARA1_OUT_LPUART3_TRG_INPUT,
    .rx_pins = { { .mux_val = 2, .select_in_reg = &IOMUXC_LPUART3_RX_SELECT_INPUT, .select_val = 0, .pin = 16 },
        { .mux_val = 0xff, .select_in_reg = nullptr, .select_val = 0, .pin = 0xff } },
    .tx_pins = { { .mux_val = 2, .select_in_reg = &IOMUXC_LPUART3_TX_SELECT_INPUT, .select_val = 0, .pin = 17 },
        { .mux_val = 0xff, .select_in_reg = nullptr, .select_val = 0, .pin = 0xff } },
};

PROGMEM constinit const SerialT4::hardware_t SerialT4::serial5_hw_ = {
    .index = 4,
    .irq = IRQ_LPUART8,
    .irq_priority = IRQ_PRIORITY_,
    .port_addr = IMXRT_LPUART8_ADDRESS,
    .irq_handler = isr_serial<5>,
    .ccm_register = &CCM_CCGR6,
    .ccm_value = CCM_CCGR6_LPUART8(CCM_CCGR_ON),
    .xbar_out_trig = XBARA1_OUT_LPUART8_TRG_INPUT,
#ifdef ARDUINO_TEENSY41
    .rx_pins = { { .mux_val = 2, .select_in_reg = &IOMUXC_LPUART8_RX_SELECT_INPUT, .select_val = 1, .pin = 21 },
        { .mux_val = 2, .select_in_reg = &IOMUXC_LPUART8_RX_SELECT_INPUT, .select_val = 0, .pin = 46 } },
    .tx_pins = { { .mux_val = 2, .select_in_reg = &IOMUXC_LPUART8_TX_SELECT_INPUT, .select_val = 1, .pin = 20 },
        { .mux_val = 2, .select_in_reg = &IOMUXC_LPUART8_TX_SELECT_INPUT, .select_val = 0, .pin = 47 } },
#else
    .rx_pins = { { .mux_val = 2, .select_in_reg = &IOMUXC_LPUART8_RX_SELECT_INPUT, .select_val = 1, .pin = 21 },
        { .mux_val = 2, .select_in_reg = &IOMUXC_LPUART8_RX_SELECT_INPUT, .select_val = 0, .pin = 38 } },
    .tx_pins = { { .mux_val = 2, .select_in_reg = &IOMUXC_LPUART8_TX_SELECT_INPUT, .select_val = 1, .pin = 20 },
        { .mux_val = 2, .select_in_reg = &IOMUXC_LPUART8_TX_SELECT_INPUT, .select_val = 0, .pin = 39 } },
#endif // ARDUINO_TEENSY41
};

PROGMEM constinit const SerialT4::hardware_t SerialT4::serial6_hw_ = {
    .index = 5,
    .irq = IRQ_LPUART1,
    .irq_priority = IRQ_PRIORITY_,
    .port_addr = IMXRT_LPUART1_ADDRESS,
    .irq_handler = isr_serial<6>,
    .ccm_register = &CCM_CCGR5,
    .ccm_value = CCM_CCGR5_LPUART1(CCM_CCGR_ON),
    .xbar_out_trig = XBARA1_OUT_LPUART1_TRG_INPUT,
    .rx_pins = { { .mux_val = 2, .select_in_reg = nullptr, .select_val = 0, .pin = 25 },
        { .mux_val = 0xff, .select_in_reg = nullptr, .select_val = 0, .pin = 0xff } },
    .tx_pins = { { .mux_val = 2, .select_in_reg = nullptr, .select_val = 0, .pin = 24 },
        { .mux_val = 0xff, .select_in_reg = nullptr, .select_val = 0, .pin = 0xff } },
};

PROGMEM constinit const SerialT4::hardware_t SerialT4::serial7_hw_ = {
    .index = 6,
    .irq = IRQ_LPUART7,
    .irq_priority = IRQ_PRIORITY_,
    .port_addr = IMXRT_LPUART7_ADDRESS,
    .irq_handler = isr_serial<7>,
    .ccm_register = &CCM_CCGR5,
    .ccm_value = CCM_CCGR5_LPUART7(CCM_CCGR_ON),
    .xbar_out_trig = XBARA1_OUT_LPUART7_TRG_INPUT,
    .rx_pins = { { .mux_val = 2, .select_in_reg = &IOMUXC_LPUART7_RX_SELECT_INPUT, .select_val = 1, .pin = 28 },
        { .mux_val = 0xff, .select_in_reg = nullptr, .select_val = 0, .pin = 0xff } },
    .tx_pins = { { .mux_val = 2, .select_in_reg = &IOMUXC_LPUART7_TX_SELECT_INPUT, .select_val = 1, .pin = 29 },
        { .mux_val = 0xff, .select_in_reg = nullptr, .select_val = 0, .pin = 0xff } },
};

#ifdef ARDUINO_TEENSY41
PROGMEM constinit const SerialT4::hardware_t SerialT4::serial8_hw_ = {
    .index = 7,
    .irq = IRQ_LPUART5,
    .irq_priority = IRQ_PRIORITY_,
    .port_addr = IMXRT_LPUART5_ADDRESS,
    .irq_handler = isr_serial<8>,
    .ccm_register = &CCM_CCGR3,
    .ccm_value = CCM_CCGR3_LPUART5(CCM_CCGR_ON),
    .xbar_out_trig = XBARA1_OUT_LPUART5_TRG_INPUT,
    .rx_pins = { { .mux_val = 1, .select_in_reg = &IOMUXC_LPUART5_RX_SELECT_INPUT, .select_val = 1, .pin = 34 },
        { .mux_val = 2, .select_in_reg = &IOMUXC_LPUART5_RX_SELECT_INPUT, .select_val = 0, .pin = 48 } },
    .tx_pins = { { .mux_val = 1, .select_in_reg = &IOMUXC_LPUART5_TX_SELECT_INPUT, .select_val = 1, .pin = 35 },
        { .mux_val = 0xff, .select_in_reg = nullptr, .select_val = 0, .pin = 0xff } },
};
#endif // ARDUINO_TEENSY41


FLASHMEM SerialT4::SerialT4(uint8_t index)
    : p_hardware_ { get_hardware(index) }, p_port_ { reinterpret_cast<IMXRT_LPUART_t*>(p_hardware_->port_addr) }, transmitting_ {}, rx_pin_index_ {},
      tx_pin_index_ {}, rx_buffer_ {}, tx_buffer_ {}, rx_overflow_ {}, rx_last_caller_ {}, tx_last_caller_ {}, stream_helper_ { *this } {
    configASSERT(p_hardware_);
    configASSERT(p_port_);

    p_instances_[index] = this;

    if (DEBUG_LEVEL_ >= 4) {
        arduino::Serial.printf(PSTR("SerialT4::SerialT4(%u)\r\n"), p_hardware_->index);
    }
}

FLASHMEM SerialT4::~SerialT4() {
    end();
}

FLASHMEM bool SerialT4::begin(uint32_t baud, uint16_t format, size_t rx_buf_size, size_t tx_buf_size) {
    if (DEBUG_LEVEL_ >= 3) {
        arduino::Serial.printf(PSTR("SerialT4(%u)::begin(%u, %u, %u, %u)\r\n"), p_hardware_->index, baud, format, rx_buf_size, tx_buf_size);
    }

    if (!p_port_ || !p_hardware_ || format) {
        return false;
    }

    NVIC_DISABLE_IRQ(p_hardware_->irq);

    if (!rx_buffer_) {
        rx_buffer_ = ::xStreamBufferCreate(rx_buf_size ? rx_buf_size : RX_DFLT_BUF_SIZE_, 1);
    }
    configASSERT(rx_buffer_);

    if (!tx_buffer_) {
        tx_buffer_ = ::xStreamBufferCreate(tx_buf_size ? tx_buf_size : TX_DFLT_BUF_SIZE_, 1);
    }
    configASSERT(tx_buffer_);

    auto p_rx_buf { reinterpret_cast<const StaticStreamBuffer_t*>(rx_buffer_) };
    auto rx_tail { p_rx_buf->uxDummy1[0] };
    auto rx_head { p_rx_buf->uxDummy1[1] };
    const auto rx_length { p_rx_buf->uxDummy1[2] };
    const auto rx_trigger { p_rx_buf->uxDummy1[3] };
    const auto p_buf { reinterpret_cast<const uint8_t*>(p_rx_buf->pvDummy2[2]) };

    if (DEBUG_LEVEL_ >= 4) {
        arduino::Serial.printf(PSTR("SerialT4(%u)::begin(): rx_tail=%u rx_head=%u rx_length=%u rx_trigger=%u buffer_offset=%u\r\n"), p_hardware_->index,
            rx_tail, rx_head, rx_length, rx_trigger, reinterpret_cast<uintptr_t>(p_buf) - reinterpret_cast<uintptr_t>(p_rx_buf));
    }
    configASSERT(rx_head == 0);
    configASSERT(rx_tail == 0);
    configASSERT(rx_length == (rx_buf_size ? rx_buf_size : RX_DFLT_BUF_SIZE_) + 1);
    configASSERT(rx_trigger == 1);
    configASSERT(p_buf == reinterpret_cast<const uint8_t*>(p_rx_buf) + 36);
    uint8_t tmp[7] { 1, 2, 3, 4, 5, 6, 7 };
    ::xStreamBufferSend(rx_buffer_, &tmp, 7, 0);
    rx_head = p_rx_buf->uxDummy1[1];
    rx_tail = p_rx_buf->uxDummy1[0];
    configASSERT(rx_head == 7);
    configASSERT(rx_tail == 0);
    ::xStreamBufferReceive(rx_buffer_, &tmp, 3, 0);
    rx_tail = p_rx_buf->uxDummy1[0];
    configASSERT(rx_tail == 3);
    ::xStreamBufferReceive(rx_buffer_, &tmp, 4, 0);
    rx_tail = p_rx_buf->uxDummy1[0];
    configASSERT(rx_tail == 7);

    const float base { static_cast<float>(UART_CLOCK_) / static_cast<float>(baud) };
    float besterr { 1e20 };
    int bestdiv { 1 };
    int bestosr { 4 };
    for (int osr { 4 }; osr <= 32; ++osr) {
        const float div { base / static_cast<float>(osr) };
        int divint { static_cast<int>(div + 0.5f) };
        if (divint < 1) {
            divint = 1;
        } else if (divint > 8'191) {
            divint = 8'191;
        }
        float err { (static_cast<float>(divint) - div) / div };
        if (err < 0.f) {
            err = -err;
        }
        if (err <= besterr) {
            besterr = err;
            bestdiv = divint;
            bestosr = osr;
        }
    }
    if (DEBUG_LEVEL_ >= 4) {
        arduino::Serial.printf(PSTR("  baud %u: osr=%d, div=%d\r\n"), baud, bestosr, bestdiv);
    }

    transmitting_ = false;

    *p_hardware_->ccm_register = *p_hardware_->ccm_register | p_hardware_->ccm_value;

    *(portControlRegister(p_hardware_->rx_pins[rx_pin_index_].pin)) = IOMUXC_PAD_DSE(7) | IOMUXC_PAD_PKE | IOMUXC_PAD_PUE | IOMUXC_PAD_PUS(3) | IOMUXC_PAD_HYS;
    *(portConfigRegister(p_hardware_->rx_pins[rx_pin_index_].pin)) = p_hardware_->rx_pins[rx_pin_index_].mux_val;
    if (p_hardware_->rx_pins[rx_pin_index_].select_in_reg) {
        *(p_hardware_->rx_pins[rx_pin_index_].select_in_reg) = p_hardware_->rx_pins[rx_pin_index_].select_val;
    }

    *(portControlRegister(p_hardware_->tx_pins[tx_pin_index_].pin)) = IOMUXC_PAD_SRE | IOMUXC_PAD_DSE(3) | IOMUXC_PAD_SPEED(3);
    *(portConfigRegister(p_hardware_->tx_pins[tx_pin_index_].pin)) = p_hardware_->tx_pins[tx_pin_index_].mux_val;
    if (p_hardware_->tx_pins[tx_pin_index_].select_in_reg) {
        *(p_hardware_->tx_pins[tx_pin_index_].select_in_reg) = p_hardware_->tx_pins[tx_pin_index_].select_val;
    }

    p_port_->BAUD = LPUART_BAUD_OSR(bestosr - 1) | LPUART_BAUD_SBR(bestdiv) | (bestosr <= 8 ? LPUART_BAUD_BOTHEDGE : 0);
    p_port_->PINCFG = 0;

    clear();

    /* Enable the transmitter, receiver and enable receiver interrupt */
    NVIC_CLEAR_PENDING(p_hardware_->irq);
    ::attachInterruptVector(static_cast<IRQ_NUMBER_t>(p_hardware_->irq), p_hardware_->irq_handler);
    NVIC_SET_PRIORITY(p_hardware_->irq, p_hardware_->irq_priority);
    NVIC_ENABLE_IRQ(p_hardware_->irq);

    const uint8_t tx_water { static_cast<uint8_t>(tx_fifo_size_ < 16 ? tx_fifo_size_ >> 1 : 7) };
    // const uint16_t rx_fifo_size { static_cast<uint16_t>(((p_port_->FIFO >> 0) & 7) << 2) };
    const uint8_t rx_water { static_cast<uint8_t>(rx_fifo_size_ < 16 ? rx_fifo_size_ >> 1 : 7) };

    if (DEBUG_LEVEL_ >= 4) {
        arduino::Serial.printf(PSTR("SerialT4(%u)::begin(): stat=0x%x ctrl=0x%x fifo=0x%x water=0x%x\r\n"), p_hardware_->index, p_port_->STAT, p_port_->CTRL,
            p_port_->FIFO, p_port_->WATER);
        arduino::Serial.printf(PSTR("  FIFO sizes: tx=%u rx=%u\r\n"), tx_fifo_size_, rx_fifo_size_);
        arduino::Serial.printf(PSTR("  Watermark:  tx=%u rx=%u\r\n"), tx_water, rx_water);
    }

    p_port_->WATER = LPUART_WATER_RXWATER(rx_water) | LPUART_WATER_TXWATER(tx_water);
    p_port_->FIFO = p_port_->FIFO | (LPUART_FIFO_TXFE | LPUART_FIFO_RXFE);

    /* Lets configure up our CTRL register value */
    uint32_t ctrl { CTRL_TX_INACTIVE_ };

    /* Now process the bits in the Format value passed in, bits 0-2 - Parity plus 9 bit. */
    ctrl |= (format & (LPUART_CTRL_PT | LPUART_CTRL_PE)); // configure parity - turn off PT, PE, M and configure PT, PE
    if (format & 4) {
        ctrl |= LPUART_CTRL_M; // 9 bits (might include parity)
    }
    if ((format & 0xf) == 4) {
        ctrl |= LPUART_CTRL_R9T8; // 8N2 is 9 bit with 9th bit always 1
    }

    /* Bit 5 TXINVERT */
    if (format & 0x20) {
        ctrl |= LPUART_CTRL_TXINV; // tx invert
    }

    /* Write out computed CTRL */
    p_port_->CTRL = ctrl;

    /* Bit 3 10 bit - Will assume that begin already cleared it. Process some other bits which change other registers. */
    if (format & 8) {
        p_port_->BAUD = p_port_->BAUD | LPUART_BAUD_M10;
    }

    /* Bit 4 RXINVERT */
    uint32_t c { p_port_->STAT & ~LPUART_STAT_RXINV };
    if (format & 0x10) {
        c |= LPUART_STAT_RXINV; // rx invert
    }
    p_port_->STAT = c;

    /* Bit 8 can turn on 2 stop bit mote */
    if (format & 0x100) {
        p_port_->BAUD = p_port_->BAUD | LPUART_BAUD_SBNS;
    }

    if (DEBUG_LEVEL_ >= 4) {
        arduino::Serial.printf(PSTR("SerialT4(%u)::begin(): done.\r\n"), p_hardware_->index);
    }

    return true;
}

FLASHMEM void SerialT4::end() {
    p_port_->CTRL = 0; // disable the TX and RX ...

    // Not sure if this is best, but I think most IO pins default to Mode 5? which appears to be digital IO?
    *(portConfigRegister(p_hardware_->rx_pins[rx_pin_index_].pin)) = 5;
    *(portConfigRegister(p_hardware_->tx_pins[tx_pin_index_].pin)) = 5;

    clear();

    if (rx_buffer_) {
        ::vStreamBufferDelete(rx_buffer_);
        rx_buffer_ = nullptr;
    }
    if (tx_buffer_) {
        ::vStreamBufferDelete(tx_buffer_);
        tx_buffer_ = nullptr;
    }
}

FLASHMEM bool SerialT4::setRX(uint8_t pin) {
    if (pin == p_hardware_->rx_pins[rx_pin_index_].pin) {
        return true;
    }

    for (uint8_t rx_pin_new_index {}; rx_pin_new_index < CNT_RX_PINS_; ++rx_pin_new_index) {
        if (pin == p_hardware_->rx_pins[rx_pin_new_index].pin) {
            // new pin - so lets maybe reset the old pin to INPUT? and then set new pin parameters
            // only change IO pins if done after begin has been called.
            if ((*p_hardware_->ccm_register & p_hardware_->ccm_value)) {
                *(portConfigRegister(p_hardware_->rx_pins[rx_pin_index_].pin)) = 5;

                /* Now set new pin info */
                *(portControlRegister(p_hardware_->rx_pins[rx_pin_new_index].pin)) =
                    IOMUXC_PAD_DSE(7) | IOMUXC_PAD_PKE | IOMUXC_PAD_PUE | IOMUXC_PAD_PUS(3) | IOMUXC_PAD_HYS;
                *(portConfigRegister(p_hardware_->rx_pins[rx_pin_new_index].pin)) = p_hardware_->rx_pins[rx_pin_new_index].mux_val;
                if (p_hardware_->rx_pins[rx_pin_new_index].select_in_reg) {
                    *(p_hardware_->rx_pins[rx_pin_new_index].select_in_reg) = p_hardware_->rx_pins[rx_pin_new_index].select_val;
                }
            }
            rx_pin_index_ = rx_pin_new_index;

            return true;
        }
    }

    /* If we got to here and did not find a valid pin there. Maybe see if it is an XBar pin... */
    for (uint8_t i {}; i < count_pin_to_xbar_info; ++i) {
        if (pin_to_xbar_info[i].pin == pin) {
            /* So it is an XBAR pin set the XBAR */
            if (DEBUG_LEVEL_ >= 4) {
                arduino::Serial.printf(PSTR("SerialT4(%u)::setRX(): ACTS XB(%d), X(%u %u), MUX:%x\r\n"), p_hardware_->index, i,
                    pin_to_xbar_info[i].xbar_in_index, p_hardware_->xbar_out_trig, pin_to_xbar_info[i].mux_val);
            }
            CCM_CCGR2 = CCM_CCGR2 | CCM_CCGR2_XBAR1(CCM_CCGR_ON);
            xbar_connect(pin_to_xbar_info[i].xbar_in_index, p_hardware_->xbar_out_trig);

            /* We need to update port register to use this as the trigger */
            p_port_->PINCFG = LPUART_PINCFG_TRGSEL(1); // Trigger select as alternate RX

            /* Configure the pin */
            *(portControlRegister(pin)) = IOMUXC_PAD_DSE(7) | IOMUXC_PAD_PKE | IOMUXC_PAD_PUE | IOMUXC_PAD_PUS(3) | IOMUXC_PAD_HYS;
            *(portConfigRegister(pin)) = pin_to_xbar_info[i].mux_val;
            p_port_->MODIR = p_port_->MODIR | LPUART_MODIR_TXCTSE;
            if (pin_to_xbar_info[i].select_input_register) {
                *(pin_to_xbar_info[i].select_input_register) = pin_to_xbar_info[i].select_val;
            }
            if (DEBUG_LEVEL_ >= 4) {
                arduino::Serial.printf(PSTR("SerialT4(%u)::setRX(): stat:%x ctrl:%x fifo:%x water:%x\r\n"), p_hardware_->index, p_port_->STAT, p_port_->CTRL,
                    p_port_->FIFO, p_port_->WATER);
                arduino::Serial.printf(PSTR("  PINCFG: %x MODIR: %x\r\n"), p_port_->PINCFG, p_port_->MODIR);
            }

            return true;
        }
    }

    return false;
}

FLASHMEM bool SerialT4::setTX(uint8_t pin, bool opendrain) {
    uint8_t tx_pin_new_index = tx_pin_index_;

    if (pin != p_hardware_->tx_pins[tx_pin_index_].pin) {
        for (tx_pin_new_index = 0; tx_pin_new_index < CNT_TX_PINS_; ++tx_pin_new_index) {
            if (pin == p_hardware_->tx_pins[tx_pin_new_index].pin) {
                break;
            }
        }
        if (tx_pin_new_index == CNT_TX_PINS_) {
            return false;
        }
    }

    // turn on or off opendrain mode.
    // new pin - so lets maybe reset the old pin to INPUT? and then set new pin parameters
    if ((*p_hardware_->ccm_register & p_hardware_->ccm_value)) { // only do if we are already active.
        if (tx_pin_new_index != tx_pin_index_) {
            *(portConfigRegister(p_hardware_->tx_pins[tx_pin_index_].pin)) = 5;
            *(portConfigRegister(p_hardware_->tx_pins[tx_pin_new_index].pin)) = p_hardware_->tx_pins[tx_pin_new_index].mux_val;
        }
    }

    /* Now set new pin info */
    tx_pin_index_ = tx_pin_new_index;
    if (opendrain) {
        *(portControlRegister(pin)) = IOMUXC_PAD_ODE | IOMUXC_PAD_DSE(3) | IOMUXC_PAD_SPEED(3);
    } else {
        *(portControlRegister(pin)) = IOMUXC_PAD_SRE | IOMUXC_PAD_DSE(3) | IOMUXC_PAD_SPEED(3);
    }

    return true;
}

size_t SerialT4::available() const {
    return ::xStreamBufferBytesAvailable(rx_buffer_);
}

int SerialT4::peek() const {
    if (available()) {
        auto p_rx_buf { reinterpret_cast<const StaticStreamBuffer_t*>(rx_buffer_) };
        auto rx_tail { p_rx_buf->uxDummy1[0] };
        const auto p_buf { reinterpret_cast<const uint8_t*>(p_rx_buf->pvDummy2[2]) };

        return static_cast<int>(p_buf[rx_tail]);
    }

    return -1;
}

size_t SerialT4::read(void* p_data, size_t length, bool blocking) const {
    if (DEBUG_LEVEL_) {
        if (!rx_last_caller_) {
            rx_last_caller_ = ::xTaskGetCurrentTaskHandle();
        } else {
            configASSERT(rx_last_caller_ == ::xTaskGetCurrentTaskHandle());
        }
    }

    return ::xStreamBufferReceive(rx_buffer_, p_data, length, blocking ? portMAX_DELAY : 0);
}

size_t SerialT4::availableForWrite() const {
    return ::xStreamBufferSpacesAvailable(tx_buffer_);
}

size_t SerialT4::write(const void* p_data, size_t length, bool blocking) {
    if (!length) {
        return 0;
    }

    if (DEBUG_LEVEL_) {
        if (!tx_last_caller_) {
            tx_last_caller_ = ::xTaskGetCurrentTaskHandle();
        } else {
            configASSERT(tx_last_caller_ == ::xTaskGetCurrentTaskHandle());
        }
    }

    if (DEBUG_LEVEL_ >= 4) {
        arduino::Serial.printf(PSTR("SerialT4(%u)::write(X, %u): tx_buffer size=%u free=%u\r\n"), p_hardware_->index, length,
            ::xStreamBufferBytesAvailable(tx_buffer_), ::xStreamBufferSpacesAvailable(tx_buffer_));
    }

    const auto n { ::xStreamBufferSend(tx_buffer_, p_data, length, blocking ? portMAX_DELAY : 0) };
    if (DEBUG_LEVEL_ >= 4) {
        arduino::Serial.printf(PSTR("SerialT4(%u)::write(X, %u): tx_buffer size=%u free=%u\r\n"), p_hardware_->index, length,
            ::xStreamBufferBytesAvailable(tx_buffer_), ::xStreamBufferSpacesAvailable(tx_buffer_));
    }

    if (!(p_port_->CTRL & LPUART_CTRL_TIE)) {
        transmitting_ = true;
        p_port_->CTRL = p_port_->CTRL | LPUART_CTRL_TIE;
    }

    return n;
}

void SerialT4::write_direct(uint8_t c) const {
    while (!(p_port_->STAT & LPUART_STAT_TDRE)) {
        freertos::delay_ms(1);
    }
    p_port_->DATA = c;
}

void SerialT4::flush() const {
    while (transmitting_) {
        ::vTaskDelay(1);
    }
}

void SerialT4::flush_direct() const {
    while (!(p_port_->STAT & LPUART_STAT_TDRE)) {
    }
}

void SerialT4::clear() {
    ::xStreamBufferReset(rx_buffer_);
    ::xStreamBufferReset(tx_buffer_);
    rx_last_caller_ = nullptr;
    tx_last_caller_ = nullptr;
    rx_overflow_ = false;
}


FASTRUN void SerialT4::isr() {
    BaseType_t reschedule { pdFALSE };

    /* See if we have stuff to read in */
    if (p_port_->STAT & (LPUART_STAT_RDRF | LPUART_STAT_IDLE)) {
        /* If it is an idle status clear it */
        if (p_port_->STAT & LPUART_STAT_IDLE) {
            p_port_->STAT = p_port_->STAT | LPUART_STAT_IDLE;
        }

        const uint8_t avail { static_cast<uint8_t>((p_port_->WATER >> 24) & 7) };
        if (avail) {
            uint8_t data[avail];
            for (uint8_t i {}; i < avail; ++i) {
                data[i] = static_cast<uint8_t>(p_port_->DATA);
            }

            const auto n { ::xStreamBufferSendFromISR(rx_buffer_, data, avail, &reschedule) };

            if (n != avail) {
                rx_overflow_ = true;
            }
        }
    }

    /* See if we are transmitting */
    if (transmitting_) {
        const uint32_t ctrl { p_port_->CTRL };
        if ((ctrl & LPUART_CTRL_TIE) && (p_port_->STAT & LPUART_STAT_TDRE)) {
            uint32_t to_transmit { tx_fifo_size_ - ((p_port_->WATER >> 8) & 7) };
            uint8_t data[to_transmit];

            to_transmit = ::xStreamBufferReceiveFromISR(tx_buffer_, data, to_transmit, &reschedule);
            for (uint8_t i {}; i < to_transmit; ++i) {
                p_port_->DATA = data[i];
            }

            if (::xStreamBufferIsEmpty(tx_buffer_) == pdTRUE) {
                p_port_->CTRL = (p_port_->CTRL | LPUART_CTRL_TCIE) & (~LPUART_CTRL_TIE);
            }
        } else if ((ctrl & LPUART_CTRL_TCIE) && (p_port_->STAT & LPUART_STAT_TC)) {
            /* Transmission finished */
            transmitting_ = false;
            p_port_->CTRL = p_port_->CTRL & (~LPUART_CTRL_TCIE);
        }
    }

    portYIELD_FROM_ISR(reschedule);
    portDATA_SYNC_BARRIER(); // mitigate arm errata #838869
}


int SerialT4::StreamHelper::available() {
    return static_cast<int>(io_.available());
}

int SerialT4::StreamHelper::read() {
    const SerialIO& io { io_ };
    return io.read();
}

int SerialT4::StreamHelper::peek() {
    const SerialIO& io { io_ };
    return io.peek();
}

size_t SerialT4::StreamHelper::write(uint8_t b) {
    const auto res { io_.write(b) };
    if (b == '\n') {
        io_.write('\r');
    }
    return res;
}

} // namespace freertos

#endif // ARDUINO_TEENSY40 || ARDUINO_TEENSY41
