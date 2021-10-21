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
 * @file    serial_t4.cpp
 * @brief   Teensy 4.x serialport FreeRTOS driver
 * @author  Timo Sandmann
 * @date    16.10.2021
 */

#if defined ARDUINO_TEENSY40 || defined ARDUINO_TEENSY41
#include "serial_t4.h"


extern "C" void xbar_connect(unsigned int input, unsigned int output);


namespace arduino {
namespace teensy4 {
SerialT4 Serial1 { 0 };
SerialT4 Serial2 { 1 };
SerialT4 Serial3 { 2 };
SerialT4 Serial4 { 3 };
SerialT4 Serial5 { 4 };
SerialT4 Serial6 { 5 };
SerialT4 Serial7 { 6 };
#ifdef ARDUINO_TEENSY41
SerialT4 Serial8 { 7 };
#endif
SerialIOStreamAdapter Serial { arduino::Serial };
} // namespace teensy4


PROGMEM const SerialT4::hardware_t SerialT4::Serial1_hw = { 0, IRQ_LPUART6, isr_Serial1, CCM_CCGR3, CCM_CCGR3_LPUART6(CCM_CCGR_ON),
#ifdef ARDUINO_TEENSY41
    { { 0, 2, &IOMUXC_LPUART6_RX_SELECT_INPUT, 1 }, { 52, 2, &IOMUXC_LPUART6_RX_SELECT_INPUT, 0 } },
    { { 1, 2, &IOMUXC_LPUART6_TX_SELECT_INPUT, 1 }, { 53, 2, &IOMUXC_LPUART6_TX_SELECT_INPUT, 0 } },
#else
    { { 0, 2, &IOMUXC_LPUART6_RX_SELECT_INPUT, 1 }, { 0xff, 0xff, nullptr, 0 } }, { { 1, 2, &IOMUXC_LPUART6_TX_SELECT_INPUT, 1 }, { 0xff, 0xff, nullptr, 0 } },
#endif // ARDUINO_TEENSY41
    IRQ_PRIORITY_, XBARA1_OUT_LPUART6_TRG_INPUT };

PROGMEM const SerialT4::hardware_t SerialT4::Serial2_hw = { 1, IRQ_LPUART4, isr_Serial2, CCM_CCGR1, CCM_CCGR1_LPUART4(CCM_CCGR_ON),
    { { 7, 2, &IOMUXC_LPUART4_RX_SELECT_INPUT, 2 }, { 0xff, 0xff, nullptr, 0 } }, { { 8, 2, &IOMUXC_LPUART4_TX_SELECT_INPUT, 2 }, { 0xff, 0xff, nullptr, 0 } },

    IRQ_PRIORITY_, XBARA1_OUT_LPUART4_TRG_INPUT };

PROGMEM const SerialT4::hardware_t SerialT4::Serial3_hw = { 2, IRQ_LPUART2, isr_Serial3, CCM_CCGR0, CCM_CCGR0_LPUART2(CCM_CCGR_ON),
    { { 15, 2, &IOMUXC_LPUART2_RX_SELECT_INPUT, 1 }, { 0xff, 0xff, nullptr, 0 } },
    { { 14, 2, &IOMUXC_LPUART2_TX_SELECT_INPUT, 1 }, { 0xff, 0xff, nullptr, 0 } }, IRQ_PRIORITY_, XBARA1_OUT_LPUART2_TRG_INPUT };

PROGMEM const SerialT4::hardware_t SerialT4::Serial4_hw = { 3, IRQ_LPUART3, isr_Serial4, CCM_CCGR0, CCM_CCGR0_LPUART3(CCM_CCGR_ON),
    { { 16, 2, &IOMUXC_LPUART3_RX_SELECT_INPUT, 0 }, { 0xff, 0xff, nullptr, 0 } },
    { { 17, 2, &IOMUXC_LPUART3_TX_SELECT_INPUT, 0 }, { 0xff, 0xff, nullptr, 0 } }, IRQ_PRIORITY_, XBARA1_OUT_LPUART3_TRG_INPUT };

PROGMEM const SerialT4::hardware_t SerialT4::Serial5_hw = { 4, IRQ_LPUART8, isr_Serial5, CCM_CCGR6, CCM_CCGR6_LPUART8(CCM_CCGR_ON),
#ifdef ARDUINO_TEENSY41
    { { 21, 2, &IOMUXC_LPUART8_RX_SELECT_INPUT, 1 }, { 46, 2, &IOMUXC_LPUART8_RX_SELECT_INPUT, 0 } },
    { { 20, 2, &IOMUXC_LPUART8_TX_SELECT_INPUT, 1 }, { 47, 2, &IOMUXC_LPUART8_TX_SELECT_INPUT, 0 } },
#else
    { { 21, 2, &IOMUXC_LPUART8_RX_SELECT_INPUT, 1 }, { 38, 2, &IOMUXC_LPUART8_RX_SELECT_INPUT, 0 } },
    { { 20, 2, &IOMUXC_LPUART8_TX_SELECT_INPUT, 1 }, { 39, 2, &IOMUXC_LPUART8_TX_SELECT_INPUT, 0 } },
#endif // ARDUINO_TEENSY41
    IRQ_PRIORITY_, XBARA1_OUT_LPUART8_TRG_INPUT };

PROGMEM const SerialT4::hardware_t SerialT4::Serial6_hw = { 5, IRQ_LPUART1, isr_Serial6, CCM_CCGR5, CCM_CCGR5_LPUART1(CCM_CCGR_ON),
    { { 25, 2, nullptr, 0 }, { 0xff, 0xff, nullptr, 0 } }, { { 24, 2, nullptr, 0 }, { 0xff, 0xff, nullptr, 0 } }, IRQ_PRIORITY_, XBARA1_OUT_LPUART1_TRG_INPUT };

PROGMEM const SerialT4::hardware_t SerialT4::Serial7_hw = { 6, IRQ_LPUART7, isr_Serial7, CCM_CCGR5, CCM_CCGR5_LPUART7(CCM_CCGR_ON),
    { { 28, 2, &IOMUXC_LPUART7_RX_SELECT_INPUT, 1 }, { 0xff, 0xff, nullptr, 0 } },
    { { 29, 2, &IOMUXC_LPUART7_TX_SELECT_INPUT, 1 }, { 0xff, 0xff, nullptr, 0 } }, IRQ_PRIORITY_, XBARA1_OUT_LPUART7_TRG_INPUT };

#ifdef ARDUINO_TEENSY41
PROGMEM const SerialT4::hardware_t SerialT4::Serial8_hw = { 7, IRQ_LPUART5, isr_Serial8, CCM_CCGR3, CCM_CCGR3_LPUART5(CCM_CCGR_ON),
    { { 34, 1, &IOMUXC_LPUART5_RX_SELECT_INPUT, 1 }, { 48, 2, &IOMUXC_LPUART5_RX_SELECT_INPUT, 0 } },
    { { 35, 1, &IOMUXC_LPUART5_TX_SELECT_INPUT, 1 }, { 0xff, 0xff, nullptr, 0 } }, IRQ_PRIORITY_, XBARA1_OUT_LPUART5_TRG_INPUT };
#endif // ARDUINO_TEENSY41

FLASHMEM SerialT4::SerialT4(const uint8_t index)
    : p_hardware_ { get_hardware(index) }, p_port_ { get_port(index) }, tx_fifo_size_ { calc_fifo_size(p_port_->FIFO) }, transmitting_ {}, rx_pin_index_ {},
      tx_pin_index_ {}, rx_buffer_ {}, tx_buffer_ {}, rx_overflow_ {}, rx_last_caller_ {}, tx_last_caller_ {}, stream_helper_ { *this } {
    configASSERT(p_hardware_);
    configASSERT(p_port_);
}

FLASHMEM SerialT4::~SerialT4() {
    end();
}

FLASHMEM bool SerialT4::begin(const uint32_t baud, const uint16_t format, const size_t rx_buf_size, const size_t tx_buf_size) {
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

    p_hardware_->ccm_register = p_hardware_->ccm_register | p_hardware_->ccm_value;

    *(portControlRegister(p_hardware_->rx_pins[rx_pin_index_].pin)) = IOMUXC_PAD_DSE(7) | IOMUXC_PAD_PKE | IOMUXC_PAD_PUE | IOMUXC_PAD_PUS(3) | IOMUXC_PAD_HYS;
    *(portConfigRegister(p_hardware_->rx_pins[rx_pin_index_].pin)) = p_hardware_->rx_pins[rx_pin_index_].mux_val;
    if (p_hardware_->rx_pins[rx_pin_index_].select_input_register) {
        *(p_hardware_->rx_pins[rx_pin_index_].select_input_register) = p_hardware_->rx_pins[rx_pin_index_].select_val;
    }

    *(portControlRegister(p_hardware_->tx_pins[tx_pin_index_].pin)) = IOMUXC_PAD_SRE | IOMUXC_PAD_DSE(3) | IOMUXC_PAD_SPEED(3);
    *(portConfigRegister(p_hardware_->tx_pins[tx_pin_index_].pin)) = p_hardware_->tx_pins[tx_pin_index_].mux_val;
    if (p_hardware_->tx_pins[tx_pin_index_].select_input_register) {
        *(p_hardware_->tx_pins[tx_pin_index_].select_input_register) = p_hardware_->tx_pins[tx_pin_index_].select_val;
    }

    p_port_->BAUD = LPUART_BAUD_OSR(bestosr - 1) | LPUART_BAUD_SBR(bestdiv) | (bestosr <= 8 ? LPUART_BAUD_BOTHEDGE : 0);
    p_port_->PINCFG = 0;

    clear();

    /* Enable the transmitter, receiver and enable receiver interrupt */
    NVIC_CLEAR_PENDING(p_hardware_->irq);
    ::attachInterruptVector(p_hardware_->irq, p_hardware_->irq_handler);
    NVIC_SET_PRIORITY(p_hardware_->irq, p_hardware_->irq_priority);
    NVIC_ENABLE_IRQ(p_hardware_->irq);

    // const uint16_t tx_fifo_size { static_cast<uint16_t>(((p_port_->FIFO >> 4) & 7) << 2) };
    const uint8_t tx_water { static_cast<uint8_t>(tx_fifo_size_ < 16 ? tx_fifo_size_ >> 1 : 7) };
    const uint16_t rx_fifo_size { static_cast<uint16_t>(((p_port_->FIFO >> 0) & 7) << 2) };
    const uint8_t rx_water { static_cast<uint8_t>(rx_fifo_size < 16 ? rx_fifo_size >> 1 : 7) };

    if (DEBUG_LEVEL_ >= 4) {
        arduino::Serial.printf(PSTR("SerialT4(%u)::begin(): stat=0x%x ctrl=0x%x fifo=0x%x water=0x%x\r\n"), p_hardware_->index, p_port_->STAT, p_port_->CTRL,
            p_port_->FIFO, p_port_->WATER);
        arduino::Serial.printf(PSTR("  FIFO sizes: tx=%u rx=%u\r\n"), tx_fifo_size_, rx_fifo_size);
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
    flush(); // wait for buffered data to send
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

FLASHMEM bool SerialT4::setRX(const uint8_t pin) {
    if (pin == p_hardware_->rx_pins[rx_pin_index_].pin) {
        return true;
    }

    for (uint8_t rx_pin_new_index {}; rx_pin_new_index < CNT_RX_PINS_; ++rx_pin_new_index) {
        if (pin == p_hardware_->rx_pins[rx_pin_new_index].pin) {
            // new pin - so lets maybe reset the old pin to INPUT? and then set new pin parameters
            // only change IO pins if done after begin has been called.
            if ((p_hardware_->ccm_register & p_hardware_->ccm_value)) {
                *(portConfigRegister(p_hardware_->rx_pins[rx_pin_index_].pin)) = 5;

                /* Now set new pin info */
                *(portControlRegister(p_hardware_->rx_pins[rx_pin_new_index].pin)) =
                    IOMUXC_PAD_DSE(7) | IOMUXC_PAD_PKE | IOMUXC_PAD_PUE | IOMUXC_PAD_PUS(3) | IOMUXC_PAD_HYS;
                *(portConfigRegister(p_hardware_->rx_pins[rx_pin_new_index].pin)) = p_hardware_->rx_pins[rx_pin_new_index].mux_val;
                if (p_hardware_->rx_pins[rx_pin_new_index].select_input_register) {
                    *(p_hardware_->rx_pins[rx_pin_new_index].select_input_register) = p_hardware_->rx_pins[rx_pin_new_index].select_val;
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
                    pin_to_xbar_info[i].xbar_in_index, p_hardware_->xbar_out_lpuartX_trig_input, pin_to_xbar_info[i].mux_val);
            }
            CCM_CCGR2 = CCM_CCGR2 | CCM_CCGR2_XBAR1(CCM_CCGR_ON);
            xbar_connect(pin_to_xbar_info[i].xbar_in_index, p_hardware_->xbar_out_lpuartX_trig_input);

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

FLASHMEM bool SerialT4::setTX(const uint8_t pin, const bool opendrain) {
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
    if ((p_hardware_->ccm_register & p_hardware_->ccm_value)) { // only do if we are already active.
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

size_t SerialT4::read(void* p_data, const size_t length, const bool blocking) const {
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

size_t SerialT4::write(const void* p_data, const size_t length, const bool blocking) {
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

void SerialT4::write_direct(const uint8_t c) const {
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
    return io_.write(b);
}

} // namespace arduino

#endif // ARDUINO_TEENSY40 || ARDUINO_TEENSY41
