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
 * @file    i2c_t4.cpp
 * @brief   Teensy 4.x I2C FreeRTOS driver
 * @author  Timo Sandmann
 * @date    25.03.2021
 */

#if defined ARDUINO_TEENSY40 || defined ARDUINO_TEENSY41
#include "i2c_t4.h"

#include <cstring>


namespace freertos {

I2CT4* I2CT4::p_instances_[I2CT4::MAX_BUS_ + 1] {};


PROGMEM constinit const I2CT4::hardware_t I2CT4::i2c1_hw_ {
    IMXRT_LPI2C1_ADDRESS,
    &CCM_CCGR2,
    CCM_CCGR2_LPI2C1(CCM_CCGR_ON),
    isr_wire<0>,
    IRQ_LPI2C1,
    { { .mux_val = 3 | 0x10, .select_input_reg = &IOMUXC_LPI2C1_SDA_SELECT_INPUT, .select_val = 1, .pin = 18 },
        { .mux_val = 0xff, .select_input_reg = nullptr, .select_val = 0, .pin = 0xff } },
    { { .mux_val = 3 | 0x10, .select_input_reg = &IOMUXC_LPI2C1_SCL_SELECT_INPUT, .select_val = 1, .pin = 19 },
        { .mux_val = 0xff, .select_input_reg = nullptr, .select_val = 0, .pin = 0xff } },
};

PROGMEM constinit const I2CT4::hardware_t I2CT4::i2c3_hw_ {
    IMXRT_LPI2C3_ADDRESS,
    &CCM_CCGR2,
    CCM_CCGR2_LPI2C3(CCM_CCGR_ON),
    isr_wire<1>,
    IRQ_LPI2C3,
#ifdef ARDUINO_TEENSY41
    { { .mux_val = 1 | 0x10, .select_input_reg = &IOMUXC_LPI2C3_SDA_SELECT_INPUT, .select_val = 2, .pin = 17 },
        { .mux_val = 2 | 0x10, .select_input_reg = &IOMUXC_LPI2C3_SDA_SELECT_INPUT, .select_val = 1, .pin = 44 } },
    { { .mux_val = 1 | 0x10, .select_input_reg = &IOMUXC_LPI2C3_SCL_SELECT_INPUT, .select_val = 2, .pin = 16 },
        { .mux_val = 2 | 0x10, .select_input_reg = &IOMUXC_LPI2C3_SCL_SELECT_INPUT, .select_val = 1, .pin = 45 } },
#else // T4 and ARDUINO_TEENSY_MICROMOD
    { { .mux_val = 1 | 0x10, .select_input_reg = &IOMUXC_LPI2C3_SDA_SELECT_INPUT, .select_val = 2, .pin = 17 },
        { .mux_val = 2 | 0x10, .select_input_reg = &IOMUXC_LPI2C3_SDA_SELECT_INPUT, .select_val = 1, .pin = 36 } },
    { { .mux_val = 1 | 0x10, .select_input_reg = &IOMUXC_LPI2C3_SCL_SELECT_INPUT, .select_val = 2, .pin = 16 },
        { .mux_val = 2 | 0x10, .select_input_reg = &IOMUXC_LPI2C3_SCL_SELECT_INPUT, .select_val = 1, .pin = 37 } },
#endif
};

PROGMEM constinit const I2CT4::hardware_t I2CT4::i2c4_hw_ {
    IMXRT_LPI2C4_ADDRESS,
    &CCM_CCGR6,
    CCM_CCGR6_LPI2C4_SERIAL(CCM_CCGR_ON),
    isr_wire<2>,
    IRQ_LPI2C4,
    { { .mux_val = 0 | 0x10, .select_input_reg = &IOMUXC_LPI2C4_SDA_SELECT_INPUT, .select_val = 1, .pin = 25 },
        { .mux_val = 0xff, .select_input_reg = nullptr, .select_val = 0, .pin = 0xff } },
    { { .mux_val = 0 | 0x10, .select_input_reg = &IOMUXC_LPI2C4_SCL_SELECT_INPUT, .select_val = 1, .pin = 24 },
        { .mux_val = 0xff, .select_input_reg = nullptr, .select_val = 0, .pin = 0xff } },
};


FLASHMEM I2CT4::I2CT4(uint8_t bus)
    : p_hardware_ { get_hardware(bus) }, p_port_ { reinterpret_cast<IMXRT_LPI2C_t*>(p_hardware_->port_addr) }, pin_sda_ { 255 }, pin_scl_ { 255 }, caller_ {} {
    p_instances_[bus] = this;

    NVIC_DISABLE_IRQ(p_hardware_->irq_number);
    NVIC_CLEAR_PENDING(p_hardware_->irq_number);
    ::attachInterruptVector(static_cast<IRQ_NUMBER_t>(p_hardware_->irq_number), p_hardware_->irq_function);
    NVIC_SET_PRIORITY(p_hardware_->irq_number, 128);
}

FLASHMEM void I2CT4::begin() {
    p_port_->MDER = 0;
    NVIC_ENABLE_IRQ(p_hardware_->irq_number);

    // 24 MHz clock
    CCM_CSCDR2 = (CCM_CSCDR2 & ~CCM_CSCDR2_LPI2C_CLK_PODF(63)) | CCM_CSCDR2_LPI2C_CLK_SEL;
    *p_hardware_->clock_gate_reg |= p_hardware_->clock_gate_mask;
    p_port_->MCR = LPI2C_MCR_RST;
    setClock(100'000);

    transmitting_ = false;
}

FLASHMEM void I2CT4::configure_pin(const pin_info_t& pin_info) const {
    *(portControlRegister(pin_info.pin)) = PINCONFIG_;
    *(portConfigRegister(pin_info.pin)) = pin_info.mux_val;

    if (pin_info.select_input_reg) {
        *(pin_info.select_input_reg) = pin_info.select_val;
    }
}

FLASHMEM void I2CT4::setSDA(uint8_t pin) {
    if (pin == pin_sda_) {
        return;
    }

    uint8_t new_index {};
    while (true) {
        const uint8_t sda_pin { p_hardware_->sda_pins[new_index].pin };
        if (sda_pin == 255) {
            return;
        }
        if (sda_pin == pin) {
            break;
        }
        if (++new_index >= sizeof(p_hardware_->sda_pins)) {
            return;
        }
    }

    if (pin_sda_ == 255 || (*p_hardware_->clock_gate_reg & p_hardware_->clock_gate_mask)) {
        if (pin_sda_ != 255) {
            // disable old pin, hard to know what to go back to?
            *(portConfigRegister(pin_sda_)) = 5;
        }
        // setup new one...
        configure_pin(p_hardware_->sda_pins[new_index]);
    }

    pin_sda_ = pin;
}

FLASHMEM void I2CT4::setSCL(uint8_t pin) {
    if (pin == pin_scl_) {
        return;
    }

    uint8_t new_index {};
    while (true) {
        const uint8_t scl_pin { p_hardware_->scl_pins[new_index].pin };
        if (scl_pin == 255) {
            return;
        }
        if (scl_pin == pin) {
            break;
        }
        if (++new_index >= sizeof(p_hardware_->scl_pins)) {
            return;
        }
    }

    if (pin_scl_ == 255 || (*p_hardware_->clock_gate_reg & p_hardware_->clock_gate_mask)) {
        if (pin_scl_ != 255) {
            // disable old pin, hard to know what to go back to?
            *(portConfigRegister(pin_scl_)) = 5;
        }
        // setup new one...
        configure_pin(p_hardware_->scl_pins[new_index]);
    }

    pin_scl_ = pin;
}

void I2CT4::setClock(uint32_t frequency) {
    p_port_->MCR = 0;
    if (frequency < 400'000) {
        // 100 kHz
        p_port_->MCCR0 = LPI2C_MCCR0_CLKHI(55) | LPI2C_MCCR0_CLKLO(59) | LPI2C_MCCR0_DATAVD(25) | LPI2C_MCCR0_SETHOLD(40);
        p_port_->MCFGR1 = LPI2C_MCFGR1_PRESCALE(1);
        p_port_->MCFGR2 = LPI2C_MCFGR2_FILTSDA(5) | LPI2C_MCFGR2_FILTSCL(5) | LPI2C_MCFGR2_BUSIDLE(3'000); // idle timeout 250 us
        p_port_->MCFGR3 = LPI2C_MCFGR3_PINLOW(CLOCK_STRETCH_TIMEOUT_US_ * 12 / 256 + 1);
    } else if (frequency < 1'000'000) {
        // 400 kHz
        p_port_->MCCR0 = LPI2C_MCCR0_CLKHI(26) | LPI2C_MCCR0_CLKLO(28) | LPI2C_MCCR0_DATAVD(12) | LPI2C_MCCR0_SETHOLD(18);
        p_port_->MCFGR1 = LPI2C_MCFGR1_PRESCALE(0);
        p_port_->MCFGR2 = LPI2C_MCFGR2_FILTSDA(2) | LPI2C_MCFGR2_FILTSCL(2) | LPI2C_MCFGR2_BUSIDLE(3'600); // idle timeout 150 us
        p_port_->MCFGR3 = LPI2C_MCFGR3_PINLOW(CLOCK_STRETCH_TIMEOUT_US_ * 24 / 256 + 1);
    } else {
        // 1 MHz
        p_port_->MCCR0 = LPI2C_MCCR0_CLKHI(9) | LPI2C_MCCR0_CLKLO(10) | LPI2C_MCCR0_DATAVD(4) | LPI2C_MCCR0_SETHOLD(7);
        p_port_->MCFGR1 = LPI2C_MCFGR1_PRESCALE(0);
        p_port_->MCFGR2 = LPI2C_MCFGR2_FILTSDA(1) | LPI2C_MCFGR2_FILTSCL(1) | LPI2C_MCFGR2_BUSIDLE(2'400); // idle timeout 100 us
        p_port_->MCFGR3 = LPI2C_MCFGR3_PINLOW(CLOCK_STRETCH_TIMEOUT_US_ * 24 / 256 + 1);
    }
    p_port_->MCCR1 = p_port_->MCCR0;
    p_port_->MCFGR0 = 0;
    p_port_->MFCR = LPI2C_MFCR_RXWATER(1) | LPI2C_MFCR_TXWATER(1);
    p_port_->MCR = LPI2C_MCR_MEN;

    p_port_->MFCR = LPI2C_MFCR_RXWATER(RX_WATERMARK_) | LPI2C_MFCR_TXWATER(TX_WATERMARK_);
}

bool I2CT4::force_clock() {
    bool ret {};
    const uint32_t sda_mask { digitalPinToBitMask(pin_sda_) };
    const uint32_t scl_mask { digitalPinToBitMask(pin_scl_) };
    const uint32_t mux_sda { *(portConfigRegister(pin_sda_)) };
    const uint32_t mux_scl { *(portConfigRegister(pin_scl_)) };
    // take control of pins with GPIO
    *portConfigRegister(pin_sda_) = 5 | 0x10;
    *portSetRegister(pin_sda_) = sda_mask;
    *portModeRegister(pin_sda_) |= sda_mask;
    *portConfigRegister(pin_scl_) = 5 | 0x10;
    *portSetRegister(pin_scl_) = scl_mask;
    *portModeRegister(pin_scl_) |= scl_mask;
    delayMicroseconds(10);

    for (int i {}; i < 9; ++i) {
        if ((*portInputRegister(pin_sda_) & sda_mask) && (*portInputRegister(pin_scl_) & scl_mask)) {
            // success, both pins are high
            ret = true;
            break;
        }
        *portClearRegister(pin_scl_) = scl_mask;
        delayMicroseconds(5);
        *portSetRegister(pin_scl_) = scl_mask;
        delayMicroseconds(5);
    }
    // return control of pins to I2C
    *(portConfigRegister(pin_sda_)) = mux_sda;
    *(portConfigRegister(pin_scl_)) = mux_scl;
    return ret;
}

bool I2CT4::wait_idle() {
    elapsedMillis timeout;
    while (true) {
        const uint32_t status { p_port_->MSR }; // pg 2899 & 2892
        if (!(status & LPI2C_MSR_BBF)) {
            break; // bus is available
        }
        if (status & LPI2C_MSR_MBF) {
            break; // we already have bus control
        }

        if (timeout > 16) {
            if (DEBUG_LEVEL_) {
                printf_debug(PSTR("I2CT4::wait_idle(): timeout waiting for idle, MSR = %x\r\n"), status);
            }
            if (force_clock()) {
                break;
            }
            if (DEBUG_LEVEL_) {
                printf_debug(PSTR("I2CT4::wait_idle(): unable to get control of I2C bus\r\n"));
            }
            return false;
        }

        ::vTaskDelay(pdMS_TO_TICKS(1));
    }
    p_port_->MSR = 0x7F00; // clear all prior flags
    return true;
}

uint8_t I2CT4::endTransmission(bool sendStop) {
    configASSERT(caller_ == nullptr);

    uint32_t tx_len { tx_buffer_length_ };
    if (!tx_len) {
        return 1; // no address for transmit
    }
    if (!wait_idle()) {
        if (DEBUG_LEVEL_) {
            printf_debug(PSTR("eT wait_idle() failed\r\n"));
        }
        return 2;
    }
    uint32_t tx_index {}; // 0=start, 1=addr, 2-(N-1)=data, N=stop
    elapsedMillis timeout;
    caller_ = ::xTaskGetCurrentTaskHandle();
    if (DEBUG_LEVEL_ >= 3) {
        printf_debug(PSTR("eT for 0x%x\r\n"), reinterpret_cast<uintptr_t>(*p_caller_));
    }
    bool isr_timeout { false };
    while (true) {
        // transmit stuff, if we haven't already
        if (tx_index <= tx_len) {
            uint32_t fifo_used { p_port_->MFSR & 7 }; // pg 2914
            while (fifo_used < 4) {
                if (tx_index == 0) {
                    p_port_->MTDR = LPI2C_MTDR_CMD_START | tx_buffer_[0];
                    tx_index = 1;
                } else if (tx_index < tx_len) {
                    p_port_->MTDR = LPI2C_MTDR_CMD_TRANSMIT | tx_buffer_[tx_index++];
                } else {
                    if (sendStop) {
                        p_port_->MTDR = LPI2C_MTDR_CMD_STOP;
                    }
                    tx_index++;
                    break;
                }
                fifo_used++;
            }
        }
        // monitor status
        const uint32_t status { p_port_->MSR }; // pg 2884 & 2891
        if (status & LPI2C_MSR_ALF) {
            p_port_->MCR = p_port_->MCR | (LPI2C_MCR_RTF | LPI2C_MCR_RRF); // clear FIFOs

            p_port_->MIER = 0;
            if (DEBUG_LEVEL_) {
                printf_debug(PSTR("eT abort (LPI2C_MSR_ALF) for 0x%x\r\n"), reinterpret_cast<uintptr_t>(*p_caller_));
            }
            caller_ = nullptr;
            transmitting_ = false;
            return 3; // we lost bus arbitration to another master
        }
        if (status & LPI2C_MSR_NDF) {
            p_port_->MCR = p_port_->MCR | (LPI2C_MCR_RTF | LPI2C_MCR_RRF); // clear FIFOs
            p_port_->MTDR = LPI2C_MTDR_CMD_STOP;

            p_port_->MIER = 0;
            if (DEBUG_LEVEL_) {
                printf_debug(PSTR("eT abort (LPI2C_MSR_NDF) for 0x%x\r\n"), reinterpret_cast<uintptr_t>(*p_caller_));
            }
            caller_ = nullptr;
            transmitting_ = false;
            return 4; // NACK
        }
        if ((status & LPI2C_MSR_PLTF) || timeout > 50) {
            p_port_->MCR = p_port_->MCR | (LPI2C_MCR_RTF | LPI2C_MCR_RRF); // clear FIFOs
            p_port_->MTDR = LPI2C_MTDR_CMD_STOP; // try to send a stop

            p_port_->MIER = 0;
            if (DEBUG_LEVEL_) {
                printf_debug(PSTR("eT abort (LPI2C_MSR_PLTF | timeout) for 0x%x\r\n"), reinterpret_cast<uintptr_t>(*p_caller_));
                printf_debug(PSTR("\ttimeout=%u ms\r\n"), static_cast<uint32_t>(timeout));
            }
            caller_ = nullptr;
            transmitting_ = false;
            return 5; // clock stretched too long or generic timeout
        }

        if (DEBUG_LEVEL_ >= 2 && isr_timeout) {
            isr_timeout = false;
            printf_debug(PSTR("eT ISR TIMEOUT recovered\r\n"));
        }

        // are we done yet?
        if (tx_index > tx_len) {
            const uint32_t tx_fifo { p_port_->MFSR & 7 };
            if (tx_fifo == 0 && ((status & LPI2C_MSR_SDF) || !sendStop)) {
                p_port_->MIER = 0;
                if (DEBUG_LEVEL_ >= 3) {
                    printf_debug(PSTR("eT done for 0x%x\r\n"), reinterpret_cast<uintptr_t>(*p_caller_));
                }
                caller_ = nullptr;
                transmitting_ = false;
                return 0;
            }
        }

        const uint32_t tx_fifo { p_port_->MFSR & 7 };
        if (tx_fifo >= 4 || (tx_index > tx_len && tx_fifo > 2)) { // FIXME: parameter for 4? | test second condition
            if (DEBUG_LEVEL_ >= 4) {
                printf_debug(PSTR("eT waiting for irq, tx_fifo=%u\r\n"), tx_fifo);
            }
            p_port_->MIER = LPI2C_MIER_TDIE;
            uint32_t status;
            if (::xTaskNotifyWait(DEBUG_LEVEL_ >= 2 ? 2 : 0, 0, DEBUG_LEVEL_ >= 2 ? &status : nullptr, pdMS_TO_TICKS(1)) == pdFALSE) {
                p_port_->MIER = 0;
                if (DEBUG_LEVEL_ >= 2) {
                    isr_timeout = true;
                    printf_debug(PSTR("eT ISR TIMEOUT\r\n"));
                }
            } else if (DEBUG_LEVEL_ >= 2 && !(status & 2)) {
                printf_debug(PSTR("eT unexpected ISR return value: 0x%x\r\n"), status);
            }
            if (DEBUG_LEVEL_ >= 4) {
                const uint32_t tx_fifo { p_port_->MFSR & 7 };
                printf_debug(PSTR("eT tx_fifo=%u\r\n"), tx_fifo);
                (void) tx_fifo;
            }
        }
    }
    configASSERT(false);
}

size_t I2CT4::write(uint8_t data) {
    if (!transmitting_) {
        return 0;
    }

    if (tx_buffer_length_ >= BUFFER_LENGTH_ + 1) {
        return 0;
    }
    tx_buffer_[tx_buffer_length_++] = data;

    return 1;
}

size_t I2CT4::write(const uint8_t* data, size_t quantity) {
    if (!transmitting_) {
        return 0;
    }

    const size_t avail { BUFFER_LENGTH_ + 1 - tx_buffer_length_ };
    if (quantity > avail) {
        quantity = avail;
    }
    std::memcpy(tx_buffer_ + tx_buffer_length_, data, quantity);
    tx_buffer_length_ += quantity;

    return quantity;
}

int I2CT4::available() {
    return rx_buffer_length_ - rx_buffer_index_;
}

int I2CT4::read() {
    if (rx_buffer_index_ >= rx_buffer_length_) {
        return -1;
    }
    return rx_buffer_[rx_buffer_index_++];
}

int I2CT4::readBytes(uint8_t* data, size_t quantity) {
    const auto avail { available() };
    if (avail < 1) {
        return avail;
    }
    if (quantity > static_cast<size_t>(avail)) {
        quantity = avail;
    }

    std::memcpy(data, rx_buffer_ + rx_buffer_index_, quantity);
    rx_buffer_index_ += quantity;

    return quantity;
}

int I2CT4::peek() {
    if (rx_buffer_index_ >= rx_buffer_length_) {
        return -1;
    }
    return rx_buffer_[rx_buffer_index_];
}

void I2CT4::flush() {}

uint8_t I2CT4::requestFrom(uint8_t address, uint8_t quantity, bool sendStop) {
    configASSERT(caller_ == nullptr);
    caller_ = ::xTaskGetCurrentTaskHandle();

    if (DEBUG_LEVEL_ >= 3) {
        printf_debug(PSTR("rF(%u) for 0x%x\r\n"), quantity, reinterpret_cast<uintptr_t>(caller_));
    }

    if (!wait_idle()) {
        if (DEBUG_LEVEL_) {
            printf_debug(PSTR("rF wait_idle() failed\r\n"));
        }
        return 255;
    }

    address = (address & 0x7F) << 1;
    if (quantity < 1) {
        quantity = 1;
    }
    rx_buffer_index_ = 0;
    rx_buffer_length_ = 0;
    uint32_t tx_state {}; // 0=begin, 1=start, 2=data, 3=stop
    elapsedMillis timeout;
    bool isr_timeout { false };
    while (true) {
        if (DEBUG_LEVEL_ >= 4) {
            printf_debug(PSTR("rF tx_state=%u rx_buffer_length_=%u rx_fifo=%u\r\n"), tx_state, rx_buffer_length_, (p_port_->MFSR >> 16) & 7);
        }
        // transmit stuff, if we haven't already
        if (tx_state < 3) {
            uint32_t tx_fifo { p_port_->MFSR & 7 }; // pg 2914
            while (tx_fifo < 4 && tx_state < 3) {
                if (tx_state == 0) {
                    p_port_->MTDR = LPI2C_MTDR_CMD_START | 1 | address;
                } else if (tx_state == 1) {
                    p_port_->MTDR = LPI2C_MTDR_CMD_RECEIVE | (quantity - 1);
                } else if (sendStop) {
                    p_port_->MTDR = LPI2C_MTDR_CMD_STOP;
                }
                ++tx_state;
                --tx_fifo;
            }
        }
        // receive stuff
        uint32_t rx_fifo;
        while ((rx_fifo = (p_port_->MFSR >> 16) & 7) > 0 && rx_buffer_length_ < sizeof(rx_buffer_)) {
            rx_buffer_[rx_buffer_length_] = p_port_->MRDR;
            if (DEBUG_LEVEL_ >= 4) {
                printf_debug(PSTR("rF data received: 0x%02x rx_fifo=%u\r\n"), rx_buffer_[rx_buffer_length_], rx_fifo);
            }
            --rx_fifo;
            ++rx_buffer_length_;
        }
        if (rx_buffer_length_ > sizeof(rx_buffer_)) {
            if (DEBUG_LEVEL_) {
                printf_debug(PSTR("rF RX BUFFER OVERFLOW\r\n"));
            }
            break;
        }
        if (DEBUG_LEVEL_ >= 4) {
            printf_debug(PSTR("rF rx_buffer_length_=%u\r\n"), rx_buffer_length_);
        }

        // monitor status, check for error conditions
        const uint32_t status { p_port_->MSR }; // pg 2884 & 2891
        if (status & LPI2C_MSR_ALF) {
            p_port_->MCR = p_port_->MCR | (LPI2C_MCR_RTF | LPI2C_MCR_RRF); // clear FIFOs
            if (DEBUG_LEVEL_) {
                printf_debug(PSTR("rF LPI2C_MSR_ALF\r\n"));
            }
            break;
        }
        if ((status & LPI2C_MSR_NDF) || (status & LPI2C_MSR_PLTF) || timeout > 50) {
            p_port_->MCR = p_port_->MCR | (LPI2C_MCR_RTF | LPI2C_MCR_RRF); // clear FIFOs
            p_port_->MTDR = LPI2C_MTDR_CMD_STOP; // try to send a stop
            if (DEBUG_LEVEL_) {
                const uint32_t tmp { timeout };
                printf_debug(PSTR("rF LPI2C_MSR_NDF | LPI2C_MSR_PLTF | TIMEOUT\r\n"));
                printf_debug(PSTR("\tMSR=0x%x timeout=%u ms\r\n"), status, tmp);
                (void) tmp;
            }
            break;
        }

        if (DEBUG_LEVEL_ >= 2 && isr_timeout) {
            isr_timeout = false;
            printf_debug(PSTR("rF ISR TIMEOUT recovered\r\n"));
        }

        // are we done yet?
        if (rx_buffer_length_ >= quantity && tx_state >= 3) {
            if (DEBUG_LEVEL_ >= 4) {
                printf_debug(PSTR("rF RX done.\r\n"));
            }
            const uint32_t tx_fifo { p_port_->MFSR & 7 };
            if (tx_fifo == 0 && ((status & LPI2C_MSR_SDF) || !sendStop)) {
                if (DEBUG_LEVEL_ >= 3) {
                    printf_debug(PSTR("rF done for 0x%x\r\n"), reinterpret_cast<uintptr_t>(*p_caller_));
                }
                break;
            }
        }

        if (tx_state >= 3 && quantity - rx_buffer_length_ > 2) { // FIXME: parameter for 2
            if (DEBUG_LEVEL_ >= 4) {
                const uint32_t rx_fifo { (p_port_->MFSR >> 16) & 7 };
                printf_debug(PSTR("rF to rx=%d rx_fifo=%u\r\n"), quantity - rx_buffer_length_, rx_fifo);
                (void) rx_fifo;
            }

            const uint32_t rx_fifo { (p_port_->MFSR >> 16) & 7 };
            if (rx_fifo == 0) {
                if (DEBUG_LEVEL_ >= 4) {
                    printf_debug(PSTR("rF waiting for irq, rx_fifo=%u\r\n"), rx_fifo);
                }
                // ::xTaskNotifyStateClear(nullptr);
                p_port_->MIER = LPI2C_MIER_RDIE;
                uint32_t status;
                if (::xTaskNotifyWait(DEBUG_LEVEL_ >= 2 ? 1 : 0, 0, DEBUG_LEVEL_ >= 2 ? &status : nullptr, pdMS_TO_TICKS(1)) == pdFALSE) {
                    p_port_->MIER = 0;
                    if (DEBUG_LEVEL_ >= 2) {
                        isr_timeout = true;
                        printf_debug(PSTR("rF ISR TIMEOUT\r\n"));
                    }
                } else if (DEBUG_LEVEL_ >= 2 && !(status & 1)) {
                    printf_debug(PSTR("rF unexpected ISR return value: 0x%x\r\n"), status);
                }
                if (DEBUG_LEVEL_ >= 4) {
                    const uint32_t rx_fifo { (p_port_->MFSR >> 16) & 7 };
                    printf_debug(PSTR("rF rx_fifo=%u\r\n"), rx_fifo);
                    (void) rx_fifo;
                }
            } else {
                if (DEBUG_LEVEL_ >= 4) {
                    const uint32_t rx_fifo { (p_port_->MFSR >> 16) & 7 };
                    printf_debug(PSTR("rF FIFO now filled: rx_fifo=%u\r\n"), rx_fifo);
                    (void) rx_fifo;
                }
            }
        } else {
            if (DEBUG_LEVEL_ >= 4) {
                const uint32_t rx_fifo { (p_port_->MFSR >> 16) & 7 };
                printf_debug(PSTR("rF yield(); rx_fifo=%u\r\n"), rx_fifo);
                (void) rx_fifo;
            }
            ::yield();
        }
    }
    p_port_->MIER = 0;
    caller_ = nullptr;
    isr_timeout = false;
    const uint32_t rx_fifo { (p_port_->MFSR >> 16) & 7 };
    if (rx_fifo > 0) {
        p_port_->MCR = p_port_->MCR | LPI2C_MCR_RRF;
        if (DEBUG_LEVEL_ >= 2) {
            printf_debug(PSTR("rF FIFO cleared\r\n"));
        }
    }

    return rx_buffer_length_;
}

FASTRUN void I2CT4::isr() {
    p_port_->MIER = 0;
    if (caller_) {
        uint32_t status;
        if (DEBUG_LEVEL_ >= 2) {
            status = p_port_->MSR;
        }
        BaseType_t higher_woken {};
        if (::xTaskNotifyFromISR(caller_, DEBUG_LEVEL_ >= 2 ? (status & LPI2C_MSR_RDF ? 1 : (status & LPI2C_MSR_TDF ? 2 : 4)) : 0,
                DEBUG_LEVEL_ >= 2 ? eSetBits : eNoAction, &higher_woken)) {
            portYIELD_FROM_ISR(higher_woken);
        }
    }
    portDATA_SYNC_BARRIER(); // mitigate arm errata #838869
}

} // namespace freertos

#endif // ARDUINO_TEENSY40 || ARDUINO_TEENSY41
