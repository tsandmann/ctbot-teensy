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
 * @file    i2c_t4.cpp
 * @brief   Teensy 4.x I2C FreeRTOS driver
 * @author  Timo Sandmann
 * @date    25.03.2021
 */

#if defined ARDUINO_TEENSY40 || defined ARDUINO_TEENSY41
#include "i2c_t4.h"


namespace arduino {
TaskHandle_t I2CT4::caller_[3] {};

namespace teensy4 {
I2CT4 Wire { 0 };
I2CT4 Wire1 { 1 };
I2CT4 Wire2 { 2 };
} // namespace teensy4

FLASHMEM I2CT4::I2CT4(const uint8_t bus) : TwoWire { get_port(bus), *get_hardware(bus) }, p_caller_ { &caller_[bus] } {
    NVIC_DISABLE_IRQ(hardware.irq_number);
    NVIC_CLEAR_PENDING(hardware.irq_number);
    ::attachInterruptVector(hardware.irq_number, bus == 0 ? isr1 : (bus == 1 ? isr2 : isr3));
    NVIC_SET_PRIORITY(hardware.irq_number, 128);
    NVIC_ENABLE_IRQ(hardware.irq_number);
    port->MDER = 0;
}

void I2CT4::setClock(uint32_t frequency) {
    TwoWire::setClock(frequency);
    port->MFCR = LPI2C_MFCR_RXWATER(RX_WATERMARK_) | LPI2C_MFCR_TXWATER(TX_WATERMARK_);
}

bool I2CT4::wait_idle() {
    elapsedMillis timeout;
    while (1) {
        const uint32_t status { port->MSR }; // pg 2899 & 2892
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
    port->MSR = 0x7F00; // clear all prior flags
    return true;
}

uint8_t I2CT4::endTransmission(uint8_t sendStop) {
    configASSERT(*p_caller_ == nullptr);

    uint32_t tx_len { txBufferLength };
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
    *p_caller_ = ::xTaskGetCurrentTaskHandle();
    if (DEBUG_LEVEL_ >= 3) {
        printf_debug(PSTR("eT for 0x%x\r\n"), reinterpret_cast<uintptr_t>(*p_caller_));
    }
    bool isr_timeout { false };
    while (1) {
        // transmit stuff, if we haven't already
        if (tx_index <= tx_len) {
            uint32_t fifo_used { port->MFSR & 7 }; // pg 2914
            while (fifo_used < 4) {
                if (tx_index == 0) {
                    port->MTDR = LPI2C_MTDR_CMD_START | txBuffer[0];
                    tx_index = 1;
                } else if (tx_index < tx_len) {
                    port->MTDR = LPI2C_MTDR_CMD_TRANSMIT | txBuffer[tx_index++];
                } else {
                    if (sendStop) {
                        port->MTDR = LPI2C_MTDR_CMD_STOP;
                    }
                    tx_index++;
                    break;
                }
                fifo_used++;
            }
        }
        // monitor status
        const uint32_t status { port->MSR }; // pg 2884 & 2891
        if (status & LPI2C_MSR_ALF) {
            port->MCR = port->MCR | (LPI2C_MCR_RTF | LPI2C_MCR_RRF); // clear FIFOs

            port->MIER = 0;
            if (DEBUG_LEVEL_) {
                printf_debug(PSTR("eT abort (LPI2C_MSR_ALF) for 0x%x\r\n"), reinterpret_cast<uintptr_t>(*p_caller_));
            }
            *p_caller_ = nullptr;
            return 3; // we lost bus arbitration to another master
        }
        if (status & LPI2C_MSR_NDF) {
            port->MCR = port->MCR | (LPI2C_MCR_RTF | LPI2C_MCR_RRF); // clear FIFOs
            port->MTDR = LPI2C_MTDR_CMD_STOP;

            port->MIER = 0;
            if (DEBUG_LEVEL_) {
                printf_debug(PSTR("eT abort (LPI2C_MSR_NDF) for 0x%x\r\n"), reinterpret_cast<uintptr_t>(*p_caller_));
            }
            *p_caller_ = nullptr;
            return 4; // NACK
        }
        if ((status & LPI2C_MSR_PLTF) || timeout > 50) {
            port->MCR = port->MCR | (LPI2C_MCR_RTF | LPI2C_MCR_RRF); // clear FIFOs
            port->MTDR = LPI2C_MTDR_CMD_STOP; // try to send a stop

            port->MIER = 0;
            if (DEBUG_LEVEL_) {
                printf_debug(PSTR("eT abort (LPI2C_MSR_PLTF | timeout) for 0x%x\r\n"), reinterpret_cast<uintptr_t>(*p_caller_));
                printf_debug(PSTR("\ttimeout=%u ms\r\n"), static_cast<uint32_t>(timeout));
            }
            *p_caller_ = nullptr;
            return 5; // clock stretched too long or generic timeout
        }

        if (DEBUG_LEVEL_ >= 2 && isr_timeout) {
            isr_timeout = false;
            printf_debug(PSTR("eT ISR TIMEOUT recovered\r\n"));
        }

        // are we done yet?
        if (tx_index > tx_len) {
            const uint32_t tx_fifo { port->MFSR & 7 };
            if (tx_fifo == 0 && ((status & LPI2C_MSR_SDF) || !sendStop)) {
                port->MIER = 0;
                if (DEBUG_LEVEL_ >= 3) {
                    printf_debug(PSTR("eT done for 0x%x\r\n"), reinterpret_cast<uintptr_t>(*p_caller_));
                }
                *p_caller_ = nullptr;
                return 0;
            }
        }

        const uint32_t tx_fifo { port->MFSR & 7 };
        if (tx_fifo >= 4 || (tx_index > tx_len && tx_fifo > 2)) { // FIXME: parameter for 4? | test second condition
            if (DEBUG_LEVEL_ >= 4) {
                printf_debug(PSTR("eT waiting for irq, tx_fifo=%u\r\n"), tx_fifo);
            }
            port->MIER = LPI2C_MIER_TDIE;
            uint32_t status;
            if (::xTaskNotifyWait(DEBUG_LEVEL_ >= 2 ? 2 : 0, 0, DEBUG_LEVEL_ >= 2 ? &status : nullptr, pdMS_TO_TICKS(1)) == pdFALSE) {
                port->MIER = 0;
                if (DEBUG_LEVEL_ >= 2) {
                    isr_timeout = true;
                    printf_debug(PSTR("eT ISR TIMEOUT\r\n"));
                }
            } else if (DEBUG_LEVEL_ >= 2 && !(status & 2)) {
                printf_debug(PSTR("eT unexpected ISR return value: 0x%x\r\n"), status);
            }
            if (DEBUG_LEVEL_ >= 4) {
                const uint32_t tx_fifo { port->MFSR & 7 };
                printf_debug(PSTR("eT tx_fifo=%u\r\n"), tx_fifo);
                (void) tx_fifo;
            }
        }
    }
    configASSERT(false);
}

uint8_t I2CT4::requestFrom(uint8_t address, uint8_t quantity, uint8_t sendStop) {
    configASSERT(*p_caller_ == nullptr);
    *p_caller_ = ::xTaskGetCurrentTaskHandle();

    if (DEBUG_LEVEL_ >= 3) {
        printf_debug(PSTR("rF(%u) for 0x%x\r\n"), quantity, reinterpret_cast<uintptr_t>(*p_caller_));
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
    rxBufferIndex = 0;
    rxBufferLength = 0;
    uint32_t tx_state {}; // 0=begin, 1=start, 2=data, 3=stop
    elapsedMillis timeout;
    bool isr_timeout { false };
    while (1) {
        if (DEBUG_LEVEL_ >= 4) {
            printf_debug(PSTR("rF tx_state=%u rxBufferLength=%u rx_fifo=%u\r\n"), tx_state, rxBufferLength, (port->MFSR >> 16) & 7);
        }
        // transmit stuff, if we haven't already
        if (tx_state < 3) {
            uint32_t tx_fifo { port->MFSR & 7 }; // pg 2914
            while (tx_fifo < 4 && tx_state < 3) {
                if (tx_state == 0) {
                    port->MTDR = LPI2C_MTDR_CMD_START | 1 | address;
                } else if (tx_state == 1) {
                    port->MTDR = LPI2C_MTDR_CMD_RECEIVE | (quantity - 1);
                } else if (sendStop) {
                    port->MTDR = LPI2C_MTDR_CMD_STOP;
                }
                ++tx_state;
                --tx_fifo;
            }
        }
        // receive stuff
        uint32_t rx_fifo;
        while ((rx_fifo = (port->MFSR >> 16) & 7) > 0 && rxBufferLength < sizeof(rxBuffer)) {
            rxBuffer[rxBufferLength] = port->MRDR;
            if (DEBUG_LEVEL_ >= 4) {
                printf_debug(PSTR("rF data received: 0x%02x rx_fifo=%u\r\n"), rxBuffer[rxBufferLength], rx_fifo);
            }
            --rx_fifo;
            ++rxBufferLength;
        }
        if (rxBufferLength > sizeof(rxBuffer)) {
            if (DEBUG_LEVEL_) {
                printf_debug(PSTR("rF RX BUFFER OVERFLOW\r\n"));
            }
            break;
        }
        if (DEBUG_LEVEL_ >= 4) {
            printf_debug(PSTR("rF rxBufferLength=%u\r\n"), rxBufferLength);
        }

        // monitor status, check for error conditions
        const uint32_t status { port->MSR }; // pg 2884 & 2891
        if (status & LPI2C_MSR_ALF) {
            port->MCR = port->MCR | (LPI2C_MCR_RTF | LPI2C_MCR_RRF); // clear FIFOs
            if (DEBUG_LEVEL_) {
                printf_debug(PSTR("rF LPI2C_MSR_ALF\r\n"));
            }
            break;
        }
        if ((status & LPI2C_MSR_NDF) || (status & LPI2C_MSR_PLTF) || timeout > 50) {
            port->MCR = port->MCR | (LPI2C_MCR_RTF | LPI2C_MCR_RRF); // clear FIFOs
            port->MTDR = LPI2C_MTDR_CMD_STOP; // try to send a stop
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
        if (rxBufferLength >= quantity && tx_state >= 3) {
            if (DEBUG_LEVEL_ >= 4) {
                printf_debug(PSTR("rF RX done.\r\n"));
            }
            const uint32_t tx_fifo { port->MFSR & 7 };
            if (tx_fifo == 0 && ((status & LPI2C_MSR_SDF) || !sendStop)) {
                if (DEBUG_LEVEL_ >= 3) {
                    printf_debug(PSTR("rF done for 0x%x\r\n"), reinterpret_cast<uintptr_t>(*p_caller_));
                }
                break;
            }
        }

        if (tx_state >= 3 && quantity - rxBufferLength > 2) { // FIXME: parameter for 2
            if (DEBUG_LEVEL_ >= 4) {
                const uint32_t rx_fifo { (port->MFSR >> 16) & 7 };
                printf_debug(PSTR("rF to rx=%d rx_fifo=%u\r\n"), quantity - rxBufferLength, rx_fifo);
                (void) rx_fifo;
            }

            const uint32_t rx_fifo { (port->MFSR >> 16) & 7 };
            if (rx_fifo == 0) {
                if (DEBUG_LEVEL_ >= 4) {
                    printf_debug(PSTR("rF waiting for irq, rx_fifo=%u\r\n"), rx_fifo);
                }
                // ::xTaskNotifyStateClear(nullptr);
                port->MIER = LPI2C_MIER_RDIE;
                uint32_t status;
                if (::xTaskNotifyWait(DEBUG_LEVEL_ >= 2 ? 1 : 0, 0, DEBUG_LEVEL_ >= 2 ? &status : nullptr, pdMS_TO_TICKS(1)) == pdFALSE) {
                    port->MIER = 0;
                    if (DEBUG_LEVEL_ >= 2) {
                        isr_timeout = true;
                        printf_debug(PSTR("rF ISR TIMEOUT\r\n"));
                    }
                } else if (DEBUG_LEVEL_ >= 2 && !(status & 1)) {
                    printf_debug(PSTR("rF unexpected ISR return value: 0x%x\r\n"), status);
                }
                if (DEBUG_LEVEL_ >= 4) {
                    const uint32_t rx_fifo { (port->MFSR >> 16) & 7 };
                    printf_debug(PSTR("rF rx_fifo=%u\r\n"), rx_fifo);
                    (void) rx_fifo;
                }
            } else {
                if (DEBUG_LEVEL_ >= 4) {
                    const uint32_t rx_fifo { (port->MFSR >> 16) & 7 };
                    printf_debug(PSTR("rF FIFO now filled: rx_fifo=%u\r\n"), rx_fifo);
                    (void) rx_fifo;
                }
            }
        } else {
            if (DEBUG_LEVEL_ >= 4) {
                const uint32_t rx_fifo { (port->MFSR >> 16) & 7 };
                printf_debug(PSTR("rF yield(); rx_fifo=%u\r\n"), rx_fifo);
                (void) rx_fifo;
            }
            ::yield();
        }
    }
    port->MIER = 0;
    *p_caller_ = nullptr;
    isr_timeout = false;
    const uint32_t rx_fifo { (port->MFSR >> 16) & 7 };
    if (rx_fifo > 0) {
        port->MCR = port->MCR | LPI2C_MCR_RRF;
        if (DEBUG_LEVEL_ >= 2) {
            printf_debug(PSTR("rF FIFO cleared\r\n"));
        }
    }

    return rxBufferLength;
}

FASTRUN void I2CT4::isr1() {
    IMXRT_LPI2C1.MIER = 0;
    if (caller_[0]) {
        uint32_t status;
        if (DEBUG_LEVEL_ >= 2) {
            status = IMXRT_LPI2C1.MSR;
        }
        BaseType_t higher_woken {};
        if (::xTaskNotifyFromISR(caller_[0], DEBUG_LEVEL_ >= 2 ? (status & LPI2C_MSR_RDF ? 1 : (status & LPI2C_MSR_TDF ? 2 : 4)) : 0,
                DEBUG_LEVEL_ >= 2 ? eSetBits : eNoAction, &higher_woken)) {
            portYIELD_FROM_ISR(higher_woken);
        }
    }
    portDATA_SYNC_BARRIER(); // mitigate arm errata #838869
}

FASTRUN void I2CT4::isr2() {
    IMXRT_LPI2C3.MIER = 0;
    if (caller_[1]) {
        uint32_t status;
        if (DEBUG_LEVEL_ >= 2) {
            status = IMXRT_LPI2C3.MSR;
        }
        BaseType_t higher_woken {};
        if (::xTaskNotifyFromISR(caller_[1], DEBUG_LEVEL_ >= 2 ? (status & LPI2C_MSR_RDF ? 1 : (status & LPI2C_MSR_TDF ? 2 : 4)) : 0,
                DEBUG_LEVEL_ >= 2 ? eSetBits : eNoAction, &higher_woken)) {
            portYIELD_FROM_ISR(higher_woken);
        }
    }
    portDATA_SYNC_BARRIER(); // mitigate arm errata #838869
}

FASTRUN void I2CT4::isr3() {
    IMXRT_LPI2C4.MIER = 0;
    if (caller_[2]) {
        uint32_t status;
        if (DEBUG_LEVEL_ >= 2) {
            status = IMXRT_LPI2C4.MSR;
        }
        BaseType_t higher_woken {};
        if (::xTaskNotifyFromISR(caller_[2], DEBUG_LEVEL_ >= 2 ? (status & LPI2C_MSR_RDF ? 1 : (status & LPI2C_MSR_TDF ? 2 : 4)) : 0,
                DEBUG_LEVEL_ >= 2 ? eSetBits : eNoAction, &higher_woken)) {
            portYIELD_FROM_ISR(higher_woken);
        }
    }
    portDATA_SYNC_BARRIER(); // mitigate arm errata #838869
}
} // namespace arduino

#endif // ARDUINO_TEENSY40 || ARDUINO_TEENSY41
