/*
 * Copyright (c) 2010 by Cristian Maglie <c.maglie@bug.st>
 * Copyright (c) 2014 by Paul Stoffregen <paul@pjrc.com> (Transaction API)
 * Copyright (c) 2014 by Matthijs Kooijman <matthijs@stdin.nl> (SPISettings AVR)
 * Copyright (c) 2023 by Timo Sandmann (Teensy 4.x SPI FreeRTOS driver)
 * SPI Master library for arduino.
 *
 * This file is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2
 * published by the Free Software Foundation.
 */

/**
 * @file    spi_t4.cpp
 * @brief   Teensy 4.x SPI FreeRTOS driver
 * @author  Timo Sandmann
 * @date    05.08.2023
 */

#if defined ARDUINO_TEENSY40 || defined ARDUINO_TEENSY41
#include "spi_t4.h"


namespace freertos {

SpiT4* SpiT4::p_instances_[MAX_BUS_ + 1] {};

void SpiT4::begin() {
    p_instances_[hardware().tx_dma_channel == DMAMUX_SOURCE_LPSPI4_TX ? 0 : (hardware().tx_dma_channel == DMAMUX_SOURCE_LPSPI3_TX ? 1 : 2)] = this;

    *hardware().clock_gate_register = *hardware().clock_gate_register & ~hardware().clock_gate_mask;
    CCM_CBCMR = (CCM_CBCMR & ~(CCM_CBCMR_LPSPI_PODF_MASK | CCM_CBCMR_LPSPI_CLK_SEL_MASK)) | CCM_CBCMR_LPSPI_PODF(2) | CCM_CBCMR_LPSPI_CLK_SEL(1); // pg 714
    const uint32_t fastio { IOMUXC_PAD_DSE(7) | IOMUXC_PAD_SPEED(2) };
    if constexpr (DEBUG_) {
        Serial.printf(PSTR("SPI MISO: %d MOSI: %d, SCK: %d\r\n"), hardware().miso_pin[miso_pin_index_], hardware().mosi_pin[mosi_pin_index_],
            hardware().sck_pin[sck_pin_index_]);
    }
    *(portControlRegister(hardware().miso_pin[miso_pin_index_])) = fastio;
    *(portControlRegister(hardware().mosi_pin[mosi_pin_index_])) = fastio;
    *(portControlRegister(hardware().sck_pin[sck_pin_index_])) = fastio;

    if constexpr (DEBUG_) {
        Serial.printf(PSTR("CBCMR = 0x%08x\r\n"), CCM_CBCMR);
    }
    *hardware().clock_gate_register = *hardware().clock_gate_register | hardware().clock_gate_mask;
    *(portConfigRegister(hardware().miso_pin[miso_pin_index_])) = hardware().miso_mux[miso_pin_index_];
    *(portConfigRegister(hardware().mosi_pin[mosi_pin_index_])) = hardware().mosi_mux[mosi_pin_index_];
    *(portConfigRegister(hardware().sck_pin[sck_pin_index_])) = hardware().sck_mux[sck_pin_index_];

    // set the mux pins
    *hardware().sck_select_input_register = hardware().sck_select_val[sck_pin_index_];
    *hardware().miso_select_input_register = hardware().miso_select_val[miso_pin_index_];
    *hardware().mosi_select_input_register = hardware().mosi_select_val[mosi_pin_index_];

    port().CR = LPSPI_CR_RST;

    const uint32_t tx_fifo_size { 1U << static_cast<uint8_t>(port().PARAM) };
    if constexpr (DEBUG_) {
        const uint32_t rx_fifo_size { 1U << static_cast<uint8_t>(port().PARAM >> 8U) };
        Serial.printf(PSTR("SpiT4::begin(): rx_fifo_size:%u tx_fifo_size:%u\r\n"), rx_fifo_size, tx_fifo_size);
    }
    // initialize the transmit FIFO watermark to FIFO size - 1
    port().FCR = LPSPI_FCR_TXWATER(tx_fifo_size - 1);

    // initialize the SPI to be in a known default state
    beginTransaction(SpiT4Settings {});
    endTransaction();
}

uint8_t SpiT4::pinIsChipSelect(uint8_t pin) const {
    for (size_t i {}; i < sizeof(hardware().cs_pin); ++i) {
        if (pin == hardware().cs_pin[i]) {
            return hardware().cs_mask[i];
        }
    }
    return 0;
}

bool SpiT4::pinIsChipSelect(uint8_t pin1, uint8_t pin2) const {
    uint8_t pin1_mask, pin2_mask;

    if ((pin1_mask = (uint8_t) pinIsChipSelect(pin1)) == 0) {
        return false;
    }

    if ((pin2_mask = (uint8_t) pinIsChipSelect(pin2)) == 0) {
        return false;
    }

    if constexpr (DEBUG_) {
        Serial.printf(PSTR("SpiT4::pinIsChipSelect( %d %d 0x%x 0x%x)\r\n"), pin1, pin2, pin1_mask, pin2_mask);
    }

    if ((pin1_mask & pin2_mask) != 0) {
        return false;
    }

    return true;
}


bool SpiT4::pinIsMOSI(uint8_t pin) const {
    for (size_t i {}; i < sizeof(hardware().mosi_pin); ++i) {
        if (pin == hardware().mosi_pin[i]) {
            return true;
        }
    }
    return false;
}

bool SpiT4::pinIsMISO(uint8_t pin) const {
    for (size_t i {}; i < sizeof(hardware().miso_pin); ++i) {
        if (pin == hardware().miso_pin[i]) {
            return true;
        }
    }
    return false;
}

bool SpiT4::pinIsSCK(uint8_t pin) const {
    for (size_t i {}; i < sizeof(hardware().sck_pin); ++i) {
        if (pin == hardware().sck_pin[i]) {
            return true;
        }
    }
    return false;
}

// setCS() is not intended for use from normal Arduino programs/sketches.
uint8_t SpiT4::setCS(uint8_t pin) {
    for (size_t i {}; i < sizeof(hardware().cs_pin); ++i) {
        if (pin == hardware().cs_pin[i]) {
            *(portConfigRegister(pin)) = hardware().cs_mux[i];

            if (hardware().pcs_select_input_register[i]) {
                *hardware().pcs_select_input_register[i] = hardware().pcs_select_val[i];
            }

            return hardware().cs_mask[i];
        }
    }
    return 0;
}

void SpiT4::setMOSI(uint8_t pin) {
    if (pin != hardware().mosi_pin[mosi_pin_index_]) {
        for (size_t i {}; i < sizeof(hardware().mosi_pin); ++i) {
            if (pin == hardware().mosi_pin[i]) {
                if (*hardware().clock_gate_register & hardware().clock_gate_mask) {
                    // FIXME: Unclear what to do with previous pin as there is no unused setting
                    const uint32_t fastio { IOMUXC_PAD_DSE(7) | IOMUXC_PAD_SPEED(2) };
                    *(portControlRegister(hardware().mosi_pin[i])) = fastio;
                    *(portConfigRegister(hardware().mosi_pin[i])) = hardware().mosi_mux[i];
                    *hardware().mosi_select_input_register = hardware().mosi_select_val[i];
                }
                mosi_pin_index_ = i;
                return;
            }
        }
    }
}

void SpiT4::setMISO(uint8_t pin) {
    if (pin != hardware().miso_pin[miso_pin_index_]) {
        for (size_t i {}; i < sizeof(hardware().miso_pin); ++i) {
            if (pin == hardware().miso_pin[i]) {
                if (*hardware().clock_gate_register & hardware().clock_gate_mask) {
                    // FIXME: Unclear what to do with previous pin as there is no unused setting
                    const uint32_t fastio { IOMUXC_PAD_DSE(7) | IOMUXC_PAD_SPEED(2) };
                    *(portControlRegister(hardware().miso_pin[i])) = fastio;
                    *(portConfigRegister(hardware().miso_pin[i])) = hardware().miso_mux[i];
                    *hardware().miso_select_input_register = hardware().miso_select_val[i];
                }
                miso_pin_index_ = i;
                return;
            }
        }
    }
}

void SpiT4::setSCK(uint8_t pin) {
    if (pin != hardware().sck_pin[sck_pin_index_]) {
        for (size_t i {}; i < sizeof(hardware().sck_pin); ++i) {
            if (pin == hardware().sck_pin[i]) {
                if (*hardware().clock_gate_register & hardware().clock_gate_mask) {
                    // FIXME: Unclear what to do with previous pin as there is no unused setting
                    uint32_t fastio = IOMUXC_PAD_DSE(7) | IOMUXC_PAD_SPEED(2);
                    *(portControlRegister(hardware().sck_pin[i])) = fastio;
                    *(portConfigRegister(hardware().sck_pin[i])) = hardware().sck_mux[i];
                    *hardware().sck_select_input_register = hardware().sck_select_val[i];
                }
                sck_pin_index_ = i;
                return;
            }
        }
    }
}

#ifdef ARDUINO_TEENSY41
PROGMEM constinit const SpiT4::Hardware_t SpiT4::lpspi4_hardware_ = {
    &CCM_CCGR1, CCM_CCGR1_LPSPI4(CCM_CCGR_ON), // CLOCK
    isr_dma<0>, DMAMUX_SOURCE_LPSPI4_TX, DMAMUX_SOURCE_LPSPI4_RX, // DMA
    12, 255, 3 | 0x10, 0, 0, 0, &IOMUXC_LPSPI4_SDI_SELECT_INPUT, // MISO
    11, 255, 3 | 0x10, 0, 0, 0, &IOMUXC_LPSPI4_SDO_SELECT_INPUT, // MOSI
    13, 255, 3 | 0x10, 0, 0, 0, &IOMUXC_LPSPI4_SCK_SELECT_INPUT, // SCK
    10, 37, 36, 3 | 0x10, 2 | 0x10, 2 | 0x10, 1, 2, 3, 0, 0, 0, &IOMUXC_LPSPI4_PCS0_SELECT_INPUT, nullptr, nullptr // CS
};
#else // ARDUINO_TEENSY40
PROGMEM constinit const SpiT4::Hardware_t SpiT4::lpspi4_hardware_ = {
    &CCM_CCGR1, CCM_CCGR1_LPSPI4(CCM_CCGR_ON), // CLOCK
    isr_dma<0>, DMAMUX_SOURCE_LPSPI4_TX, DMAMUX_SOURCE_LPSPI4_RX, // DMA
    12, 255, 3 | 0x10, 0, 0, 0, &IOMUXC_LPSPI4_SDI_SELECT_INPUT, // MISO
    11, 255, 3 | 0x10, 0, 0, 0, &IOMUXC_LPSPI4_SDO_SELECT_INPUT, // MOSI
    13, 255, 3 | 0x10, 0, 0, 0, &IOMUXC_LPSPI4_SCK_SELECT_INPUT, // SCK
    10, 255, 255, 3 | 0x10, 0, 0, 1, 0, 0, 0, 0, 0, &IOMUXC_LPSPI4_PCS0_SELECT_INPUT, nullptr, nullptr // CS
};
#endif // ARDUINO_TEENSY41

#ifdef ARDUINO_TEENSY41
PROGMEM constinit const SpiT4::Hardware_t SpiT4::lpspi3_hardware_ = {
    &CCM_CCGR1, CCM_CCGR1_LPSPI3(CCM_CCGR_ON), // CLOCK
    isr_dma<1>, DMAMUX_SOURCE_LPSPI3_TX, DMAMUX_SOURCE_LPSPI3_RX, // DMA
    1, 39, 7 | 0x10, 2 | 0x10, 0, 1, &IOMUXC_LPSPI3_SDI_SELECT_INPUT, // MISO
    26, 255, 2 | 0x10, 0, 1, 0, &IOMUXC_LPSPI3_SDO_SELECT_INPUT, // MOSI
    27, 255, 2 | 0x10, 0, 1, 0, &IOMUXC_LPSPI3_SCK_SELECT_INPUT, // SCK
    0, 38, 255, 7 | 0x10, 2 | 0x10, 0, 1, 1, 0, 0, 1, 0, &IOMUXC_LPSPI3_PCS0_SELECT_INPUT, &IOMUXC_LPSPI3_PCS0_SELECT_INPUT, nullptr // CS
};
#else // ARDUINO_TEENSY40
PROGMEM constinit const SpiT4::Hardware_t SpiT4::lpspi3_hardware_ = {
    &CCM_CCGR1, CCM_CCGR1_LPSPI3(CCM_CCGR_ON), // CLOCK
    isr_dma<1>, DMAMUX_SOURCE_LPSPI3_TX, DMAMUX_SOURCE_LPSPI3_RX, // DMA
    1, 255, 7 | 0x10, 0, 0, 0, &IOMUXC_LPSPI3_SDI_SELECT_INPUT, // MISO
    26, 255, 2 | 0x10, 0, 1, 0, &IOMUXC_LPSPI3_SDO_SELECT_INPUT, // MOSI
    27, 255, 2 | 0x10, 0, 1, 0, &IOMUXC_LPSPI3_SCK_SELECT_INPUT, // SCK
    0, 255, 255, 7 | 0x10, 0, 0, 1, 0, 0, 0, 0, 0, &IOMUXC_LPSPI3_PCS0_SELECT_INPUT, &IOMUXC_LPSPI3_PCS0_SELECT_INPUT, nullptr // CS
};
#endif // ARDUINO_TEENSY41

#ifdef ARDUINO_TEENSY41
PROGMEM constinit const SpiT4::Hardware_t SpiT4::lpspi1_hardware_ = {
    &CCM_CCGR1, CCM_CCGR1_LPSPI1(CCM_CCGR_ON), // CLOCK
    isr_dma<2>, DMAMUX_SOURCE_LPSPI1_TX, DMAMUX_SOURCE_LPSPI1_RX, // DMA
    42, 54, 4 | 0x10, 3 | 0x10, 1, 0, &IOMUXC_LPSPI1_SDI_SELECT_INPUT, // MISO
    43, 50, 4 | 0x10, 3 | 0x10, 1, 0, &IOMUXC_LPSPI1_SDO_SELECT_INPUT, // MOSI
    45, 49, 4 | 0x10, 3 | 0x10, 1, 0, &IOMUXC_LPSPI1_SCK_SELECT_INPUT, // SCK
    44, 255, 255, 4 | 0x10, 0, 0, 1, 0, 0, 0, 0, 0, &IOMUXC_LPSPI1_PCS0_SELECT_INPUT, nullptr, nullptr // CS
};
#else // ARDUINO_TEENSY40
PROGMEM constinit const SpiT4::Hardware_t SpiT4::lpspi1_hardware_ = {
    &CCM_CCGR1, CCM_CCGR1_LPSPI1(CCM_CCGR_ON), // CLOCK
    isr_dma<2>, DMAMUX_SOURCE_LPSPI1_TX, DMAMUX_SOURCE_LPSPI1_RX, // DMA
    34, 255, 4 | 0x10, 0, 1, 0, &IOMUXC_LPSPI1_SDI_SELECT_INPUT, // MISO
    35, 255, 4 | 0x10, 0, 1, 0, &IOMUXC_LPSPI1_SDO_SELECT_INPUT, // MOSI
    37, 255, 4 | 0x10, 0, 1, 0, &IOMUXC_LPSPI1_SCK_SELECT_INPUT, // SCK
    36, 255, 255, 4 | 0x10, 0, 0, 1, 0, 0, 0, 0, 0, &IOMUXC_LPSPI1_PCS0_SELECT_INPUT, nullptr, nullptr // CS
};
#endif // ARDUINO_TEENSY41

void SpiT4::usingInterrupt(IRQ_NUMBER_t interrupt) {
    const uint32_t n { static_cast<uint32_t>(interrupt) };

    if (n >= NVIC_NUM_INTERRUPTS) {
        return;
    }

    if constexpr (DEBUG_) {
        Serial.printf(PSTR("SpiT4::usingInterrupt(%u)\r\n"), n);
    }

    interruptMasksUsed_ |= (1 << (n >> 5));
    interruptMask_[n >> 5] |= (1 << (n & 0x1F));

    if constexpr (DEBUG_) {
        Serial.printf(PSTR("interruptMasksUsed = %u\r\n"), interruptMasksUsed_);
        Serial.printf(PSTR("interruptMask[0] = 0x%08x\r\n"), interruptMask_[0]);
        Serial.printf(PSTR("interruptMask[1] = 0x%08x\r\n"), interruptMask_[1]);
        Serial.printf(PSTR("interruptMask[2] = 0x%08x\r\n"), interruptMask_[2]);
        Serial.printf(PSTR("interruptMask[2] = 0x%08x\r\n"), interruptMask_[3]);
        Serial.printf(PSTR("interruptMask[2] = 0x%08x\r\n"), interruptMask_[4]);
    }
}

void SpiT4::notUsingInterrupt(IRQ_NUMBER_t interrupt) {
    const uint32_t n { static_cast<uint32_t>(interrupt) };
    if (n >= NVIC_NUM_INTERRUPTS) {
        return;
    }

    interruptMask_[n >> 5] &= ~(1 << (n & 0x1F));
    if (interruptMask_[n >> 5] == 0) {
        interruptMasksUsed_ &= ~(1 << (n >> 5));
    }
}

void SpiT4::beginTransaction(SpiT4Settings& settings) {
    if (interruptMasksUsed_) {
        if (interruptMasksUsed_ & 0x01) {
            const auto primask { __get_PRIMASK() };
            __disable_irq();

            interruptSave_[0] = NVIC_ICER0 & interruptMask_[0];
            NVIC_ICER0 = interruptSave_[0];

            __set_PRIMASK(primask);
        }
        if (interruptMasksUsed_ & 0x02) {
            const auto primask { __get_PRIMASK() };
            __disable_irq();

            interruptSave_[1] = NVIC_ICER1 & interruptMask_[1];
            NVIC_ICER1 = interruptSave_[1];

            __set_PRIMASK(primask);
        }
        if (interruptMasksUsed_ & 0x04) {
            const auto primask { __get_PRIMASK() };
            __disable_irq();

            interruptSave_[2] = NVIC_ICER2 & interruptMask_[2];
            NVIC_ICER2 = interruptSave_[2];

            __set_PRIMASK(primask);
        }
        if (interruptMasksUsed_ & 0x08) {
            const auto primask { __get_PRIMASK() };
            __disable_irq();

            interruptSave_[3] = NVIC_ICER3 & interruptMask_[3];
            NVIC_ICER3 = interruptSave_[3];

            __set_PRIMASK(primask);
        }
        if (interruptMasksUsed_ & 0x10) {
            const auto primask { __get_PRIMASK() };
            __disable_irq();

            interruptSave_[4] = NVIC_ICER4 & interruptMask_[4];
            NVIC_ICER4 = interruptSave_[4];

            __set_PRIMASK(primask);
        }
    }

    if (settings.get_clock() != clock_) {
        static constexpr uint32_t clk_sel[4] = {
            664615384, // PLL3 PFD1
            720000000, // PLL3 PFD0
            528000000, // PLL2
            396000000 // PLL2 PFD2
        };

        // save away the new settings
        clock_ = settings.get_clock();

        const uint32_t cbcmr { CCM_CBCMR };
        const uint32_t clkhz { clk_sel[(cbcmr >> 4) & 0x03] / (((cbcmr >> 26) & 0x07) + 1) }; // LPSPI peripheral clock

        uint32_t d { clock_ ? clkhz / clock_ : clkhz };
        if (d && clkhz / d > clock_) {
            d++;
        }
        if (d > 257) {
            d = 257; // max div
        }
        uint32_t div;
        if (d > 2) {
            div = d - 2;
        } else {
            div = 0;
        }

        ccr_ = LPSPI_CCR_SCKDIV(div) | LPSPI_CCR_DBT(div / 2) | LPSPI_CCR_PCSSCK(div / 2);
        if constexpr (DEBUG_) {
            Serial.printf(PSTR("SpiT4::beginTransaction(%u)\r\n"), settings.get_clock());
            Serial.printf(PSTR("SpiT4::beginTransaction(): clkhz:%u MHz cbcmr:0x%x\r\n"), clkhz / 1'000'000UL, cbcmr);
            Serial.printf(PSTR("SpiT4::beginTransaction(): CCR:0x%x TCR:0x%x\r\n"), ccr_, settings.get_tcr());
            Serial.print(PSTR("SpiT4::beginTransaction(): SPI frequency set to: "));
            const uint32_t freq { clkhz / ((ccr_ & 0xff) + 2U) };
            if (freq % 1'000'000U == 0) {
                Serial.printf(PSTR("%u MHz\r\n"), freq / 1'000'000U);
            } else if (freq % 1'000U == 0) {
                Serial.printf(PSTR("%u KHz\r\n"), freq / 1'000U);
            } else {
                Serial.printf(PSTR("%u Hz\r\n"), freq);
            }
        }
    }
    port().CR = 0;
    port().CFGR1 = LPSPI_CFGR1_MASTER | LPSPI_CFGR1_SAMPLE;
    port().CCR = ccr_;
    port().TCR = settings.get_tcr();
    port().CR = LPSPI_CR_MEN;
}

void SpiT4::endTransaction() {
    if (interruptMasksUsed_) {
        if (interruptMasksUsed_ & 0x01) {
            NVIC_ISER0 = interruptSave_[0];
        }
        if (interruptMasksUsed_ & 0x02) {
            NVIC_ISER1 = interruptSave_[1];
        }
        if (interruptMasksUsed_ & 0x04) {
            NVIC_ISER2 = interruptSave_[2];
        }
        if (interruptMasksUsed_ & 0x08) {
            NVIC_ISER3 = interruptSave_[3];
        }
        if (interruptMasksUsed_ & 0x10) {
            NVIC_ISER4 = interruptSave_[4];
        }
    }
    if constexpr (DEBUG_) {
        Serial.printf(PSTR("SpiT4::endTransaction CCR:%x TCR:%x\r\n"), port().CCR, port().TCR);
    }
}

template <SpiT4TransferType T>
void SpiT4::transfer(const T* tx_buffer, T* rx_buffer, size_t count) {
    if (count == 0) {
        return;
    }

    T* p_write { const_cast<T*>(tx_buffer) };
    T* p_read { rx_buffer };
    size_t count_read { count };

    uint32_t tcr;
    if (sizeof(T) > 1) {
        tcr = port().TCR;
        port().TCR = (tcr & 0xfffff000) | LPSPI_TCR_FRAMESZ(sizeof(T) * 8 - 1); // turn on N bit mode
    }
    // Lets clear the reader queue
    port().CR = LPSPI_CR_RRF | LPSPI_CR_MEN; // clear the queue and make sure still enabled.

    while (count > 0) {
        // Push out the next byte;
        port().TDR = p_write ? *p_write++ : static_cast<T>(transfer_write_fill_);
        count--; // how many bytes left to output.
        // Make sure queue is not full before pushing next byte out
        do {
            if ((port().RSR & LPSPI_RSR_RXEMPTY) == 0) {
                const T b { static_cast<T>(port().RDR) }; // Read any pending RX bytes in
                if (p_read) {
                    *p_read++ = b;
                }
                count_read--;
            }
        } while ((port().SR & LPSPI_SR_TDF) == 0);
    }

    // now lets wait for all of the read bytes to be returned...
    while (count_read) {
        if ((port().RSR & LPSPI_RSR_RXEMPTY) == 0) {
            const T b { static_cast<T>(port().RDR) }; // Read any pending RX bytes in
            if (p_read) {
                *p_read++ = b;
            }
            count_read--;
        }
    }

    if (sizeof(T) > 1) {
        port().TCR = tcr; // restore TCR
    }
}

template void SpiT4::transfer<uint8_t>(const uint8_t*, uint8_t*, size_t);
template void SpiT4::transfer<uint16_t>(const uint16_t*, uint16_t*, size_t);
template void SpiT4::transfer<uint32_t>(const uint32_t*, uint32_t*, size_t);

void SpiT4::end() {
    // only do something if we have begun
    if (*hardware().clock_gate_register & hardware().clock_gate_mask) {
        port().CR = 0; // turn off the enable
        pinMode(hardware().miso_pin[miso_pin_index_], arduino::INPUT_DISABLE);
        pinMode(hardware().mosi_pin[mosi_pin_index_], arduino::INPUT_DISABLE);
        pinMode(hardware().sck_pin[sck_pin_index_], arduino::INPUT_DISABLE);
    }
}

bool SpiT4::init_dma_channels() {
    // setup the RX chain
    dmaRX_.disable();
    dmaRX_.source((volatile uint8_t&) port().RDR);
    dmaRX_.disableOnCompletion();
    dmaRX_.triggerAtHardwareEvent(hardware().rx_dma_channel);
    dmaRX_.attachInterrupt(hardware().dma_rxisr);
    dmaRX_.interruptAtCompletion();

    // We may be using settings chain here so lets set it up.
    // Now lets setup TX chain. Note if trigger TX is not set
    // we need to have the RX do it for us.
    dmaTX_.disable();
    dmaTX_.destination((volatile uint8_t&) port().TDR);
    dmaTX_.disableOnCompletion();

    if (hardware().tx_dma_channel) {
        dmaTX_.triggerAtHardwareEvent(hardware().tx_dma_channel);
    } else {
        if constexpr (DEBUG_) {
            Serial.printf(PSTR("SPI InitDMA tx triger by RX: 0x%x\r\n"), reinterpret_cast<uintptr_t>(&dmaRX_));
        }
        dmaTX_.triggerAtTransfersOf(dmaRX_);
    }

    dma_state_ = DMAState::idle; // should be first thing set!
    return true;
}

void SpiT4::DMA_channel_transfer_count(DMAChannel& channel, uint32_t len) {
    // TODO: validation of length
    DMABaseClass::TCD_t* p_tcd { channel.TCD };
    if (!(p_tcd->BITER & DMA_TCD_BITER_ELINK)) {
        p_tcd->BITER = len & 0x7fff;
    } else {
        p_tcd->BITER = (p_tcd->BITER & 0xFE00) | (len & 0x1ff);
    }
    p_tcd->CITER = p_tcd->BITER;
}

void SpiT4::dump_dma_tcd(const DMABaseClass* p_dma) {
    Serial.printf(PSTR("SpiT4::dump_dma_tcd(0x%x): tcd=0x%x\r\n  "), reinterpret_cast<uintptr_t>(p_dma), reinterpret_cast<uintptr_t>(p_dma->TCD));
    Serial.printf(PSTR("SA=0x%x SO=%4d AT=0x%04x NB=%7u SL=%6d\r\n  DA=0x%x DO=%4d CI=0x%04x CS=0x%05x BI=0x%04x DL=0x%x\r\n"),
        reinterpret_cast<uintptr_t>(p_dma->TCD->SADDR), p_dma->TCD->SOFF, p_dma->TCD->ATTR, p_dma->TCD->NBYTES, p_dma->TCD->SLAST,
        reinterpret_cast<uintptr_t>(p_dma->TCD->DADDR), p_dma->TCD->DOFF, p_dma->TCD->CITER, p_dma->TCD->CSR, p_dma->TCD->BITER, p_dma->TCD->DLASTSGA);
}

template <SpiT4TransferType T>
bool SpiT4::setup_dma_transfer(const T* tx_buffer, T* rx_buffer, size_t count) {
    DMAMEM static T bit_bucket __attribute__((aligned(4)));

    if (dma_state_ == DMAState::uninitialized) {
        if (!init_dma_channels()) {
            return false;
        }
    }

    if (dma_state_ == DMAState::active) {
        return false; // already active
    }

    // lets clear cache before we update sizes...
    T* write_data { const_cast<T*>(tx_buffer) };
    const auto tx_addr { reinterpret_cast<uintptr_t>(write_data) };
    if (tx_addr >= 0x20'200'000UL && tx_addr < 0x20'280'000UL) {
        arm_dcache_flush(write_data, count * sizeof(T));
    }
    const auto rx_addr { reinterpret_cast<uintptr_t>(rx_buffer) };
    if (rx_addr >= 0x20'200'000UL && rx_addr < 0x20'280'000UL) {
        arm_dcache_delete(rx_buffer, count * sizeof(T));
    }

    // Now handle the cases where the count > then how many we can output in one DMA request
    if (count > MAX_DMA_COUNT_) {
        dma_count_remaining_ = count - MAX_DMA_COUNT_;
        count = MAX_DMA_COUNT_;
    } else {
        dma_count_remaining_ = 0;
    }

    // Now See if caller passed in a source buffer.
    dmaTX_.TCD->ATTR_DST = sizeof(T) - 1; // N bit mode
    if (tx_buffer) {
        dmaTX_.sourceBuffer(write_data, count * sizeof(T));
        dmaTX_.TCD->SLAST = 0; // Finish with it pointing to next location
    } else {
        T* ptr { reinterpret_cast<T*>(&transfer_write_fill_) };
        dmaTX_.source(*ptr);
        DMA_channel_transfer_count(dmaTX_, count);
    }
    dmaRX_.TCD->ATTR_SRC = sizeof(T) - 1; // N bit mode
    if (rx_buffer) {
        dmaRX_.destinationBuffer(rx_buffer, count * sizeof(T));
        dmaRX_.TCD->DLASTSGA = 0; // At end point after our bufffer
    } else {
        // Write only mode
        dmaRX_.destination(bit_bucket);
        DMA_channel_transfer_count(dmaRX_, count);
    }

    if constexpr (DEBUG_DMA_) {
        dump_dma_tcd(&dmaTX_);
        dump_dma_tcd(&dmaRX_);
    }

    // Make sure port is in N bit mode and clear watermark
    if (sizeof(T) > 1) {
        dma_tcr_saved_ = port().TCR;
        port().TCR = (port().TCR & ~(LPSPI_TCR_FRAMESZ(31))) | LPSPI_TCR_FRAMESZ(sizeof(T) * 8 - 1);
    }
    port().FCR = 0;

    port().DER = LPSPI_DER_TDDE | LPSPI_DER_RDDE; // enable DMA on both TX and RX
    port().SR = 0x3f00; // clear out all of the other status...

    return true;
}

template bool SpiT4::setup_dma_transfer<uint8_t>(const uint8_t*, uint8_t*, size_t);
template bool SpiT4::setup_dma_transfer<uint16_t>(const uint16_t*, uint16_t*, size_t);
template bool SpiT4::setup_dma_transfer<uint32_t>(const uint32_t*, uint32_t*, size_t);

template <SpiT4TransferType T>
bool SpiT4::transfer(const T* tx_buffer, T* rx_buffer, size_t count, EventResponderRef event_responder) {
    event_responder.clearEvent(); // make sure it is not set yet

    if (count < 2) {
        // use non-async version to simplify cases...
        transfer(tx_buffer, rx_buffer, count);
        event_responder.triggerEvent();
        return true;
    }

    if (!setup_dma_transfer(tx_buffer, rx_buffer, count)) {
        return false;
    }

    dma_event_responder_ = &event_responder;
    start_dma_transfer();

    return true;
}

template bool SpiT4::transfer<uint8_t>(const uint8_t*, uint8_t*, size_t, EventResponderRef);
template bool SpiT4::transfer<uint16_t>(const uint16_t*, uint16_t*, size_t, EventResponderRef);
template bool SpiT4::transfer<uint32_t>(const uint32_t*, uint32_t*, size_t, EventResponderRef);

template <SpiT4TransferType T>
bool SpiT4::transfer_os(const T* tx_buffer, T* rx_buffer, size_t count, uint32_t timeout_ms) {
    if (count < 2) {
        // use non-async version to simplify cases...
        transfer(tx_buffer, rx_buffer, count);
        return true;
    }

    if (!setup_dma_transfer(tx_buffer, rx_buffer, count)) {
        return false;
    }

    caller_ = ::xTaskGetCurrentTaskHandle();
    start_dma_transfer();

    if (::ulTaskNotifyTake(pdTRUE, timeout_ms ? pdMS_TO_TICKS(timeout_ms) : portMAX_DELAY) != 1) {
        /* timeout */
        dma_count_remaining_ = 0;
        caller_ = nullptr;
        return false;
    }
    return true;
}

template bool SpiT4::transfer_os<uint8_t>(const uint8_t*, uint8_t*, size_t, uint32_t);
template bool SpiT4::transfer_os<uint16_t>(const uint16_t*, uint16_t*, size_t, uint32_t);
template bool SpiT4::transfer_os<uint32_t>(const uint32_t*, uint32_t*, size_t, uint32_t);

void SpiT4::dma_rxisr() {
    dmaRX_.clearInterrupt();
    dmaTX_.clearComplete();
    dmaRX_.clearComplete();

    if (dma_count_remaining_) {
        // What do I need to do to start it back up again...
        // We will use the BITR/CITR from RX as TX may have prefed some stuff
        if (dma_count_remaining_ > MAX_DMA_COUNT_) {
            dma_count_remaining_ -= MAX_DMA_COUNT_;
        } else {
            DMA_channel_transfer_count(dmaTX_, dma_count_remaining_);
            DMA_channel_transfer_count(dmaRX_, dma_count_remaining_);
            dma_count_remaining_ = 0;
        }
        dmaRX_.enable();
        dmaTX_.enable();
    } else {
        port().FCR = LPSPI_FCR_TXWATER(15); // restore the FSR status...
        port().DER = 0; // DMA no longer doing TX (or RX)

        port().CR = LPSPI_CR_MEN | LPSPI_CR_RRF | LPSPI_CR_RTF; // actually clear both...
        port().SR = 0x3f00; // clear out all of the other status...
        port().TCR = dma_tcr_saved_;

        dma_state_ = DMAState::completed; // set back to 1 in case our call wants to start up dma again
        // delete dmaRX_;
        // dmaRX_ = nullptr;
        // delete dmaTX_;
        // dmaTX_ = nullptr;

        if (caller_) {
            BaseType_t higher_woken { pdFALSE };
            ::vTaskNotifyGiveFromISR(caller_, &higher_woken);
            caller_ = nullptr;
            portYIELD_FROM_ISR(higher_woken);
            portDATA_SYNC_BARRIER(); // mitigate arm errata #838869
            return;
        }

        if (dma_event_responder_) {
            dma_event_responder_->triggerEvent();
            dma_event_responder_ = nullptr;
        }
    }
}

} // namespace freertos

#endif // ARDUINO_TEENSY40 || ARDUINO_TEENSY41
