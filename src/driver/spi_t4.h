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
 * @file    spi_t4.h
 * @brief   Teensy 4.x SPI FreeRTOS driver
 * @author  Timo Sandmann
 * @date    05.08.2023
 */

#pragma once

#if defined ARDUINO_TEENSY40 || defined ARDUINO_TEENSY41

#include "arduino_freertos.h"
#include "DMAChannel.h"
#include "EventResponder.h"

#include <cstdint>
#include <concepts>


namespace freertos {

template <typename T>
concept SpiT4TransferType = std::same_as<uint8_t, T> || std::same_as<uint16_t, T> || std::same_as<uint32_t, T>;


class SpiT4Settings {
public:
    static constexpr uint8_t LSB_FIRST { 0 };
    static constexpr uint8_t MSB_FIRST { 1 };
    static constexpr uint8_t MODE_0 { 0x0 };
    static constexpr uint8_t MODE_1 { 0x4 };
    static constexpr uint8_t MODE_2 { 0x8 };
    static constexpr uint8_t MODE_3 { 0xC };

    constexpr SpiT4Settings(uint32_t clock, uint8_t bit_order, uint8_t data_mode) : clock_ { clock } {
        tcr_ = LPSPI_TCR_FRAMESZ(7);

        // handle LSB setup
        if (bit_order == LSB_FIRST) {
            tcr_ |= LPSPI_TCR_LSBF;
        }

        // Handle Data Mode
        if (data_mode & 0x08) {
            tcr_ |= LPSPI_TCR_CPOL;
        }

        // Note: On T3.2 when we set CPHA it also updated the timing. It moved the
        // PCS to SCK Delay Prescaler into the After SCK Delay Prescaler
        if (data_mode & 0x04) {
            tcr_ |= LPSPI_TCR_CPHA;
        }
    }

    constexpr SpiT4Settings() : SpiT4Settings { 4'000'000, MSB_FIRST, MODE_0 } {}

    auto get_clock() const {
        return clock_;
    }

    auto get_tcr() const {
        return tcr_;
    }

protected:
    uint32_t clock_;
    uint32_t tcr_; // transmit command, pg 2664 (RT1050 ref, rev 2)
};


class SpiT4 {
    static constexpr bool DEBUG_ { false };
    static constexpr bool DEBUG_DMA_ { false };

public:
    static constexpr uint8_t MAX_BUS_ { 2 };
    static constexpr uint8_t CNT_MISO_PINS_ { 2 };
    static constexpr uint8_t CNT_MOSI_PINS_ { 2 };
    static constexpr uint8_t CNT_SCK_PINS_ { 2 };
    static constexpr uint8_t CNT_CS_PINS_ { 3 };

    struct Hardware_t {
        volatile uint32_t* clock_gate_register;
        uint32_t clock_gate_mask;
        void (*dma_rxisr)();
        uint8_t tx_dma_channel;
        uint8_t rx_dma_channel;
        // MISO pins
        uint8_t miso_pin[CNT_MISO_PINS_];
        uint32_t miso_mux[CNT_MISO_PINS_];
        uint8_t miso_select_val[CNT_MISO_PINS_];
        volatile uint32_t* miso_select_input_register;

        // MOSI pins
        uint8_t mosi_pin[CNT_MOSI_PINS_];
        uint32_t mosi_mux[CNT_MOSI_PINS_];
        uint8_t mosi_select_val[CNT_MOSI_PINS_];
        volatile uint32_t* mosi_select_input_register;

        // SCK pins
        uint8_t sck_pin[CNT_SCK_PINS_];
        uint32_t sck_mux[CNT_SCK_PINS_];
        uint8_t sck_select_val[CNT_SCK_PINS_];
        volatile uint32_t* sck_select_input_register;

        // CS Pins
        uint8_t cs_pin[CNT_CS_PINS_];
        uint32_t cs_mux[CNT_CS_PINS_];
        uint8_t cs_mask[CNT_CS_PINS_];
        uint8_t pcs_select_val[CNT_CS_PINS_];
        volatile uint32_t* pcs_select_input_register[CNT_CS_PINS_];
    } __attribute__((packed));

    static const Hardware_t lpspi4_hardware_;
    static const Hardware_t lpspi3_hardware_;
    static const Hardware_t lpspi1_hardware_;

public:
    SpiT4(uintptr_t port, const Hardware_t* hardware) : port_addr_ { port }, hardware_addr_ { hardware } {}

    constexpr SpiT4& operator=(const SpiT4& rhs) {
        port_addr_ = rhs.port_addr_;
        hardware_addr_ = rhs.hardware_addr_;

        return *this;
    }

    void begin();

    // If SPI is to used from within an interrupt, this function registers
    // that interrupt with the SPI library, so beginTransaction() can
    // prevent conflicts. n is the number used with attachInterrupt().
    void usingInterrupt(uint8_t n) {
        if (n >= CORE_NUM_DIGITAL) {
            return;
        }

        usingInterrupt(IRQ_GPIO6789);
    }

    void usingInterrupt(IRQ_NUMBER_t interrupt);

    void notUsingInterrupt(IRQ_NUMBER_t interrupt);

    void beginTransaction(SpiT4Settings&& settings) {
        SpiT4Settings tmp { settings };
        beginTransaction(tmp);
    }

    // Before using SPI.transfer() or asserting chip select pins,
    // this function is used to gain exclusive access to the SPI bus
    // and configure the correct settings.
    void beginTransaction(SpiT4Settings& settings);

    template <SpiT4TransferType T>
    T transfer(const T data) {
        uint32_t tcr;
        if (sizeof(T) > 1) {
            tcr = port().TCR;
            port().TCR = (tcr & 0xfffff000) | LPSPI_TCR_FRAMESZ(sizeof(T) * 8 - 1); // turn on N bit mode
        }

        port().TDR = data; // output data
        while ((port().RSR & LPSPI_RSR_RXEMPTY)) {
            // wait while the RSR fifo is empty...
        }

        if (sizeof(T) > 1) {
            port().TCR = tcr; // restore TCR
        }

        return static_cast<T>(port().RDR);
    }

    uint8_t transfer(const int data) {
        return transfer(static_cast<uint8_t>(data));
    }

    uint16_t transfer16(const uint16_t data) {
        return transfer(data);
    }

    uint32_t transfer32(const uint32_t data) {
        return transfer(data);
    }

    void transfer(void* tx_buffer, size_t count) {
        transfer(reinterpret_cast<uint8_t*>(tx_buffer), count);
    }

    template <SpiT4TransferType T>
    void transfer(T* tx_buffer, size_t count) {
        transfer(tx_buffer, tx_buffer, count);
    }

    void setTransferWriteFill(uint32_t ch) {
        transfer_write_fill_ = ch;
    }

    template <SpiT4TransferType T>
    void transfer(const T* tx_buffer, T* retbuf, size_t count);

    template <SpiT4TransferType T>
    bool transfer(const T* tx_buffer, T* rx_buffer, size_t count, EventResponderRef event_responder);

    template <SpiT4TransferType T>
    bool transfer_os(const T* tx_buffer, T* rx_buffer, size_t count, uint32_t timeout_ms = 0);

    // After performing a group of transfers and releasing the chip select
    // signal, this function allows others to access the SPI bus
    void endTransaction();

    void end();

    void setMOSI(uint8_t pin);

    void setMISO(uint8_t pin);

    void setSCK(uint8_t pin);

    // return true if "pin" has special chip select capability
    uint8_t pinIsChipSelect(uint8_t pin) const;

    bool pinIsMOSI(uint8_t pin) const;

    bool pinIsMISO(uint8_t pin) const;

    bool pinIsSCK(uint8_t pin) const;

    // return true if both pin1 and pin2 have independent chip select capability
    bool pinIsChipSelect(uint8_t pin1, uint8_t pin2) const;

    // configure a pin for chip select and return its SPI_MCR_PCSIS bitmask
    // setCS() is a special function, not intended for use from normal Arduino
    // programs/sketches. See the ILI3941_t3 library for an example.
    uint8_t setCS(uint8_t pin);

private:
    static constexpr size_t MAX_DMA_COUNT_ { 32'767 };

    static SpiT4* p_instances_[MAX_BUS_ + 1];

    static void DMA_channel_transfer_count(DMAChannel& channel, uint32_t len);

    static void dump_dma_tcd(const DMABaseClass* dmabc);

    template <uint8_t BUS>
    FASTRUN static void isr_dma() {
        static_assert(BUS <= MAX_BUS_, "invalid BUS.");

        SpiT4* p_spi { p_instances_[BUS] };
        p_spi->dma_rxisr();
    }

    uintptr_t port_addr_;
    const Hardware_t* hardware_addr_;
    uint32_t clock_ {};
    uint32_t ccr_ {};
    uint8_t miso_pin_index_ {};
    uint8_t mosi_pin_index_ {};
    uint8_t sck_pin_index_ {};
    uint8_t interruptMasksUsed_ {};
    uint32_t interruptMask_[(NVIC_NUM_INTERRUPTS + 31) / 32] {};
    uint32_t interruptSave_[(NVIC_NUM_INTERRUPTS + 31) / 32] {};

    enum class DMAState : uint8_t { uninitialized, idle, active, completed };

    DMAState dma_state_ { DMAState::uninitialized };
    size_t dma_count_remaining_ {}; // How many bytes left to output after current DMA completes
    DMAChannel dmaTX_ {};
    DMAChannel dmaRX_ {};
    EventResponder* dma_event_responder_ {};
    TaskHandle_t caller_ {};
    uint32_t dma_tcr_saved_ {};

    alignas(4) uint32_t transfer_write_fill_ {};

    IMXRT_LPSPI_t& port() const {
        return *reinterpret_cast<IMXRT_LPSPI_t*>(port_addr_);
    }

    const Hardware_t& hardware() const {
        return *reinterpret_cast<const Hardware_t*>(hardware_addr_);
    }

    bool init_dma_channels();

    template <SpiT4TransferType T>
    bool setup_dma_transfer(const T* buf, T* retbuf, size_t count);

    void start_dma_transfer() {
        dma_state_ = DMAState::active;
        dmaRX_.enable();
        dmaTX_.enable();
    }

    void dma_rxisr();
};

template <uint8_t BUS>
SpiT4* get_spi() {
    static_assert(BUS <= SpiT4::MAX_BUS_, "invalid BUS.");

    switch (BUS) {
        case 0: {
            static SpiT4 instance { IMXRT_LPSPI4_ADDRESS, &SpiT4::lpspi4_hardware_ };
            return &instance;
        }
        case 1: {
            static SpiT4 instance { IMXRT_LPSPI3_ADDRESS, &SpiT4::lpspi3_hardware_ };
            return &instance;
        }
        case 2: {
            static SpiT4 instance { IMXRT_LPSPI1_ADDRESS, &SpiT4::lpspi1_hardware_ };
            return &instance;
        }
    }
};
} // namespace freertos

#endif // ARDUINO_TEENSY40 || ARDUINO_TEENSY41
