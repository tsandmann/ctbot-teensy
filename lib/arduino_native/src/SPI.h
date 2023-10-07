/*
 * Copyright (c) 2023 Timo Sandmann
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
 * @file    SPI.h
 * @brief   SPI dummy for a POSIX environment
 * @author  Timo Sandmann
 * @date    19.08.2023
 */

#pragma once

#include <concepts>
#include <cstdint>
#include <cstddef>


class EventResponder {};


class SPISettings {
public:
    SPISettings(uint32_t, uint8_t, uint8_t) {}
};


template <typename T>
concept SPITransferType = std::same_as<uint8_t, T> || std::same_as<uint16_t, T> || std::same_as<uint32_t, T>;

class SPIClass {
public:
    void begin() {}

    uint8_t setCS(uint8_t) {
        return 0;
    }

    void setMOSI(uint8_t) {}

    void setMISO(uint8_t) {}

    void setSCK(uint8_t) {}

    void beginTransaction(SPISettings&) {}

    void endTransaction() {}

    template <SPITransferType T>
    T transfer(const T) {
        return T {};
    }

    uint8_t transfer(int) {
        return 0;
    }

    uint16_t transfer16(uint16_t data) {
        return transfer(data);
    }

    uint32_t transfer32(uint32_t data) {
        return transfer(data);
    }

    void transfer(void* buf, size_t count) {
        transfer(reinterpret_cast<uint8_t*>(buf), count);
    }

    template <SPITransferType T>
    void transfer(T* buf, size_t count) {
        transfer(buf, buf, count);
    }

    void setTransferWriteFill(uint32_t) {}

    template <SPITransferType T>
    void transfer(const T*, T*, size_t) {}

    template <SPITransferType T>
    bool transfer(const T*, T*, size_t, EventResponder&) {
        return false;
    }

    template <SPITransferType T>
    bool transfer_os(const T*, T*, size_t) {
        return false;
    }
};

extern SPIClass SPI;
extern SPIClass SPI1;
extern SPIClass SPI2;


class EventResponder;


namespace freertos {

class SpiT4Settings {
public:
    static constexpr uint8_t LSB_FIRST { 0 };
    static constexpr uint8_t MSB_FIRST { 1 };
    static constexpr uint8_t MODE_0 { 0x0 };
    static constexpr uint8_t MODE_1 { 0x4 };
    static constexpr uint8_t MODE_2 { 0x8 };
    static constexpr uint8_t MODE_3 { 0xC };

    constexpr SpiT4Settings(uint32_t, uint8_t, uint8_t) {}

    constexpr SpiT4Settings() : SpiT4Settings { 0, MSB_FIRST, MODE_0 } {}

    uint32_t get_clock() const {
        return 0;
    }

    uint32_t get_tcr() const {
        return 0;
    }
};


class SpiT4 {
public:
    static constexpr uint8_t MAX_BUS_ { 2 };

    constexpr SpiT4(uintptr_t, const void*) {}

    void begin() {}

    void setMOSI(uint8_t) {}

    void setMISO(uint8_t) {}

    void setSCK(uint8_t) {}

    uint8_t setCS(uint8_t) {
        return 0;
    };

    void beginTransaction(SpiT4Settings&) {}

    void endTransaction() {}

    template <typename T>
    T transfer(const T) {
        return T {};
    }

    template <typename T>
    void transfer(T* buf, size_t count) {
        transfer(buf, buf, count);
    }

    void setTransferWriteFill(uint32_t) {}

    template <typename T>
    void transfer(const T*, T*, size_t) {}

    template <typename T>
    bool transfer(const T*, T*, size_t, EventResponder&) {
        return false;
    }

    template <typename T>
    bool transfer_os(const T*, T*, size_t) {
        return false;
    }
};

template <uint8_t BUS>
SpiT4* get_spi() {
    static_assert(BUS <= SpiT4::MAX_BUS_, "invalid BUS.");

    switch (BUS) {
        case 0: {
            static SpiT4 instance { 0, nullptr };
            return &instance;
        }
        case 1: {
            static SpiT4 instance { 1, nullptr };
            return &instance;
        }
        case 2: {
            static SpiT4 instance { 2, nullptr };
            return &instance;
        }
    }
};
} // namespace freertos
