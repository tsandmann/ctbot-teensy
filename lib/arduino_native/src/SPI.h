#pragma once

#include <concepts>
#include <cstdint>
#include <cstddef>


namespace arduino {
#ifndef LSBFIRST
static constexpr uint8_t LSBFIRST { 0 };
#endif
#ifndef MSBFIRST
static constexpr uint8_t MSBFIRST { 1 };
#endif

static constexpr uint8_t SPI_MODE0 { 0x0 };
static constexpr uint8_t SPI_MODE1 { 0x4 };
static constexpr uint8_t SPI_MODE2 { 0x8 };
static constexpr uint8_t SPI_MODE3 { 0xC };
} // namespace arduino


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
