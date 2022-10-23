#pragma once

#include "SPI.h"

#include <cstdint>


class TS_Point {
public:
    TS_Point() : x {}, y {}, z {} {}

    TS_Point(int16_t x, int16_t y, int16_t z) : x { x }, y { y }, z { z } {}

    bool operator==(TS_Point p) {
        return ((p.x == x) && (p.y == y) && (p.z == z));
    }

    bool operator!=(TS_Point p) {
        return ((p.x != x) || (p.y != y) || (p.z != z));
    }

    int16_t x, y, z;
};


class XPT2046_Touchscreen {
public:
    XPT2046_Touchscreen(uint8_t cspin) : XPT2046_Touchscreen { cspin, &SPI } {}
    XPT2046_Touchscreen(uint8_t cspin, SPIClass* p_spi) : XPT2046_Touchscreen { 500, cspin, p_spi } {}
    XPT2046_Touchscreen(uint16_t z_threshold, uint8_t cspin) : XPT2046_Touchscreen { z_threshold, cspin, &SPI } {}
    XPT2046_Touchscreen(uint16_t z_threshold, uint8_t cspin, SPIClass* p_spi) : XPT2046_Touchscreen { z_threshold, cspin, p_spi, 255 } {}
    XPT2046_Touchscreen(uint16_t, uint8_t, SPIClass*, uint8_t) {}

    bool begin() {
        return false;
    }

    void setRotation(uint8_t) {}

    bool touched() {
        return false;
    }

    TS_Point getPoint() {
        return {};
    }

    void readData(uint16_t*, uint16_t*, uint8_t*) {}
};
