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
    XPT2046_Touchscreen(uint8_t, SPIClass*, uint8_t tirq = 255) {
        (void) tirq;
    }

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
