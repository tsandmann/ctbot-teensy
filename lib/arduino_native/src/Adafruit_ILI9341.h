#pragma once

#include "Adafruit_GFX.h"
#include "SPI.h"
#include "Print.h"

#include <cstdint>
#include <cstddef>


class Adafruit_SPITFT : public Print {
public:
    Adafruit_SPITFT(uint16_t w, uint16_t h, int8_t cs, int8_t dc, int8_t rst) : Adafruit_SPITFT(w, h, &SPI, cs, dc, rst) {}

    Adafruit_SPITFT(uint16_t, uint16_t, SPIClass*, int8_t, int8_t, int8_t) {}

    void drawFramebuffer(const uint16_t*) {}
};


class Adafruit_ILI9341 : public Adafruit_SPITFT {
public:
    static constexpr uint16_t color565(uint8_t red, uint8_t green, uint8_t blue) {
        return ((red & 0xF8) << 8) | ((green & 0xFC) << 3) | (blue >> 3);
    }

    Adafruit_ILI9341(SPIClass* spiClass, int8_t cs, int8_t dc, int8_t rst) : Adafruit_SPITFT(0, 0, spiClass, cs, dc, rst) {}

    Adafruit_ILI9341(int8_t cs, int8_t dc, int8_t rst) : Adafruit_SPITFT(0, 0, cs, dc, rst) {}

    void begin(uint32_t) {}

    void setRotation(uint8_t) {}

    void setTextWrap(bool) {}

    int16_t height() const {
        return 0;
    }

    int16_t width() const {
        return 0;
    }

    int16_t getCursorX() const {
        return 0;
    }

    int16_t getCursorY() const {
        return 0;
    }

    virtual void drawLine(int16_t, int16_t, int16_t, int16_t, uint16_t) {}

    virtual void drawRect(int16_t, int16_t, int16_t, int16_t, uint16_t) {}

    void drawCircle(int16_t, int16_t, int16_t, uint16_t) {}

    void drawTriangle(int16_t, int16_t, int16_t, int16_t, int16_t, int16_t, uint16_t) {}

    virtual void fillRect(int16_t, int16_t, int16_t, int16_t, uint16_t) {}

    void fillCircle(int16_t, int16_t, int16_t, uint16_t) {}

    void fillTriangle(int16_t, int16_t, int16_t, int16_t, int16_t, int16_t, uint16_t) {}

    void drawRoundRect(int16_t, int16_t, int16_t, int16_t, int16_t, uint16_t) {}

    void fillRoundRect(int16_t, int16_t, int16_t, int16_t, int16_t, uint16_t) {}

    virtual void fillScreen(uint16_t color) {
        fillRect(0, 0, 0, 0, color);
    }

    void setCursor(int16_t, int16_t) {}

    void setTextSize(uint8_t) {}

    void setTextColor(uint16_t) {}

    void getTextBounds(const char*, int16_t, int16_t, int16_t*, int16_t*, uint16_t*, uint16_t*) {}

    virtual size_t write(uint8_t) override {
        return 0;
    }

    virtual size_t write(const uint8_t*, size_t) override {
        return 0;
    }

    void drawFastHLine(int16_t, int16_t, int16_t, uint16_t) {}

    void drawFastVLine(int16_t, int16_t, int16_t, uint16_t) {}
};
