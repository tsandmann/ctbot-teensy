#pragma once

#include "Print.h"

#include <cstdint>


class GFXfont;

class Adafruit_GFX : public Print {
public:
    Adafruit_GFX(int16_t, int16_t) {}

    virtual ~Adafruit_GFX() = default;

    virtual void drawPixel(int16_t x, int16_t y, uint16_t color) = 0;

    virtual void startWrite() {}

    virtual void writePixel(int16_t x, int16_t y, uint16_t color) {
        drawPixel(x, y, color);
    }

    virtual void writeFillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color) {
        fillRect(x, y, w, h, color);
    }

    virtual void writeFastVLine(int16_t x, int16_t y, int16_t h, uint16_t color) {
        drawFastVLine(x, y, h, color);
    }

    virtual void writeFastHLine(int16_t x, int16_t y, int16_t w, uint16_t color) {
        drawFastHLine(x, y, w, color);
    }

    virtual void writeLine(int16_t, int16_t, int16_t, int16_t, uint16_t) {}

    virtual void endWrite() {}

    virtual void setRotation(uint8_t) {}

    virtual void invertDisplay(bool) {}

    virtual void drawFastVLine(int16_t, int16_t, int16_t, uint16_t) {}

    virtual void drawFastHLine(int16_t, int16_t, int16_t, uint16_t) {}

    virtual void fillRect(int16_t, int16_t, int16_t, int16_t, uint16_t) {}

    virtual void fillScreen(uint16_t) {}

    virtual void drawLine(int16_t, int16_t, int16_t, int16_t, uint16_t) {}

    virtual void drawRect(int16_t, int16_t, int16_t, int16_t, uint16_t) {}

    void drawCircle(int16_t, int16_t, int16_t, uint16_t) {}

    void drawCircleHelper(int16_t, int16_t, int16_t, uint8_t, uint16_t) {}

    void fillCircle(int16_t, int16_t, int16_t, uint16_t) {}

    void fillCircleHelper(int16_t, int16_t, int16_t, uint8_t, int16_t, uint16_t) {}

    void drawTriangle(int16_t, int16_t, int16_t, int16_t, int16_t, int16_t, uint16_t) {}

    void fillTriangle(int16_t, int16_t, int16_t, int16_t, int16_t, int16_t, uint16_t) {}

    void drawRoundRect(int16_t, int16_t, int16_t, int16_t, int16_t, uint16_t) {}

    void fillRoundRect(int16_t, int16_t, int16_t, int16_t, int16_t, uint16_t) {}

    void drawBitmap(int16_t, int16_t, const uint8_t[], int16_t, int16_t, uint16_t) {}

    void drawBitmap(int16_t, int16_t, const uint8_t[], int16_t, int16_t, uint16_t, uint16_t) {}

    void drawBitmap(int16_t, int16_t, uint8_t*, int16_t, int16_t, uint16_t) {}

    void drawBitmap(int16_t, int16_t, uint8_t*, int16_t, int16_t, uint16_t, uint16_t) {}

    void drawXBitmap(int16_t, int16_t, const uint8_t[], int16_t, int16_t, uint16_t) {}

    void drawGrayscaleBitmap(int16_t, int16_t, const uint8_t[], int16_t, int16_t) {}

    void drawGrayscaleBitmap(int16_t, int16_t, uint8_t*, int16_t, int16_t) {}

    void drawGrayscaleBitmap(int16_t, int16_t, const uint8_t[], const uint8_t[], int16_t, int16_t) {}

    void drawGrayscaleBitmap(int16_t, int16_t, uint8_t*, uint8_t*, int16_t, int16_t) {}

    void drawRGBBitmap(int16_t, int16_t, const uint16_t[], int16_t, int16_t) {}

    void drawRGBBitmap(int16_t, int16_t, uint16_t*, int16_t, int16_t) {}

    void drawRGBBitmap(int16_t, int16_t, const uint16_t[], const uint8_t[], int16_t, int16_t) {}

    void drawRGBBitmap(int16_t, int16_t, uint16_t*, uint8_t*, int16_t, int16_t) {}

    void drawChar(int16_t, int16_t, unsigned char, uint16_t, uint16_t, uint8_t) {}

    void setCursor(int16_t, int16_t) {}

    void setTextColor(uint16_t) {}

    void setTextColor(uint16_t, uint16_t) {}

    void setTextSize(uint8_t) {}

    void setTextWrap(bool) {}

    void cp437(bool = true) {}

    void setFont(const GFXfont* = nullptr) {}

    void getTextBounds(const char*, int16_t, int16_t, int16_t*, int16_t*, uint16_t*, uint16_t*) {}

    void getTextBounds(const String&, int16_t, int16_t, int16_t*, int16_t*, uint16_t*, uint16_t*) {}

    virtual size_t write(uint8_t) {
        return 0;
    }

    int16_t height() const {
        return 0;
    }

    int16_t width() const {
        return 0;
    }

    uint8_t getRotation() const {
        return 0;
    }

    int16_t getCursorX() const {
        return 0;
    }
    int16_t getCursorY() const {
        return 0;
    }
};

class GFXcanvas16 : public Adafruit_GFX {
public:
    using pixel_t = uint16_t;

    GFXcanvas16(uint16_t w, uint16_t h, pixel_t*) : Adafruit_GFX { static_cast<int16_t>(w), static_cast<int16_t>(h) } {}

    virtual ~GFXcanvas16() override {}

    virtual void drawPixel(int16_t, int16_t, uint16_t) override {}

    virtual void fillScreen(uint16_t) override {}
};

class Adafruit_GFX_Button {
public:
    void initButtonUL(Adafruit_GFX*, int16_t, int16_t, uint16_t, uint16_t, uint16_t, uint16_t, uint16_t, char*, uint8_t) {}

    void drawButton([[maybe_unused]] bool inverted = false) {}

    bool contains(int16_t, int16_t) const {
        return false;
    }

    void press(bool) {}

    bool justPressed() const {
        return false;
    }

    bool justReleased() const {
        return false;
    }
};
