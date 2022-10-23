#pragma once

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

    virtual void writeLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color);

    virtual void endWrite() {}

    virtual void setRotation(uint8_t) {}

    virtual void invertDisplay(bool) {}

    virtual void drawFastVLine(int16_t x, int16_t y, int16_t h, uint16_t color) {}

    virtual void drawFastHLine(int16_t x, int16_t y, int16_t w, uint16_t color) {}

    virtual void fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color) {}

    virtual void fillScreen(uint16_t) {}

    virtual void drawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color) {}

    virtual void drawRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color) {}

    void drawCircle(int16_t x0, int16_t y0, int16_t r, uint16_t color) {}

    void drawCircleHelper(int16_t x0, int16_t y0, int16_t r, uint8_t cornername, uint16_t color) {}

    void fillCircle(int16_t x0, int16_t y0, int16_t r, uint16_t color) {}

    void fillCircleHelper(int16_t x0, int16_t y0, int16_t r, uint8_t cornername, int16_t delta, uint16_t color) {}

    void drawTriangle(int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color) {}

    void fillTriangle(int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color) {}

    void drawRoundRect(int16_t x0, int16_t y0, int16_t w, int16_t h, int16_t radius, uint16_t color) {}

    void fillRoundRect(int16_t x0, int16_t y0, int16_t w, int16_t h, int16_t radius, uint16_t color) {}

    void drawBitmap(int16_t x, int16_t y, const uint8_t bitmap[], int16_t w, int16_t h, uint16_t color) {}

    void drawBitmap(int16_t x, int16_t y, const uint8_t bitmap[], int16_t w, int16_t h, uint16_t color, uint16_t bg) {}

    void drawBitmap(int16_t x, int16_t y, uint8_t* bitmap, int16_t w, int16_t h, uint16_t color) {}

    void drawBitmap(int16_t x, int16_t y, uint8_t* bitmap, int16_t w, int16_t h, uint16_t color, uint16_t bg) {}

    void drawXBitmap(int16_t x, int16_t y, const uint8_t bitmap[], int16_t w, int16_t h, uint16_t color) {}

    void drawGrayscaleBitmap(int16_t x, int16_t y, const uint8_t bitmap[], int16_t w, int16_t h) {}

    void drawGrayscaleBitmap(int16_t x, int16_t y, uint8_t* bitmap, int16_t w, int16_t h) {}

    void drawGrayscaleBitmap(int16_t x, int16_t y, const uint8_t bitmap[], const uint8_t mask[], int16_t w, int16_t h) {}

    void drawGrayscaleBitmap(int16_t x, int16_t y, uint8_t* bitmap, uint8_t* mask, int16_t w, int16_t h) {}

    void drawRGBBitmap(int16_t x, int16_t y, const uint16_t bitmap[], int16_t w, int16_t h) {}

    void drawRGBBitmap(int16_t x, int16_t y, uint16_t* bitmap, int16_t w, int16_t h) {}

    void drawRGBBitmap(int16_t x, int16_t y, const uint16_t bitmap[], const uint8_t mask[], int16_t w, int16_t h) {}

    void drawRGBBitmap(int16_t x, int16_t y, uint16_t* bitmap, uint8_t* mask, int16_t w, int16_t h) {}

    void drawChar(int16_t, int16_t, unsigned char, uint16_t, uint16_t, uint8_t) {}

    void setCursor(int16_t, int16_t) {}

    void setTextColor(uint16_t) {}

    void setTextColor(uint16_t, uint16_t) {}

    void setTextSize(uint8_t) {}

    void setTextWrap(bool) {}

    void cp437(bool = true) {}

    void setFont(const GFXfont* = nullptr) {}

    void getTextBounds(const char* string, int16_t x, int16_t y, int16_t* x1, int16_t* y1, uint16_t* w, uint16_t* h) {}

    void getTextBounds(const String& str, int16_t x, int16_t y, int16_t* x1, int16_t* y1, uint16_t* w, uint16_t* h) {}

    virtual size_t write(uint8_t) {}

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

    GFXcanvas16(uint16_t w, uint16_t h, pixel_t*) : Adafruit_GFX { w, h } {}

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
