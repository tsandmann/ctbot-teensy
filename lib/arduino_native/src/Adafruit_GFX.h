#pragma once

#include <cstdint>


class Adafruit_GFX;

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
