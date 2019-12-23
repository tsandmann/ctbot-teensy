#pragma once

class SPIClass {
public:
    uint8_t setCS(uint8_t) {
        return 0;
    }

    void setMOSI(uint8_t) {}

    void setMISO(uint8_t) {}

    void setSCK(uint8_t) {}
};

extern SPIClass SPI;
extern SPIClass SPI1;
extern SPIClass SPI2;
