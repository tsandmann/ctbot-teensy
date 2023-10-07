/*
 * This file is part of the ct-Bot teensy framework.
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
 * @file    tft_spi.h
 * @brief   TFT display driver base class for SPI connected devices
 * @author  Timo Sandmann
 * @date    11.02.2023
 */

#pragma once

#include <cstdint>
#include <type_traits>


namespace freertos {
class SpiT4;
class SpiT4Settings;
} // namespace freertos


class TFT_SPI {
public:
    constexpr TFT_SPI(freertos::SpiT4* p_spi, uint8_t cs, uint8_t dc, int8_t rst)
        : cs_pin_ { cs }, dc_pin_ { dc }, rst_pin_ { rst }, width_ {}, height_ {}, rotation_ {}, p_spi_ { p_spi } {}

    virtual ~TFT_SPI();

    virtual void begin(uint32_t freq) = 0;

    virtual void set_rotation(uint8_t rotation) = 0;

    virtual void invert_display(bool invert) = 0;

    virtual void set_addr_window(uint16_t x, uint16_t y, uint16_t w, uint16_t h) const = 0;

    void start_write();

    void end_write();

    void draw_framebuffer(const uint16_t* p_buffer);

protected:
    const uint8_t cs_pin_;
    const uint8_t dc_pin_;
    const int8_t rst_pin_;

    uint16_t width_;
    uint16_t height_;
    uint8_t rotation_;
    freertos::SpiT4* p_spi_;
    freertos::SpiT4Settings* p_spi_settings_;

    void dc_low() const;

    void dc_high() const;

    void spi_init(uint32_t freq = SPI_DEFAULT_FREQ);

private:
    static constexpr uint32_t SPI_DEFAULT_FREQ { 24'000'000 };
};
