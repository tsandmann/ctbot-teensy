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
 * @file    tft_spi.cpp
 * @brief   TFT display driver base class for SPI connected devices
 * @author  Timo Sandmann
 * @date    11.02.2023
 */

#include "tft_spi.h"

#include "arduino_freertos.h"
#include "SPI.h"


TFT_SPI::TFT_SPI(SPIClass* p_spi, uint8_t cs, uint8_t dc, int8_t rst)
    : cs_pin_ { cs }, dc_pin_ { dc }, rst_pin_ { rst }, width_ {}, height_ {}, rotation_ {}, p_spi_ { p_spi } {}

TFT_SPI::~TFT_SPI() {
    delete p_spi_settings_;
}

void TFT_SPI::spi_init(uint32_t freq) {
    arduino::pinMode(cs_pin_, arduino::OUTPUT);
    arduino::digitalWriteFast(cs_pin_, arduino::HIGH); // deselect

    arduino::pinMode(dc_pin_, arduino::OUTPUT);
    arduino::digitalWriteFast(dc_pin_, arduino::HIGH); // data mode

    using namespace arduino;
    p_spi_settings_ = new SPISettings { freq, arduino::MSBFIRST, SPI_MODE0 };
    configASSERT(p_spi_settings_);
    p_spi_->begin();

    if (rst_pin_ >= 0) {
        /* toggle to reset */
        arduino::pinMode(rst_pin_, arduino::OUTPUT);
        arduino::digitalWriteFast(rst_pin_, arduino::HIGH);
        ::vTaskDelay(pdMS_TO_TICKS(100));
        arduino::digitalWriteFast(rst_pin_, arduino::LOW);
        ::vTaskDelay(pdMS_TO_TICKS(100));
        arduino::digitalWriteFast(rst_pin_, arduino::HIGH);
        ::vTaskDelay(pdMS_TO_TICKS(200));
    }
}

void TFT_SPI::dc_low() const {
    arduino::digitalWriteFast(dc_pin_, arduino::LOW);
}

void TFT_SPI::dc_high() const {
    arduino::digitalWriteFast(dc_pin_, arduino::HIGH);
}

void TFT_SPI::start_write() {
    p_spi_->beginTransaction(*p_spi_settings_);
    arduino::digitalWriteFast(cs_pin_, arduino::LOW);
}

void TFT_SPI::end_write() {
    arduino::digitalWriteFast(cs_pin_, arduino::HIGH);
    p_spi_->endTransaction();
}

void TFT_SPI::draw_framebuffer(const uint16_t* p_buffer) {
    start_write();
    set_addr_window(0, 0, width_, height_);
    p_spi_->transfer_os<uint16_t>(p_buffer, nullptr, width_ * height_);
    end_write();
}
