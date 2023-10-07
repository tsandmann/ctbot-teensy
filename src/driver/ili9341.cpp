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
 * @file    ili9341.cpp
 * @brief   TFT display driver for devices with ILI9341 controller. Compatible with Adafruit's SPITFT interface.
 * @author  Timo Sandmann
 * @date    11.02.2023
 */

#include "ili9341.h"

#include "spi_t4.h"

#include "arduino_freertos.h"


ILI9341::ILI9341(uint8_t cs, uint8_t dc) : ILI9341 { freertos::get_spi<0>(), cs, dc, -1 } {}

ILI9341::ILI9341(uint8_t cs, uint8_t dc, uint8_t rst) : ILI9341 { freertos::get_spi<0>(), cs, dc, static_cast<int8_t>(rst) } {}

ILI9341::ILI9341(freertos::SpiT4* p_spi, uint8_t cs, uint8_t dc, int8_t rst) : TFT_SPI(p_spi, cs, dc, rst) {}

const uint8_t ILI9341::init_cmds_[] = {
    0xEF, 3, 0x03, 0x80, 0x02, //
    0xCF, 3, 0x00, 0xC1, 0x30, //
    0xED, 4, 0x64, 0x03, 0x12, 0x81, //
    0xEF, 3, 0x03, 0x80, 0x02, //
    0xCF, 3, 0x00, 0xC1, 0x30, //
    0xED, 4, 0x64, 0x03, 0x12, 0x81, //
    0xE8, 3, 0x85, 0x00, 0x78, 0xCB, 5, 0x39, 0x2C, 0x00, 0x34, 0x02, //
    0xF7, 1, 0x20, //
    0xEA, 2, 0x00, 0x00, //
    Commands::POWER_CTRL_1, 1, 0x23, // Power control VRH[5:0]
    Commands::POWER_CTRL_2, 1, 0x10, // Power control SAP[2:0];BT[3:0]
    Commands::VCOM_CTRL_1, 2, 0x3e, 0x28, // VCM control
    Commands::VCOM_CTRL_2, 1, 0x86, // VCM control2
    Commands::MEM_ACCESS_CTRL, 1, 0x48, // Memory Access Control
    Commands::V_SCROLL_START_ADDR, 1, 0x00, // Vertical scroll zero
    Commands::INTERFACE_PX_FMT, 1, 0x55, // Pixel Format
    Commands::FRAME_CTRL_1, 2, 0x00, 0x18, // Frame Control
    Commands::DISPLAY_FUNC_CTRL, 3, 0x08, 0x82, 0x27, // Display Function Control
    0xF2, 1, 0x00, // 3Gamma Function Disable
    Commands::GAMMA_SET, 1, 0x01, // Gamma curve selected
    Commands::PGAM_CTRL, 15, 0x0F, 0x31, 0x2B, 0x0C, 0x0E, 0x08, 0x4E, 0xF1, 0x37, 0x07, 0x10, 0x03, 0x0E, 0x09, 0x00, // Set Gamma
    Commands::NGAM_CTRL, 15, 0x00, 0x0E, 0x14, 0x03, 0x11, 0x07, 0x31, 0xC1, 0x48, 0x08, 0x0F, 0x0C, 0x31, 0x36, 0x0F, // Set Gamma
    Commands::SLEEP_OUT, 0x80, // Exit Sleep
    Commands::DISPLAY_ON, 0x80, // Display on
    0 // end of list
};

void ILI9341::write_command(uint8_t cmd) const {
    dc_low();
    p_spi_->transfer<uint16_t>(cmd);
    dc_high();
}

void ILI9341::begin(uint32_t freq) {
    spi_init(freq);

    if (rst_pin_ < 0) {
        /* no reset pin assigned */
        start_write();
        write_command(Commands::SOFT_RESET);
        end_write();
        ::vTaskDelay(pdMS_TO_TICKS(200));

        start_write();
        write_command(Commands::SLEEP_OUT);
        end_write();
        ::vTaskDelay(pdMS_TO_TICKS(120));
    }

    start_write();
    const uint8_t* p_cmd { init_cmds_ };
    uint8_t cmd;
    while ((cmd = *p_cmd++) > 0) {
        write_command(cmd);
        const uint8_t x { *p_cmd++ };
        uint8_t numArgs { static_cast<uint8_t>(x & 0x7f) };
        while (numArgs--) {
            p_spi_->transfer(*p_cmd++);
        }
        if (x & 0x80) {
            ::vTaskDelay(pdMS_TO_TICKS(150));
        }
    }
    end_write();

    width_ = TFT_WIDTH;
    height_ = TFT_HEIGHT;

    set_rotation(0);
}

void ILI9341::set_rotation(uint8_t rotation) {
    rotation_ = rotation % 4u;
    uint8_t val;
    switch (rotation_) {
        case 0:
            val = MAD_CTL::MX | MAD_CTL::BGR;
            width_ = TFT_WIDTH;
            height_ = TFT_HEIGHT;
            break;
        case 1:
            val = MAD_CTL::MV | MAD_CTL::BGR;
            width_ = TFT_HEIGHT;
            height_ = TFT_WIDTH;
            break;
        case 2:
            val = MAD_CTL::MY | MAD_CTL::BGR;
            width_ = TFT_WIDTH;
            height_ = TFT_HEIGHT;
            break;
        case 3:
            val = MAD_CTL::MX | MAD_CTL::MY | MAD_CTL::MV | MAD_CTL::BGR;
            width_ = TFT_HEIGHT;
            height_ = TFT_WIDTH;
            break;
    }

    start_write();
    write_command(Commands::MEM_ACCESS_CTRL);
    p_spi_->transfer(val);
    end_write();
}

void ILI9341::invert_display(bool invert) {
    start_write();
    write_command(invert ? Commands::DISPLAY_INVERSION_ON : Commands::DISPLAY_INVERSION_OFF);
    end_write();
}

void ILI9341::set_addr_window(uint16_t x1, uint16_t y1, uint16_t w, uint16_t h) const {
    const uint16_t x2 { static_cast<uint16_t>(x1 + w - 1u) };
    const uint16_t y2 { static_cast<uint16_t>(y1 + h - 1u) };
    write_command(Commands::COLUMN_ADDR_SET);
    p_spi_->transfer<uint32_t>((x1 << 16) | x2);
    write_command(Commands::PAGE_ADDR_SET);
    p_spi_->transfer<uint32_t>((y1 << 16) | y2);
    write_command(Commands::MEM_WRITE);
}
