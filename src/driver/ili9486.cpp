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
 * @file    ili9486.cpp
 * @brief   TFT display driver for devices with ILI9486 controller
 * @author  Timo Sandmann
 * @date    11.02.2023
 */

#include "ili9486.h"

#include "arduino_freertos.h"
#include "SPI.h"


ILI9486::ILI9486(uint8_t cs, uint8_t dc) : ILI9486 { &SPI, cs, dc, -1 } {}

ILI9486::ILI9486(uint8_t cs, uint8_t dc, uint8_t rst) : ILI9486 { &SPI, cs, dc, static_cast<int8_t>(rst) } {}

ILI9486::ILI9486(SPIClass* p_spi, uint8_t cs, uint8_t dc, int8_t rst) : TFT_SPI(p_spi, cs, dc, rst) {}

const uint8_t ILI9486::init_cmds_[] = {
    Commands::INTERFACE_PX_FMT, 1, 0x55, // Interface Pixel Format: 16 bit color interface
    Commands::POWER_CTRL_3, 1, 0x44, // Power Control 3 (normal mode)
    Commands::VCOM_CTRL_1, 4, 0, 0, 0, 0, // VCOM Control
    Commands::PGAM_CTRL, 15, 0xf, 0x1f, 0x1c, 0xc, 0xf, 0x8, 0x48, 0x98, 0x37, 0xa, 0x13, 0x4, 0x11, 0xd, 0x0, // Positive Gamma Control
    Commands::NGAM_CTRL, 15, 0xf, 0x32, 0x2e, 0xb, 0xd, 0x5, 0x47, 0x75, 0x37, 0x6, 0x10, 0x3, 0x24, 0x20, 0x0, // Negative Gamma Control
    Commands::DISPLAY_INVERSION_OFF, 0, // Display Inversion OFF
    Commands::MEM_ACCESS_CTRL, 1, MAD_CTL::MV | MAD_CTL::BGR, // Memory Access Control
    Commands::DISPLAY_ON, 80, // Display ON
    0 // end of list
};

void ILI9486::write_command(uint8_t cmd) const {
    dc_low();
    p_spi_->transfer<uint16_t>(cmd);
    dc_high();
}

void ILI9486::begin(uint32_t freq) {
    spi_init(freq);

    if (rst_pin_ < 0) {
        set_rotation(1);

        start_write();
        write_command(Commands::SOFT_RESET);
        end_write();
        ::vTaskDelay(pdMS_TO_TICKS(200));


        start_write();
        write_command(Commands::SLEEP_OUT);
        end_write();
        ::vTaskDelay(pdMS_TO_TICKS(120));
    }

    const uint8_t* p_cmd { init_cmds_ };
    uint8_t cmd;
    start_write();
    while ((cmd = *p_cmd++) > 0) {
        write_command(cmd);
        const uint8_t x { *p_cmd++ };
        uint8_t numArgs { static_cast<uint8_t>(x & 0x7f) };
        while (numArgs--) {
            p_spi_->transfer<uint16_t>(*p_cmd++);
        }
        if (x & 0x80) {
            ::vTaskDelay(pdMS_TO_TICKS(150));
        }
    }
    end_write();

    width_ = TFT_WIDTH_;
    height_ = TFT_HEIGHT_;
}

void ILI9486::set_rotation(uint8_t rotation) {
    rotation_ = rotation % 4u;
    uint16_t val;
    switch (rotation_) {
        case 0:
            val = MAD_CTL::MX | MAD_CTL::BGR;
            height_ = TFT_WIDTH_;
            width_ = TFT_HEIGHT_;
            break;
        case 1:
            val = MAD_CTL::MV | MAD_CTL::BGR;
            height_ = TFT_HEIGHT_;
            width_ = TFT_WIDTH_;
            break;
        case 2:
            val = MAD_CTL::MY | MAD_CTL::BGR;
            height_ = TFT_WIDTH_;
            width_ = TFT_HEIGHT_;
            break;
        case 3:
            val = MAD_CTL::MX | MAD_CTL::MY | MAD_CTL::MV | MAD_CTL::BGR;
            height_ = TFT_HEIGHT_;
            width_ = TFT_WIDTH_;
            break;
    }

    start_write();
    write_command(Commands::MEM_ACCESS_CTRL);
    p_spi_->transfer(val);
    end_write();
}

void ILI9486::invert_display(bool invert) {
    start_write();
    write_command(invert ? Commands::DISPLAY_INVERSION_ON : Commands::DISPLAY_INVERSION_OFF);
    end_write();
}

void ILI9486::set_addr_window(uint16_t x1, uint16_t y1, uint16_t w, uint16_t h) const {
    const uint16_t x2 { static_cast<uint16_t>(x1 + w - 1u) };
    const uint16_t y2 { static_cast<uint16_t>(y1 + h - 1u) };
    write_command(Commands::COLUMN_ADDR_SET); // TODO: combine for burst
    p_spi_->transfer<uint32_t>((x1 << 16) | x2);
    write_command(Commands::PAGE_ADDR_SET);
    p_spi_->transfer<uint32_t>((y1 << 16) | y2);
    write_command(Commands::MEM_WRITE);
}
