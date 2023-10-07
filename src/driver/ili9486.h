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
 * @file    ili9486.h
 * @brief   TFT display driver for devices with ILI9486 controller
 * @author  Timo Sandmann
 * @date    11.02.2023
 */

#pragma once

#include "tft_spi.h"

#include <cstdint>


class ILI9486 : public TFT_SPI {
protected:
    static constexpr uint32_t TFT_WIDTH_ { 320 };
    static constexpr uint32_t TFT_HEIGHT_ { 480 };

    static const uint8_t init_cmds_[];

    struct Commands {
        static constexpr uint8_t NOP { 0x0 };
        static constexpr uint8_t SOFT_RESET { 0x1 };
        static constexpr uint8_t READ_DISPLAY_ID { 0x4 };
        static constexpr uint8_t READ_NUM_ERRORS_DSI { 0x5 };
        static constexpr uint8_t READ_DISPLAY_STATUS { 0x9 };
        static constexpr uint8_t READ_DISPLAY_PW_MODE { 0xa };
        static constexpr uint8_t READ_DISPLAY_MADCTL { 0xb };
        static constexpr uint8_t READ_PX_FMT { 0xc };
        static constexpr uint8_t READ_DISPLAY_IMG_MODE { 0xd };
        static constexpr uint8_t READ_DISPLAY_SIGNAL_MODE { 0xe };
        static constexpr uint8_t READ_DISPLAY_SELF_DIAG { 0xf };
        static constexpr uint8_t SLEEP_IN { 0x10 };
        static constexpr uint8_t SLEEP_OUT { 0x11 };
        static constexpr uint8_t PARTIAL_MODE_ON { 0x12 };
        static constexpr uint8_t NORMAL_MODE_ON { 0x13 };
        static constexpr uint8_t DISPLAY_INVERSION_OFF { 0x20 };
        static constexpr uint8_t DISPLAY_INVERSION_ON { 0x21 };
        static constexpr uint8_t DISPLAY_OFF { 0x28 };
        static constexpr uint8_t DISPLAY_ON { 0x29 };
        static constexpr uint8_t COLUMN_ADDR_SET { 0x2a };
        static constexpr uint8_t PAGE_ADDR_SET { 0x2b };
        static constexpr uint8_t MEM_WRITE { 0x2c };
        static constexpr uint8_t MEM_READ { 0x2e };

        static constexpr uint8_t MEM_ACCESS_CTRL { 0x36 };
        static constexpr uint8_t V_SCROLL_START_ADDR { 0x37 };

        static constexpr uint8_t INTERFACE_PX_FMT { 0x3a };

        static constexpr uint8_t POWER_CTRL_1 { 0xc0 };
        static constexpr uint8_t POWER_CTRL_2 { 0xc1 };
        static constexpr uint8_t POWER_CTRL_3 { 0xc2 };

        static constexpr uint8_t VCOM_CTRL_1 { 0xc5 };

        static constexpr uint8_t PGAM_CTRL { 0xe0 };
        static constexpr uint8_t NGAM_CTRL { 0xe1 };
    };

    struct MAD_CTL {
        static constexpr uint8_t MY { 0x80 }; /**< Bottom to top */
        static constexpr uint8_t MX { 0x40 }; /**< Right to left */
        static constexpr uint8_t MV { 0x20 }; /**< Reverse Mode */
        static constexpr uint8_t ML { 0x10 }; /**< LCD refresh Bottom to top */
        static constexpr uint8_t RGB { 0x0 }; /**< Red-Green-Blue pixel order */
        static constexpr uint8_t BGR { 0x8 }; /**< Blue-Green-Red pixel order */
        static constexpr uint8_t MH { 0x4 }; /**< LCD refresh right to left */
    };

public:
    ILI9486(uint8_t cs, uint8_t dc);

    ILI9486(uint8_t cs, uint8_t dc, uint8_t rst);

    ILI9486(freertos::SpiT4* p_spi, uint8_t cs, uint8_t dc) : ILI9486 { p_spi, cs, dc, -1 } {}

    ILI9486(freertos::SpiT4* p_spi, uint8_t cs, uint8_t dc, int8_t rst);

    virtual ~ILI9486() = default;

    void write_command(uint8_t cmd) const;

    virtual void begin(uint32_t freq = SPI_DEFAULT_FREQ) override;

    virtual void set_rotation(uint8_t rotation) override;

    virtual void invert_display(bool invert) override;

    virtual void set_addr_window(uint16_t x, uint16_t y, uint16_t w, uint16_t h) const override;

private:
    static constexpr uint32_t SPI_DEFAULT_FREQ { 24'000'000 };
};
