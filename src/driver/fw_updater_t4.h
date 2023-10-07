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
 * @file    fw_updater_t4.h
 * @brief   Teensy 4.1 firmware updater
 * @author  Timo Sandmann
 * @date    25.03.2023
 */

#pragma once

#if defined ARDUINO_TEENSY41
#include "driver/serial_io.h"
#include "avr/pgmspace.h"

#include <cstdint>


namespace ctbot {
class CtBot;

class FwUpdaterT4 {
    static constexpr uint8_t DEBUG_LEVEL_ { 3 }; // 0: off; 1: errors; 2: warnings; 3: info; 4: verbose

    static constexpr uint32_t FLASH_SECTOR_SIZE { 0x1000 }; // Byte
    static constexpr uint32_t FLASH_WRITE_SIZE { 4 }; // Byte
    static constexpr uintptr_t FLASH_START_ADDR { 0x6000'0000 };
    static constexpr uintptr_t RAM_BUFFER_ADDR { 0x7080'0000 };
    static constexpr size_t RAM_BUFFER_SIZE { 8 * (1 << 20) };
    static constexpr uintptr_t FW_SIZE_ADDR { 0x707F'FFFC };

    FLASHMEM static bool ram_upload(CtBot& ctbot, freertos::SerialIO& io, void* dest, uint32_t size, uint32_t crc32);

    FLASHMEM static void flash_firmware_ram();

    FASTRUN __attribute__((noinline, noclone)) static bool flash_sector_not_erased(uint32_t address);

    FASTRUN __attribute__((noinline, noclone)) [[noreturn]] static void flash_move(uint8_t* __restrict p_dst, const uint8_t* __restrict p_src, uint32_t size);

    FASTRUN [[noreturn]] static void hard_reset();

public:
    FLASHMEM static bool start_flash_firmware(CtBot& ctbot, freertos::SerialIO& io, uint32_t file_size, uint32_t crc32, bool ram_buffer);
};
} // namespace ctbot

#endif // ARDUINO_TEENSY41
