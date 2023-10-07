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
 * @file    fw_updater_t4.cpp
 * @brief   Teensy 4.1 firmware updater
 * @author  Timo Sandmann
 * @date    25.03.2023
 */


#if defined ARDUINO_TEENSY41

#include "fw_updater_t4.h"
#include "ctbot.h"
#include "timer.h"

#include "arduino_freertos.h"
#include "crc32.h"

#include <cstring>


extern "C" {
extern uint8_t external_psram_size;

void eepromemu_flash_write(void* addr, const void* data, uint32_t len);
void eepromemu_flash_erase_sector(void* addr);
void eepromemu_flash_erase_32K_block(void* addr);
void eepromemu_flash_erase_64K_block(void* addr);
}

namespace ctbot {
FLASHMEM bool FwUpdaterT4::ram_upload(CtBot& ctbot, freertos::SerialIO& io, void* dest, uint32_t size, uint32_t crc32) {
    uint8_t* ptr { static_cast<uint8_t*>(dest) };

    const auto rv { ctbot.get_comm()->get_viewer_enabled() };
    ctbot.get_comm()->enable_remoteviewer(false);

    while (DEBUG_LEVEL_ >= 3 && !io.available()) {
        ::vTaskDelay(1);
    }
    io.get_rx_overflow(true); // clear rx overflow status
    const auto start { Timer::get_ms() };

    uint32_t len {};
    uint32_t feedback_points {};
    for (; len < size;) {
        while (!io.available()) {
            ::vTaskDelay(1);
            if (Timer::get_ms() - start > (size / 10 + 1)) { // timeout for less than 10 kB/s
                ctbot.get_comm()->enable_remoteviewer(rv);
                return false;
            }
        }

        const uint8_t tmp = io.read(true);
        *ptr++ = tmp;
        ++len;

        if (io.get_rx_overflow(true)) {
            if constexpr (DEBUG_LEVEL_ >= 1) {
                ctbot.get_logger()->begin("FwUpdaterT4::ram_upload(): ");
                ctbot.get_logger()->log<true>(PSTR("ERROR: RX overflow\r\n"));
            }
            ctbot.get_comm()->enable_remoteviewer(rv);
            return false;
        }

        if (len * 80 / size >= feedback_points) {
            ctbot.get_comm()->debug_print('*', false);
            ++feedback_points;
        }
    }
    ctbot.get_comm()->debug_print(PSTR("\r\n"), true);

    if constexpr (DEBUG_LEVEL_ >= 3) {
        const auto dt { Timer::get_ms() - start };
        ctbot.get_comm()->debug_printf<true>(
            PSTR("upload of %u bytes took %u ms (%.2f kB/s).\r\n"), len, dt, static_cast<float>(len) / 1.024f / static_cast<float>(dt));
    }

    ctbot.get_comm()->enable_remoteviewer(rv);

    if (crc32) {
        CRC32 crc;
        const auto ram_crc { crc.calculate(static_cast<uint8_t*>(dest), size) };
        if (ram_crc != crc32) {
            if constexpr (DEBUG_LEVEL_ >= 1) {
                ctbot.get_comm()->debug_printf<true>(PSTR("CRC32 of data: 0x%x\r\nCRC32 requsted: 0x%x\r\n"), ram_crc, crc32);
            }
            return false;
        }
    }

    return true;
}

FLASHMEM bool FwUpdaterT4::start_flash_firmware(CtBot& ctbot, freertos::SerialIO& io, uint32_t file_size, uint32_t crc32, bool ram_buffer) {
    if constexpr (!CtBotConfig::FLASH_OVER_WIFI_AVAILABLE || !CtBotConfig::UART_WIFI_FOR_CMD) {
        return false;
    }

    ctbot.get_comm()->debug_printf<true>(PSTR("FwUpdaterT4::start_flash_firmware(%u, 0x%x, %u)\r\n"), file_size, crc32, ram_buffer);

    if (!ram_buffer) {
        return false; // not implemented
    } else {
        if (external_psram_size != 16) {
            if constexpr (DEBUG_LEVEL_ >= 1) {
                ::serialport_puts(PSTR("FwUpdaterT4::start_flash_firmware(): external RAM buffer not available, abort.\r\n"));
            }
            return false;
        }

        if (!ram_upload(ctbot, io, reinterpret_cast<void*>(RAM_BUFFER_ADDR), file_size, crc32)) {
            return false;
        }
        *reinterpret_cast<uint32_t*>(FW_SIZE_ADDR) = file_size;

        std::atexit([]() FLASHMEM { FwUpdaterT4::flash_firmware_ram(); });
    }

    ctbot.stop();

    return true;
}

FLASHMEM void FwUpdaterT4::flash_firmware_ram() {
    if constexpr (!CtBotConfig::FLASH_OVER_WIFI_AVAILABLE || !CtBotConfig::UART_WIFI_FOR_CMD) {
        hard_reset();
    }

    if constexpr (DEBUG_LEVEL_ >= 3) {
        ::serialport_puts(PSTR("FwUpdaterT4::flash_firmware_ram() called.\r\n"));
    }

    if (external_psram_size != 16) {
        if constexpr (DEBUG_LEVEL_ >= 1) {
            ::serialport_puts(PSTR("FwUpdaterT4::flash_firmware_ram(): no external RAM buffer available, abort.\r\n"));
        }
        hard_reset();
    }
    auto p_buffer { reinterpret_cast<const uint8_t*>(RAM_BUFFER_ADDR) };
    auto p_flash_start { reinterpret_cast<uint8_t*>(FLASH_START_ADDR) };

    const auto fw_size { *reinterpret_cast<const uint32_t*>(FW_SIZE_ADDR) };
    if (RAM_BUFFER_SIZE < fw_size) {
        ::serialport_puts(PSTR("FwUpdaterT4::flash_firmware_ram(): buffer to small for firmware, abort.\r\n"));
        hard_reset();
    }

    if constexpr (DEBUG_LEVEL_ >= 4) {
        ::Serial.printf(
            PSTR("FwUpdaterT4::flash_firmware_ram(): &p_buffer=0x%x buffer_size=%u kB firmware_size=%u\r\n"), p_buffer, RAM_BUFFER_SIZE / 1'024, fw_size);
    }

    if (std::memcmp(p_flash_start, p_buffer, 128ul * sizeof(uint32_t)) != 0) {
        if constexpr (DEBUG_LEVEL_ >= 1) {
            ::serialport_puts(PSTR("FwUpdaterT4::flash_firmware_ram(): invalid flashconfig in new binary, abort.\r\n"));

            if constexpr (DEBUG_LEVEL_ >= 4) {
                for (size_t i {}; i < 128; ++i) {
                    ::Serial.printf(PSTR("0x%x "), reinterpret_cast<const uint32_t*>(p_buffer)[i]);
                    if (i % 16 == 0) {
                        ::Serial.println();
                    }
                }
            }
        }
        hard_reset();
    }
    ::serialport_puts(PSTR("FwUpdaterT4::flash_firmware_ram(): flashconfig is valid, start writing to flash...\r\n"));

    // move new program from buffer to flash and reboot
    flash_move(p_flash_start, p_buffer, fw_size);
}

void FwUpdaterT4::hard_reset() {
    __disable_irq();
    SRC_SRSR = SRC_SRSR; // clear SRC Reset Status Register

    /* reset with watchdog */
    IOMUXC_GPR_GPR16 = 0x0020'0007; // enable ITCM initialization, enable DTCM initialization, use FLEXRAM_BANK_CFG, reset VTOR to ROM vector table
    WDOG1_WCR = WDOG_WCR_WDA | WDOG_WCR_SRS | WDOG_WCR_WDE;
    while (true) {
    }
    __builtin_unreachable();
}

bool FwUpdaterT4::flash_sector_not_erased(uint32_t address) {
    auto sector { reinterpret_cast<uint32_t*>(address & ~(FLASH_SECTOR_SIZE - 1)) };
    for (size_t i {}; i < FLASH_SECTOR_SIZE / 4; ++i) {
        if (*sector++ != 0xffff'ffff) {
            return true; // not erased
        }
    }

    return false; // erased
}

void FwUpdaterT4::flash_move(uint8_t* __restrict p_dst, const uint8_t* __restrict p_src, uint32_t size) {
    /* disable interrupts */
    __asm__ volatile("msr basepri, %0" ::"r"(1) : "memory");
    __asm__ volatile("dsb" ::: "memory");
    __asm__ volatile("isb" ::: "memory");

    /* move size bytes containing new program from source to destination */
    for (uint32_t offset {}; offset < size; offset += FLASH_WRITE_SIZE) {
        const uint32_t addr { reinterpret_cast<uint32_t>(p_dst) + offset };

        /* erase if new sector */
        if ((addr & (FLASH_SECTOR_SIZE - 1)) == 0) {
            if (flash_sector_not_erased(addr)) {
                ::eepromemu_flash_erase_sector(reinterpret_cast<void*>(addr));
            }
        }

        /* data address passed to flash_write() must be in RAM */
        const uint32_t value { *reinterpret_cast<const uint32_t*>(p_src + offset) };
        ::eepromemu_flash_write(reinterpret_cast<void*>(addr), &value, 4);
    }

    hard_reset();
}
} // namespace ctbot

#endif // ARDUINO_TEENSY41
