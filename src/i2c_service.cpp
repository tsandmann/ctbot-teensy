/*
 * This file is part of the ct-Bot teensy framework.
 * Copyright (c) 2021 Timo Sandmann
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
 * @file    i2c_service.cpp
 * @brief   I2C FreeRTOS service
 * @author  Timo Sandmann
 * @date    25.03.2021
 */

#include "i2c_service.h"
#include "queue.h"

#include <atomic>


#if defined ARDUINO_TEENSY40 || defined ARDUINO_TEENSY41
#include "i2c_t4.h"
namespace I2C_NS = arduino::teensy4;
#else
namespace I2C_NS = arduino;
#endif

decltype(I2C_Service::i2c_task_) I2C_Service::i2c_task_;
decltype(I2C_Service::i2c_queue_) I2C_Service::i2c_queue_;
decltype(I2C_Service::p_i2c_) I2C_Service::p_i2c_;
decltype(I2C_Service::init_) I2C_Service::init_;
decltype(I2C_Service::freq_) I2C_Service::freq_;

bool I2C_Service::init(const uint8_t bus, const uint32_t freq, const uint8_t pin_sda, const uint8_t pin_scl) {
    if (bus >= i2c_task_.size()) {
        if (DEBUG_) {
            printf_debug(PSTR("I2C_Service::init(%u, %u, %u, %u): invalid bus.\r\n"), bus, freq, pin_sda, pin_scl);
        }
        return false;
    }

    if (init_[bus] && freq_[bus] == freq) {
        return true;
    }

    if (DEBUG_) {
        printf_debug(PSTR("I2C_Service::init(%u, %u, %u, %u)...\r\n"), bus, freq, pin_sda, pin_scl);
    }

    switch (bus) {
        case 0: {
            p_i2c_[bus] = &I2C_NS::Wire;
            break;
        }

        case 1: {
            p_i2c_[bus] = &I2C_NS::Wire1;
            break;
        }

        case 2: {
            p_i2c_[bus] = &I2C_NS::Wire2;
            break;
        }

#ifdef WIRE_IMPLEMENT_WIRE3
        case 3: {
            p_i2c_[bus] = &I2C_NS::Wire3;
            break;
        }
#endif // WIRE_IMPLEMENT_WIRE3

        default: {
            return false;
        }
    }

    p_i2c_[bus]->begin();
    if (pin_sda != 255) {
        p_i2c_[bus]->setSDA(pin_sda);
    }
    if (pin_scl != 255) {
        p_i2c_[bus]->setSCL(pin_scl);
    }
    p_i2c_[bus]->setClock(freq);

    if (!i2c_task_[bus]) {
        i2c_queue_[bus] = ::xQueueCreate(8, sizeof(I2C_Transfer*));
        if (!i2c_queue_[bus]) {
            init_[bus] = false;
            return false;
        }

        uintptr_t x { bus };
        void* param { reinterpret_cast<void*>(x) };
        if (::xTaskCreate(run, PSTR("I2C Svc"), 512, param, 8, &i2c_task_[bus]) != pdTRUE) {
            if (DEBUG_) {
                printf_debug(PSTR("I2C_Service::init(): xTaskCreate() failed\r\n"));
            }
            ::vQueueDelete(i2c_queue_[bus]);
            init_[bus] = false;
            return false;
        }
    }

    freq_[bus] = freq;
    init_[bus] = true;

    if (DEBUG_) {
        printf_debug(PSTR("I2C_Service::init(%u, %u, %u, %u) done\r\n"), bus, freq, pin_sda, pin_scl);
    }

    return true;
}

void I2C_Service::finish_transfer(const bool success, I2C_Transfer* transfer) {
    auto p_data { transfer };
    if (p_data->callback) {
        p_data->callback(success, &p_data);
    }
    delete p_data;
}

void I2C_Service::run(void* param) {
    const uintptr_t bus { reinterpret_cast<uintptr_t>(param) };
    if (DEBUG_) {
        printf_debug(PSTR("I2C_Service::run(): bus=%u\r\n"), bus);
    }

    I2C_Transfer* transfer {};
    uint8_t buffer[4];
    uint32_t id {};

    while (true) {
        if (::xQueueReceive(i2c_queue_[bus], &transfer, portMAX_DELAY) == pdPASS) {
            if (!transfer) {
                continue;
            }
            if (DEBUG_) {
                printf_debug(PSTR("\r\nI2C_Service::run(): transfer %u for 0x%x received:\r\n"), ++id, transfer->addr);
                if (transfer->size1) {
                    printf_debug(PSTR("\ttype1=%c\tsize1=%u\tdata1=0x%x\r\n"), transfer->type1 ? 'r' : 'w', transfer->size1,
                        transfer->type1 == 0 ? transfer->data1.data : 0);
                }
                if (transfer->size2) {
                    printf_debug(PSTR("\ttype2=%c\tsize2=%u\tdata2=0x%x\r\n"), transfer->type2 ? 'r' : 'w', transfer->size2,
                        transfer->type2 == 0 ? transfer->data2.data : 0);
                }
            }

            if (!transfer->size1) {
                finish_transfer(true, transfer, id);
            }

            p_i2c_[bus]->beginTransmission(transfer->addr);
            if (transfer->type1 == 0) {
                /* 1st transfer write */
                const auto to_send { transfer->size1 };
                if (to_send <= sizeof(I2C_Transfer::data1.data)) {
                    uint8_t* ptr { buffer };
                    for (int i { to_send - 1 }; i >= 0; --i) {
                        *ptr = static_cast<uint8_t>(transfer->data1.data >> (i << 3));
                        if (DEBUG_) {
                            // printf_debug(PSTR("I2C_Service::run(): write buffer[%u]=0x%x\r\n"), ptr - buffer, *ptr);
                        }
                        ++ptr;
                    }
                    transfer->data1.ptr = buffer;
                }

                if (p_i2c_[bus]->write(transfer->data1.ptr, to_send) != to_send) {
                    if (DEBUG_) {
                        printf_debug(PSTR("I2C_Service::run(): write() 1 FAILED\r\n"));
                    }
                    transfer->error = 1;
                    finish_transfer(false, transfer, id);
                    continue;
                }

                if (!transfer->size2 || transfer->type2) {
                    const auto status { p_i2c_[bus]->endTransmission(transfer->size2 && transfer->type2 ? 0 : 1) };
                    if (status) {
                        if (DEBUG_) {
                            printf_debug(PSTR("I2C_Service::run(): endTransmission() FAILED: %u\r\n"), status);
                        }
                        transfer->error = 2;
                        finish_transfer(false, transfer, id);
                        continue;
                    }
                }
            } else {
                /* 1st transfer read */
                transfer->error = 255;
                configASSERT(false); // FIXME: not implemented
            }

            if (transfer->size2 && transfer->type2 == 0) {
                /* 2nd transfer write */
                const auto to_send { transfer->size2 };
                if (to_send <= sizeof(I2C_Transfer::data2.data)) {
                    uint8_t* ptr { buffer };
                    for (auto i { to_send - 1 }; i >= 0; --i) {
                        *ptr = static_cast<uint8_t>(transfer->data2.data >> (i << 3));
                        if (DEBUG_) {
                            // printf_debug(PSTR("I2C_Service::run(): write buffer[%u]=0x%x\r\n"), ptr - buffer, *ptr);
                        }
                        ++ptr;
                    }
                    transfer->data2.ptr = buffer;
                }

                if (p_i2c_[bus]->write(transfer->data2.ptr, to_send) != to_send) {
                    if (DEBUG_) {
                        printf_debug(PSTR("I2C_Service::run(): write() 2 FAILED\r\n"));
                    }
                    transfer->error = 3;
                    finish_transfer(false, transfer, id);
                    continue;
                }

                const auto status { p_i2c_[bus]->endTransmission(1) };
                if (status) {
                    if (DEBUG_) {
                        printf_debug(PSTR("I2C_Service::run(): endTransmission() FAILED: %u\r\n"), status);
                    }
                    transfer->error = 4;
                    finish_transfer(false, transfer, id);
                    continue;
                }
            } else if (transfer->size2) {
                /* 2nd transfer read */
                const auto to_read { transfer->size2 };
                if (p_i2c_[bus]->requestFrom(static_cast<uint8_t>(transfer->addr), to_read, static_cast<uint8_t>(1)) != to_read) {
                    if (DEBUG_) {
                        printf_debug(PSTR("I2C_Service::run(): requestFrom() FAILED\r\n"));
                    }
                    transfer->error = 5;
                    finish_transfer(false, transfer, id);
                    continue;
                }

                if (to_read <= 4) {
                    if (p_i2c_[bus]->readBytes(buffer, to_read) != to_read) {
                        transfer->error = 6;
                        finish_transfer(false, transfer, id);
                        continue;
                    }

                    transfer->data2.data = 0;
                    uint8_t* ptr { buffer };
                    for (auto i { to_read - 1 }; i >= 0; --i) {
                        transfer->data2.data |= static_cast<uint32_t>(*ptr) << (i << 3);
                        if (DEBUG_) {
                            // printf_debug(PSTR("I2C_Service::run(): read buffer[%u]=0x%x\r\n"), ptr - buffer, *ptr);
                        }
                        ++ptr;
                    }
                    if (DEBUG_) {
                        printf_debug(PSTR("I2C_Service::run(): rx_data.data=0x%x\r\n"), transfer->data2.data);
                    }
                } else {
                    /* to_read > 4 */
                    if (p_i2c_[bus]->readBytes(transfer->data2.ptr, to_read) != to_read) {
                        if (DEBUG_) {
                            printf_debug(PSTR("I2C_Service::run(): readBytes() FAILED\r\n"));
                        }
                        transfer->error = 7;
                        finish_transfer(false, transfer, id);
                        continue;
                    }
                }
            }

            finish_transfer(true, transfer, id);
            if (DEBUG_) {
                printf_debug(PSTR("\r\nI2C_Service::run(): transfer %u completed\r\n\n"), id);
            }
        }
    }
}

I2C_Service::I2C_Service(const uint8_t bus, const uint32_t freq, const uint8_t pin_sda, const uint8_t pin_scl) : bus_ { bus } {
    init(bus, freq, pin_sda, pin_scl);
}

uint8_t I2C_Service::read_reg(
    const uint16_t addr, std::unsigned_integral auto const reg, std::integral auto& data, std::function<void(const bool, I2C_Transfer**)> callback) const {
    static_assert(sizeof(reg) <= sizeof(I2C_Transfer::data1.data), "I2C_Service::read_reg<>(): invalid reg size");
    static_assert(sizeof(data) <= sizeof(I2C_Transfer::data2.data), "I2C_Service::read_reg<>(): invalid data size");

    std::atomic<bool> transfer_done;
    I2C_Transfer* transfer { new I2C_Transfer { addr, sizeof(reg), false, sizeof(data), true } };
    transfer->data1.data = reg;

    uint8_t ret;
    if (callback) {
        transfer->callback = callback;
        ret = 0;
    } else {
        transfer_done = false;
        transfer->callback = [&data, &transfer_done, &ret](const bool done, I2C_Transfer** p_transfer) {
            if (done) {
                data = static_cast<std::remove_reference<decltype(data)>::type>((*p_transfer)->data2.data);
            }
            ret = (*p_transfer)->error;
            if (DEBUG_) {
                printf_debug(PSTR("I2C_Service::read_reg<%u, %u>() callback, data=0x%x, err=%u\r\n"), sizeof(reg), sizeof(data), data, ret);
            }
            transfer_done = true;
        };
    }

    if (::xQueueSend(i2c_queue_[bus_], &transfer, portMAX_DELAY) != pdTRUE) { // FIXME: timeout?
        return 100;
    }

    if (!callback) {
        while (!transfer_done) { // FIXME: timeout?
            portYIELD();
        }
        if (DEBUG_) {
            // printf_debug(PSTR("I2C_Service::read_reg<%u, %u>(): done, ret=%u, data=%u\r\n"), sizeof(REG), sizeof(DATA), ret, data);
        }
    }

    return ret;
}

template uint8_t I2C_Service::read_reg<uint8_t, uint8_t>(const uint16_t, const uint8_t, uint8_t&, std::function<void(const bool, I2C_Transfer**)>) const;
template uint8_t I2C_Service::read_reg<uint16_t, uint8_t>(const uint16_t, const uint16_t, uint8_t&, std::function<void(const bool, I2C_Transfer**)>) const;
template uint8_t I2C_Service::read_reg<uint8_t, uint16_t>(const uint16_t, const uint8_t, uint16_t&, std::function<void(const bool, I2C_Transfer**)>) const;
template uint8_t I2C_Service::read_reg<uint16_t, uint16_t>(const uint16_t, const uint16_t, uint16_t&, std::function<void(const bool, I2C_Transfer**)>) const;
template uint8_t I2C_Service::read_reg<uint8_t, uint32_t>(const uint16_t, const uint8_t, uint32_t&, std::function<void(const bool, I2C_Transfer**)>) const;
template uint8_t I2C_Service::read_reg<uint16_t, uint32_t>(const uint16_t, const uint16_t, uint32_t&, std::function<void(const bool, I2C_Transfer**)>) const;

uint8_t I2C_Service::read_bytes(const uint16_t addr, std::unsigned_integral auto const reg_addr, uint8_t* p_data, const uint8_t length,
    std::function<void(const bool, I2C_Transfer**)> callback) const {
    static_assert(sizeof(reg_addr) <= sizeof(I2C_Transfer::data1.data), "I2C_Service::read_bytes<>(): invalid reg_addr size");

    if (length > 32) {
        return 1;
    }

    switch (length) {
        case 0: {
            return 0;
        }
        case 1: {
            uint8_t tmp;
            const auto ret { read_reg(addr, reg_addr, tmp) };
            p_data[0] = tmp;
            return ret;
        }
        case 2: {
            uint16_t tmp;
            const auto ret { read_reg(addr, reg_addr, tmp) };
            p_data[0] = static_cast<uint8_t>(tmp >> 8);
            p_data[1] = static_cast<uint8_t>(tmp);
            return ret;
        }
        case 3: {
            return 2; // FIXME: not implemented
        }
        case 4: {
            uint32_t tmp;
            const auto ret { read_reg(addr, reg_addr, tmp) };
            p_data[0] = static_cast<uint8_t>(tmp >> 24);
            p_data[1] = static_cast<uint8_t>(tmp >> 16);
            p_data[2] = static_cast<uint8_t>(tmp >> 8);
            p_data[3] = static_cast<uint8_t>(tmp);
            return ret;
        }
    }

    I2C_Transfer* transfer { new I2C_Transfer { addr, sizeof(reg_addr), false, length, true } };
    transfer->data1.data = reg_addr;
    transfer->data2.ptr = p_data;

    uint8_t ret {};
    if (callback) {
        transfer->callback = callback;
    } else {
        transfer->caller = ::xTaskGetCurrentTaskHandle();
        transfer->callback = [&ret](const bool done, I2C_Transfer** p_transfer) {
            ret = (*p_transfer)->error;
            if (DEBUG_) {
                (void) done;
                printf_debug(PSTR("I2C_Service::read_bytes(): callback, done=%u, err=%u\r\n"), done, ret);
            }
            ::xTaskNotifyGive((*p_transfer)->caller);
        };
    }

    if (::xQueueSend(i2c_queue_[bus_], &transfer, portMAX_DELAY) != pdTRUE) { // FIXME: timeout?
        return 100;
    }

    if (callback) {
        return 0;
    }

    ::ulTaskNotifyTake(pdTRUE, portMAX_DELAY); // FIXME: timeout?
    if (DEBUG_) {
        // printf_debug(PSTR("I2C_Service::read_bytes(): done, err=%u\r\n"), ret);
    }

    return ret;
}

template uint8_t I2C_Service::read_bytes<uint8_t>(
    const uint16_t, const uint8_t, uint8_t*, const uint8_t, std::function<void(const bool, I2C_Transfer**)>) const;
template uint8_t I2C_Service::read_bytes<uint16_t>(
    const uint16_t, const uint16_t, uint8_t*, const uint8_t, std::function<void(const bool, I2C_Transfer**)>) const;

uint8_t I2C_Service::write_reg(
    const uint16_t addr, std::unsigned_integral auto const reg, std::integral auto const data, std::function<void(const bool, I2C_Transfer**)> callback) const {
    static_assert(sizeof(reg) <= sizeof(I2C_Transfer::data1.data), "I2C_Service::write_reg<>(): invalid reg size");
    static_assert(sizeof(data) <= sizeof(I2C_Transfer::data2.data), "I2C_Service::write_reg<>(): invalid data size");

    std::atomic<bool> transfer_done;
    I2C_Transfer* transfer { new I2C_Transfer { addr, sizeof(reg), false, sizeof(data), false } };
    transfer->data1.data = reg;
    transfer->data2.data = data;

    uint8_t ret {};
    if (callback) {
        transfer->callback = callback;
    } else {
        transfer_done = false;
        transfer->callback = [&transfer_done, &ret](const bool, I2C_Transfer** p_transfer) {
            ret = (*p_transfer)->error;
            if (DEBUG_) {
                printf_debug(PSTR("I2C_Service::write_reg<%u, %u>() callback done, err=%u\r\n"), sizeof(reg), sizeof(data), ret);
            }
            transfer_done = true;
        };
    }

    if (::xQueueSend(i2c_queue_[bus_], &transfer, portMAX_DELAY) != pdTRUE) {
        // FIXME: timeout?
        if (DEBUG_) {
            printf_debug(PSTR("I2C_Service::write_reg<%u, %u>() transfer queueing FAILED\r\n"), sizeof(reg), sizeof(data));
        }
        return 100;
    }

    if (!callback) {
        while (!transfer_done) { // FIXME: timeout?
            portYIELD();
        }
        if (DEBUG_) {
            // printf_debug(PSTR("I2C_Service::write_reg<%u, %u>(): done, ret=%u\r\n"), sizeof(reg), sizeof(data), ret);
        }
    }

    return ret;
}

template uint8_t I2C_Service::write_reg<uint8_t, uint8_t>(const uint16_t, const uint8_t, const uint8_t, std::function<void(const bool, I2C_Transfer**)>) const;
template uint8_t I2C_Service::write_reg<uint16_t, uint8_t>(
    const uint16_t, const uint16_t, const uint8_t, std::function<void(const bool, I2C_Transfer**)>) const;
template uint8_t I2C_Service::write_reg<uint8_t, uint16_t>(
    const uint16_t, const uint8_t, const uint16_t, std::function<void(const bool, I2C_Transfer**)>) const;
template uint8_t I2C_Service::write_reg<uint16_t, uint16_t>(
    const uint16_t, const uint16_t, const uint16_t, std::function<void(const bool, I2C_Transfer**)>) const;
template uint8_t I2C_Service::write_reg<uint8_t, uint32_t>(
    const uint16_t, const uint8_t, const uint32_t, std::function<void(const bool, I2C_Transfer**)>) const;
template uint8_t I2C_Service::write_reg<uint16_t, uint32_t>(
    const uint16_t, const uint16_t, const uint32_t, std::function<void(const bool, I2C_Transfer**)>) const;

uint8_t I2C_Service::write_bytes(const uint16_t addr, std::unsigned_integral auto const reg_addr, const uint8_t* p_data, const uint8_t length,
    std::function<void(const bool, I2C_Transfer**)> callback) const {
    static_assert(sizeof(reg_addr) <= sizeof(I2C_Transfer::data1.data), "I2C_Service::write_bytes<>(): invalid reg_addr size");

    if (length > 32) {
        return 1;
    }

    switch (length) {
        case 0: {
            return 0;
        }
        case 1: {
            uint8_t tmp { *p_data };
            return write_reg(addr, reg_addr, tmp);
        }
        case 2: {
            uint16_t tmp { p_data[1] };
            tmp |= p_data[0] << 8;
            return write_reg(addr, reg_addr, tmp);
        }
        case 3: {
            return 2; // FIXME: not implemented
        }
        case 4: {
            uint32_t tmp { p_data[3] };
            tmp |= p_data[2] << 8;
            tmp |= p_data[1] << 16;
            tmp |= p_data[0] << 24;
            return write_reg(addr, reg_addr, tmp);
        }
    }

    I2C_Transfer* transfer { new I2C_Transfer { addr, sizeof(reg_addr), false, length, false } };
    transfer->data1.data = reg_addr;
    transfer->data2.ptr = const_cast<uint8_t*>(p_data);

    uint8_t ret {};
    if (callback) {
        transfer->callback = callback;
    } else {
        transfer->caller = ::xTaskGetCurrentTaskHandle();
        transfer->callback = [&ret](const bool done, I2C_Transfer** p_transfer) {
            ret = (*p_transfer)->error;
            if (DEBUG_) {
                (void) done;
                printf_debug(PSTR("I2C_Service::write_bytes<%u>(): callback, done=%u, err=%u\r\n"), sizeof(reg_addr), done, ret);
            }
            ::xTaskNotifyGive((*p_transfer)->caller);
        };
    }

    if (::xQueueSend(i2c_queue_[bus_], &transfer, portMAX_DELAY) != pdTRUE) {
        // FIXME: timeout?
        return 100;
    }

    if (callback) {
        return 0;
    }

    ::ulTaskNotifyTake(pdTRUE, portMAX_DELAY); // FIXME: timeout?
    if (DEBUG_) {
        // printf_debug(PSTR("I2C_Service::write_bytes<%u>(): done, ret=%u\r\n"), ret, sizeof(reg));
    }

    return ret;
}

template uint8_t I2C_Service::write_bytes<uint8_t>(
    const uint16_t, const uint8_t, const uint8_t*, const uint8_t, std::function<void(const bool, I2C_Transfer**)>) const;
template uint8_t I2C_Service::write_bytes<uint16_t>(
    const uint16_t, const uint16_t, const uint8_t*, const uint8_t, std::function<void(const bool, I2C_Transfer**)>) const;


template <typename DATA>
uint8_t I2C_Service::set_bit_internal(const uint16_t addr, std::unsigned_integral auto const reg, const uint8_t bit, const bool value) const {
    static_assert(sizeof(reg) <= sizeof(I2C_Transfer::data1.data), "I2C_Service::set_bit_internal<>(): invalid reg size");
    static_assert(sizeof(DATA) <= sizeof(I2C_Transfer::data2.data), "I2C_Service::set_bit_internal<>(): invalid DATA size");

    if (DEBUG_) {
        printf_debug(PSTR("I2C_Service::set_bit_internal<%u, %u>(0x%x, 0x%x, %u, %u)\r\n"), sizeof(reg), sizeof(DATA), addr, reg, bit, value);
    }

    DATA data;
    uint8_t ret;
    std::atomic<bool> transfer_done {};
    auto read_callback { [&data, &ret, &transfer_done, this, addr, reg, bit, value](const bool done, I2C_Transfer** p_transfer) {
        if (done) {
            data = static_cast<DATA>((*p_transfer)->data2.data);
            value ? data |= 1 << bit : data &= ~(1 << bit);
            ret = write_reg(addr, reg, data, [&transfer_done](const bool, I2C_Transfer** p_transfer) {
                if (DEBUG_) {
                    (void) p_transfer;
                    printf_debug(
                        PSTR("I2C_Service::set_bit_internal<%u, %u>() write_callback done, err=%u\r\n"), sizeof(reg), sizeof(DATA), (*p_transfer)->error);
                }
                transfer_done = true;
            });

            if (DEBUG_) {
                printf_debug(PSTR("I2C_Service::set_bit_internal<%u, %u>() read_callback done, data=0x%x\r\n"), sizeof(reg), sizeof(DATA), data);
            }
        } else {
            ret = (*p_transfer)->error;
            if (DEBUG_) {
                printf_debug(PSTR("I2C_Service::set_bit_internal<%u, %u>() read_reg<>() FAILED, err=%u\r\n"), sizeof(reg), sizeof(DATA), ret);
            }
        }
    } };

    if (read_reg(addr, reg, data, read_callback)) {
        if (DEBUG_) {
            printf_debug(PSTR("I2C_Service::set_bit_internal<%u, %u>(): read_reg<>() FAILED\r\n"), sizeof(reg), sizeof(DATA));
        }
        return 10;
    }

    while (!transfer_done) { // FIXME: timeout?
        portYIELD();
    }
    if (DEBUG_) {
        if (ret) {
            // printf_debug(PSTR("I2C_Service::set_bit_internal<%u, %u>(): ret=%u\r\n"), sizeof(REG), sizeof(DATA), ret);
        }
        printf_debug(PSTR("I2C_Service::set_bit_internal<%u, %u>(0x%x, 0x%x, %u, %u) done\r\n"), sizeof(reg), sizeof(DATA), addr, reg, bit, value);
    }

    return ret;
}

uint8_t I2C_Service::set_bit(const uint16_t addr, std::unsigned_integral auto const reg, const uint8_t bit, const bool value) const {
    static_assert(sizeof(reg) <= sizeof(I2C_Transfer::data1.data), "I2C_Service::set_bit<>(): invalid reg size");

    if (bit < 8) {
        return set_bit_internal<uint8_t>(addr, reg, bit, value);
    } else if (bit < 16) {
        return set_bit_internal<uint16_t>(addr, reg, bit, value);
    } else if (bit < 32) {
        return set_bit_internal<uint32_t>(addr, reg, bit, value);
    } else {
        return 4;
    }

    return 0;
}

template uint8_t I2C_Service::set_bit<uint8_t>(const uint16_t, const uint8_t, const uint8_t, const bool) const;
template uint8_t I2C_Service::set_bit<uint16_t>(const uint16_t, const uint16_t, const uint8_t, const bool) const;


#if 0
void I2C_Service::test(const uint16_t addr, const uint32_t rx, const uint32_t tx, std::function<void(const bool, I2C_Transfer**)> callback) const {
    I2C_Transfer* data { new I2C_Transfer { addr } };
    data->callback = callback;
    if (rx) {
        data->rx_size = 1;
        data->rx_data.data = rx;
    }
    if (tx) {
        data->tx_size = 2;
        data->tx_data.data = tx;
    }

    if (::xQueueSend(i2c_queue_[bus_], &data, 0) != pdTRUE) {
        if (callback) {
            callback(false, &data);
            if (data) {
                if (DEBUG_) {
                    printf_debug(PSTR("I2C_Service::test(): deleting data with addr=0x%x\r\n"), data->addr);
                }
                delete data;
            }
        }
    }

    if (DEBUG_) {
        printf_debug("I2C_Service::test(): data for 0x%x sent.r\n", data->addr);
    }
}
#endif
