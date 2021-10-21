/*
 * This file is part of the c't-Bot teensy framework.
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
 * @file    fr_stream_buffer.h
 * @brief   FreeRTOS Stream buffer class
 * @author  Timo Sandmann
 * @date    15.10.2021
 */

#pragma once

#include "arduino_freertos.h"
#include "stream_buffer.h"


template <typename T, size_t SIZE>
class StreamBuffer {
    bool user_mem_;
    StreamBufferHandle_t buf_;
    StaticStreamBuffer_t static_buf_;

public:
    explicit StreamBuffer(void* p_buffer) : user_mem_ { p_buffer ? true : false } {
        if (p_buffer) {
            auto ptr { reinterpret_cast<uint8_t*>(p_buffer) };
            buf_ = ::xStreamBufferCreateStatic(sizeof(T) * SIZE, sizeof(T), ptr, &static_buf_);
        } else {
            buf_ = ::xStreamBufferCreate(sizeof(T) * SIZE, sizeof(T));
        }
    }

    explicit StreamBuffer() : StreamBuffer { nullptr } {}

    ~StreamBuffer() {
        if (!user_mem_) {
            ::vStreamBufferDelete(buf_);
        }
    }

    void push(const T& item) {
        ::xStreamBufferSend(buf_, &item, sizeof(T), portMAX_DELAY);
    }

    void push(T&& item) {
        ::xStreamBufferSend(buf_, &item, sizeof(T), portMAX_DELAY);
    }

    bool try_push(const T& item) {
        if (::xStreamBufferSpacesAvailable(buf_) < sizeof(T)) {
            return false;
        }

        push(item);

        return true;
    }

    bool try_push(T&& item) {
        if (::xStreamBufferSpacesAvailable(buf_) < sizeof(T)) {
            return false;
        }

        push(std::forward(item));

        return true;
    }

    void pop(T& item) {
        ::xStreamBufferReceive(buf_, &item, sizeof(T), portMAX_DELAY);
    }

    bool try_pop(T& item) {
        if (::xStreamBufferBytesAvailable(buf_) < sizeof(T)) {
            return false;
        }

        pop(item);

        return true;
    }

    bool clear() {
        return ::xStreamBufferReset(buf_) == pdTRUE;
    }

    bool empty() {
        return ::xStreamBufferIsEmpty(buf_) == pdTRUE;
    }

    bool full() const {
        return ::xStreamBufferIsFull(buf_) == pdTRUE;
    }

    size_t capacity() const {
        return SIZE;
    }

    size_t size() {
        return ::xStreamBufferBytesAvailable(buf_);
    }
};
