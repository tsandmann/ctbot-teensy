/*
 * This file is part of the ct-Bot teensy framework.
 * Copyright (c) 2018 Timo Sandmann
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
 * @file    circular_buffer.h
 * @brief   Circular buffer class
 * @author  Timo Sandmann
 * @date    22.12.2018
 * @note    based on https://github.com/embeddedartistry/embedded-resources/blob/master/examples/cpp/circular_buffer/circular_buffer.hpp
 */

#pragma once

#include <array>
#include <mutex>
#include <condition_variable>
#include <cstddef>


template <typename T, size_t SIZE>
class CircularBuffer {
    size_t head_;
    size_t tail_;
    std::array<T, SIZE>* p_buf_;
    bool full_;
    bool mem_allocated_;
    std::mutex mutex_;
    std::condition_variable cond_in_;
    std::condition_variable cond_out_;

public:
    explicit CircularBuffer(void* p_buffer) : head_ {}, tail_ {}, p_buf_ {}, full_ {}, mem_allocated_ {} {
        if (p_buffer) {
            p_buf_ = new (p_buffer) std::array<T, SIZE>;
        } else {
            p_buf_ = new std::array<T, SIZE>;
            mem_allocated_ = true;
        }
    }

    explicit CircularBuffer() : CircularBuffer { nullptr } {}

    ~CircularBuffer() {
        if (mem_allocated_) {
            delete p_buf_;
        }
    }

    void push(const T& item) {
        std::unique_lock<std::mutex> mlock(mutex_);

        while (full_) {
            cond_out_.wait(mlock);
        }

        (*p_buf_)[head_] = item;

        head_ = (head_ + 1) % SIZE;
        full_ = head_ == tail_;

        mlock.unlock();
        cond_in_.notify_one();
    }

    void push(const T&& item) {
        std::unique_lock<std::mutex> mlock(mutex_);

        while (full_) {
            cond_out_.wait(mlock);
        }

        (*p_buf_)[head_] = item;

        head_ = (head_ + 1) % SIZE;
        full_ = head_ == tail_;

        mlock.unlock();
        cond_in_.notify_one();
    }

    bool try_push(const T& item) {
        std::unique_lock<std::mutex> mlock(mutex_);
        if (full_) {
            return false;
        }

        (*p_buf_)[head_] = item;

        head_ = (head_ + 1) % SIZE;
        full_ = head_ == tail_;

        mlock.unlock();
        cond_in_.notify_one();
        return true;
    }

    bool try_push(const T&& item) {
        std::unique_lock<std::mutex> mlock(mutex_);
        if (full_) {
            return false;
        }

        (*p_buf_)[head_] = item;

        head_ = (head_ + 1) % SIZE;
        full_ = head_ == tail_;

        mlock.unlock();
        cond_in_.notify_one();
        return true;
    }

    void pop(T& item) {
        std::unique_lock<std::mutex> mlock(mutex_);
        while (!full_ && (head_ == tail_)) {
            cond_in_.wait(mlock);
        }

        // read data and advance the tail
        item = (*p_buf_)[tail_];
        full_ = false;
        tail_ = (tail_ + 1) % SIZE;

        mlock.unlock();
        cond_out_.notify_one();
    }

    bool try_pop(T& item) {
        std::unique_lock<std::mutex> mlock(mutex_);
        if (!full_ && (head_ == tail_)) {
            return false;
        }

        // read data and advance the tail
        item = (*p_buf_)[tail_];
        full_ = false;
        tail_ = (tail_ + 1) % SIZE;

        mlock.unlock();
        cond_out_.notify_one();
        return true;
    }

    void clear() {
        std::unique_lock<std::mutex> mlock(mutex_);
        head_ = tail_;
        full_ = false;
    }

    bool empty() {
        std::unique_lock<std::mutex> mlock(mutex_);
        // if head and tail are equal, we are empty
        return !full_ && (head_ == tail_);
    }

    bool full() const {
        return full_;
    }

    size_t capacity() const {
        return SIZE;
    }

    size_t size() {
        std::unique_lock<std::mutex> mlock(mutex_);
        size_t size { SIZE };

        if (!full_) {
            if (head_ >= tail_) {
                size = head_ - tail_;
            } else {
                size = SIZE + head_ - tail_;
            }
        }

        return size;
    }
};
