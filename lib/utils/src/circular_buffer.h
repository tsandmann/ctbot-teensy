/*
 * This file is part of the c't-Bot teensy framework.
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
 * @note    based on https://github.com/embeddedartistry/embedded-resources/blob/master/examples/cpp/circular_buffer.cpp
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
    bool full_;
    std::array<T, SIZE> buf_;
    std::mutex mutex_;
    std::condition_variable cond_in_;
    std::condition_variable cond_out_;

public:
    explicit CircularBuffer() : head_ {}, tail_ {}, full_ { false } {}

    void push(const T& item) {
        std::unique_lock<std::mutex> mlock(mutex_);

        while (full_) {
            cond_out_.wait(mlock);
        }

        buf_[head_] = item;

        if (full_) {
            tail_ = (tail_ + 1) % SIZE;
        }

        head_ = (head_ + 1) % SIZE;
        full_ = head_ == tail_;

        mlock.unlock();
        cond_in_.notify_one();
    }

    void push(T&& item) {
        std::unique_lock<std::mutex> mlock(mutex_);

        while (full_) {
            cond_out_.wait(mlock);
        }

        buf_[head_] = std::move(item);

        if (full_) {
            tail_ = (tail_ + 1) % SIZE;
        }

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

        buf_[head_] = item;

        if (full_) {
            tail_ = (tail_ + 1) % SIZE;
        }

        head_ = (head_ + 1) % SIZE;
        full_ = head_ == tail_;

        mlock.unlock();
        cond_in_.notify_one();
        return true;
    }

    bool try_push(T&& item) {
        std::unique_lock<std::mutex> mlock(mutex_);
        if (full_) {
            return false;
        }

        buf_[head_] = std::move(item);

        if (full_) {
            tail_ = (tail_ + 1) % SIZE;
        }

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

        // Read data and advance the tail (we now have a free space)
        item = buf_[tail_];
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

        // Read data and advance the tail (we now have a free space)
        item = std::move(buf_[tail_]);
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
        // If tail is ahead the head by 1, we are full
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
