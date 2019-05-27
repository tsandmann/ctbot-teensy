/*
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
 * @file    SD.cpp
 * @brief   Wrapper aroung Arduino SD library to execute in a POSIX environment
 * @author  Timo Sandmann
 * @date    17.02.2019
 */

#include "SD.h"

#include <cstdio>


File::File(std::fstream file, const char* name) : file_ { std::move(file) } {
    std::strncpy(name_, name, sizeof(name_));
}

File::~File() {
    if (file_.is_open()) {
        file_.close();
    }
}

size_t File::write(const uint8_t data) {
    file_.put(data);
    return 1;
}

size_t File::write(const uint8_t* buf, size_t size) {
    file_.write(reinterpret_cast<const char*>(buf), size);
    return size;
}

int File::read() {
    return file_.get();
}

int File::read(void* buf, uint16_t nbyte) {
    auto ptr { reinterpret_cast<char*>(buf) };
    file_.read(ptr, nbyte);

    return nbyte;
}

int File::peek() {
    return file_.peek();
}

int File::available() {
    const auto pos { file_.tellg() };
    auto fsize { file_.tellg() };
    file_.seekg(0, std::ios::end);
    fsize = file_.tellg() - fsize;
    file_.seekg(pos, std::ios::beg);

    return static_cast<int>(fsize);
}

void File::flush() {
    file_.flush();
}

bool File::seek(uint32_t pos) {
    file_.seekg(pos);
    file_.seekp(pos);
    return true;
}

uint32_t File::size() {
    const auto pos { file_.tellg() };

    file_.seekg(0, std::ios::beg);
    auto fsize { file_.tellg() };

    file_.seekg(0, std::ios::end);
    fsize = file_.tellg() - fsize;

    file_.seekg(pos, std::ios::beg);

    return static_cast<uint32_t>(fsize);
}

void File::close() {
    file_.close();
}

File::operator bool() {
    return file_.operator bool();
}

File SDClass::open(const char* name, uint8_t mode) {
    std::fstream f;
    f.open(name, std::ios::binary | (mode & O_WRITE ? std::ios::out : std::ios::in));
    return File { std::move(f), name };
}

bool SDClass::exists(const char* name) {
    std::ifstream f(name);
    return f.good();
}

bool SDClass::remove(const char* name) {
    return std::remove(name) == 0;
}

SDClass SD;
