#pragma once

#include "arduino_fixed.h"


static constexpr uint8_t O_READ { 0X01 };
static constexpr uint8_t O_RDONLY { O_READ };
static constexpr uint8_t O_WRITE { 0X02 };
static constexpr uint8_t O_WRONLY { O_WRITE };
static constexpr uint8_t O_RDWR { O_READ | O_WRITE };
static constexpr uint8_t O_ACCMODE { O_READ | O_WRITE };
static constexpr uint8_t O_APPEND { 0X04 };
static constexpr uint8_t O_SYNC { 0X08 };
static constexpr uint8_t O_CREAT { 0X10 };
static constexpr uint8_t O_EXCL { 0X20 };
static constexpr uint8_t O_TRUNC { 0X40 };


// FIXME: to be implemented
class SdFile : public arduino::Print {};


// FIXME: to be implemented
class File : public arduino::Stream {
public:
    File() = default;
    File(SdFile, const char*) {}
    File(const File&) {}
    ~File() = default;
    virtual size_t write(uint8_t) {
        return 0;
    }
    virtual size_t write(const uint8_t*, size_t) {
        return 0;
    }
    virtual int read() {
        return -1;
    }
    virtual int peek() {
        return -1;
    }
    virtual int available() {
        return 0;
    }
    virtual void flush() {}
    int read(void*, uint16_t) {
        return -1;
    }
    bool seek(uint32_t) {
        return false;
    }
    uint32_t position() {
        return 0;
    }
    uint32_t size() {
        return 0;
    }
    void close() {}
    operator bool() {
        return false;
    }
    const char* name() {
        return "";
    }

    bool isDirectory() {
        return false;
    }
    File openNextFile(uint8_t = O_RDONLY) {
        return *this;
    }
    void rewindDirectory() {}

    using arduino::Print::write;
};


// FIXME: to be implemented
class SDClass {
public:
    bool begin(uint8_t = 0) {
        return false;
    }

    File open(const char*, uint8_t = O_READ) {
        return File {};
    }

    bool exists(const char*) {
        return false;
    }

    bool mkdir(const char*) {
        return false;
    }

    bool remove(const char*) {
        return false;
    }

    bool rmdir(const char*) {
        return false;
    }
};

extern SDClass SD;
