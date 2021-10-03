#pragma once

#include "Arduino.h"

#define FILE_READ 0
#define FILE_WRITE 1
#define FILE_WRITE_BEGIN 2

enum SeekMode { SeekSet = 0, SeekCur = 1, SeekEnd = 2 };

class File;

class FileImpl {
protected:
    virtual ~FileImpl() {}
    virtual size_t read(void* buf, size_t nbyte) = 0;
    virtual size_t write(const void* buf, size_t size) = 0;
    virtual int available() = 0;
    virtual int peek() = 0;
    virtual void flush() = 0;
    virtual bool truncate(uint64_t size = 0) = 0;
    virtual bool seek(uint64_t pos, int mode) = 0;
    virtual uint64_t position() = 0;
    virtual uint64_t size() = 0;
    virtual void close() = 0;
    virtual bool isOpen() = 0;
    virtual const char* name() = 0;
    virtual bool isDirectory() = 0;
    virtual File openNextFile(uint8_t mode = 0) = 0;
    virtual void rewindDirectory(void) = 0;
    virtual bool getCreateTime(DateTimeFields&) {
        return false;
    }
    virtual bool getModifyTime(DateTimeFields&) {
        return false;
    }
    virtual bool setCreateTime(const DateTimeFields&) {
        return false;
    }
    virtual bool setModifyTime(const DateTimeFields&) {
        return false;
    }

private:
    friend class File;
    unsigned int refcount = 0; // number of File instances referencing this FileImpl
};

class File final : public Stream {
public:
    // Empty constructor, used when a program creates a File variable
    // but does not immediately assign or initialize it.
    constexpr File() : f(nullptr) {}

    // Explicit FileImpl constructor.  Used by libraries which provide
    // access to files stored on media.  Normally this is used within
    // functions derived from FS::open() and FileImpl::openNextFile().
    // Not normally called used from ordinary programs or libraries
    // which only access files.
    File(FileImpl* file) {
        f = file;
        if (f)
            f->refcount++;
        // Serial.printf("File ctor %x, refcount=%d\n", (int)f, get_refcount());
    }

    // Copy constructor.  Typically used when a File is passed by value
    // into a function.  The File instance within the called function is
    // a copy of the original.  Also used when a File instance is created
    // and assigned a value (eg, "File f =
    File(const File& file) {
        f = file.f;
        if (f)
            f->refcount++;
        // Serial.printf("File copy ctor %x, refcount=%d\n", (int)f, get_refcount());
    }
#ifdef FILE_USE_MOVE
    // Move constructor.
    File(const File&& file) {
        f = file.f;
        if (f)
            f->refcount++;
        // Serial.printf("File copy ctor %x, refcount=%d\n", (int)f, get_refcount());
    }
#endif
    // Copy assignment.
    File& operator=(const File& file) {
        // Serial.println("File copy assignment");
        if (file.f)
            file.f->refcount++;
        if (f) {
            dec_refcount(); /*Serial.println("File copy assignment autoclose");*/
        }
        f = file.f;
        return *this;
    }
#ifdef FILE_USE_MOVE
    // Move assignment.
    File& operator=(const File&& file) {
        // Serial.println("File move assignment");
        if (file.f)
            file.f->refcount++;
        if (f) {
            dec_refcount(); /*Serial.println("File move assignment autoclose");*/
        }
        f = file.f;
        return *this;
    }
#endif
    virtual ~File() {
        // Serial.printf("File dtor %x, refcount=%d\n", (int)f, get_refcount());
        if (f)
            dec_refcount();
    }
    size_t read(void* buf, size_t nbyte) {
        return (f) ? f->read(buf, nbyte) : 0;
    }
    size_t write(const void* buf, size_t size) {
        return (f) ? f->write(buf, size) : 0;
    }
    int available() {
        return (f) ? f->available() : 0;
    }
    int peek() {
        return (f) ? f->peek() : -1;
    }
    void flush() {
        if (f)
            f->flush();
    }
    bool truncate(uint64_t size = 0) {
        return (f) ? f->truncate(size) : false;
    }
    bool seek(uint64_t pos, int mode) {
        return (f) ? f->seek(pos, mode) : false;
    }
    uint64_t position() {
        return (f) ? f->position() : 0;
    }
    uint64_t size() {
        return (f) ? f->size() : 0;
    }
    void close() {
        if (f) {
            f->close();
            dec_refcount();
        }
    }
    operator bool() {
        return (f) ? f->isOpen() : false;
    }
    const char* name() {
        return (f) ? f->name() : "";
    }
    bool isDirectory() {
        return (f) ? f->isDirectory() : false;
    }
    File openNextFile(uint8_t mode = 0) {
        return (f) ? f->openNextFile(mode) : *this;
    }
    void rewindDirectory(void) {
        if (f)
            f->rewindDirectory();
    }
    bool getCreateTime(DateTimeFields& tm) {
        return (f) ? f->getCreateTime(tm) : false;
    }
    bool getModifyTime(DateTimeFields& tm) {
        return (f) ? f->getModifyTime(tm) : false;
    }
    bool setCreateTime(const DateTimeFields& tm) {
        return (f) ? f->setCreateTime(tm) : false;
    }
    bool setModifyTime(const DateTimeFields& tm) {
        return (f) ? f->setModifyTime(tm) : false;
    }
    bool seek(uint64_t pos) {
        return seek(pos, SeekSet);
    }
    int read() {
        if (!f)
            return -1;
        unsigned char b;
        if (f->read(&b, 1) < 1)
            return -1;
        return b;
    }
    size_t write(uint8_t b) {
        return (f) ? f->write(&b, 1) : 0;
    }
    size_t write(const char* str) {
        return (f) ? f->write(str, strlen(str)) : 0;
    }
    size_t readBytes(char* buffer, size_t length) {
        return read(buffer, length);
    }
    size_t write(unsigned long n) {
        return write((uint8_t) n);
    }
    size_t write(long n) {
        return write((uint8_t) n);
    }
    size_t write(unsigned int n) {
        return write((uint8_t) n);
    }
    size_t write(int n) {
        return write((uint8_t) n);
    }
    using Print::write;

private:
    void dec_refcount() {
        if (--(f->refcount) == 0) {
            f->close();
            delete f;
        }
        f = nullptr;
    }
    int get_refcount() {
        if (f == nullptr)
            return -1;
        return f->refcount;
    }
    FileImpl* f;
};


class FS {
public:
    FS() {}
    virtual File open(const char* filename, uint8_t mode = FILE_READ) = 0;
    virtual bool exists(const char* filepath) = 0;
    virtual bool mkdir(const char* filepath) = 0;
    virtual bool rename(const char* oldfilepath, const char* newfilepath) = 0;
    virtual bool remove(const char* filepath) = 0;
    virtual bool rmdir(const char* filepath) = 0;
    virtual uint64_t usedSize() = 0;
    virtual uint64_t totalSize() = 0;
};
