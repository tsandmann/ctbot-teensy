#pragma once

#include "SD.h"
#include <cstdint>
#include <string>
#include <type_traits>


// FIXME: to be implemented
class JsonArray {
public:
    template <typename TValue>
    bool is(const size_t) const {
        return false;
    }

    template <typename TValue>
    TValue get(const size_t) const {
        static TValue v;
        return v;
    }

    template <typename T>
    bool add(const T&) {
        return false;
    }

    template <typename T>
    bool set(size_t, const T&) {
        return false;
    }
};


// FIXME: to be implemented
class JsonObject {
public:
    bool success() const {
        return false;
    }

    bool containsKey(const std::string&) const {
        return false;
    }

    template <typename TValue>
    bool is(const std::string&) const {
        return false;
    }

    template <typename TValue>
    typename std::enable_if<!std::is_same<TValue, JsonArray>::value, TValue>::type get(const std::string&) const {
        static TValue v;
        return v;
    }

    template <typename TValue>
    typename std::enable_if<std::is_same<TValue, JsonArray>::value, JsonArray>::type& get(const std::string&) const {
        static JsonArray j;
        return j;
    }

    template <typename TValue>
    bool set(const std::string&, const TValue&) {
        return false;
    }

    JsonArray& createNestedArray(const std::string&) {
        static JsonArray j;
        return j;
    }

    size_t printTo(std::string&) {
        return 0;
    }

    size_t printTo(File&) {
        return 0;
    }
};


// FIXME: to be implemented
class DynamicJsonBuffer {
public:
    JsonObject& parseObject(File&, uint8_t = 0) {
        static JsonObject j;
        return j;
    };

    JsonObject& createObject() {
        static JsonObject j;
        return j;
    }
};
