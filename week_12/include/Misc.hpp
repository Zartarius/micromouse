#pragma once

#include "Robot.hpp"
#include "PIDController.hpp"

#define MILLISECONDS_TO_MICROSECONDS(X) ((unsigned long)(X) * 1000UL)
#define HALT() do {} while (true)

namespace mm {

/*
    Same functionality as delay(), but it regularly updates the gyroscope too.
    Use this instead of delay() everywhere once the gyroscope is initialised.
*/
void delayWhileUpdating(unsigned long duration_ms) {
    auto& robot = GET_ROBOT();

    unsigned long start = millis();
    while (millis() - start < duration_ms) {
        robot.gyroscope.update();
    }
}

/*
    Ringbuffer data structure, for use in moving average filters and other stuff.
*/
template <typename T, size_t len>
struct RingBuffer {
public:
    bool push(const T& item) {
        if (isFull()) return false;
        buffer[buffer_head] = item;
        buffer_head = (buffer_head + 1) % len;
        count++;
        return true;
    }

    bool pop(T& item) {
        if (isEmpty()) return false;
        item = buffer[buffer_tail];
        buffer_tail = (buffer_tail + 1) % len;
        count--;
        return true;
    }

    bool peek(T& item) const {
        if (isEmpty()) return false;
        item = buffer[buffer_tail];
        return true;
    }

    bool isEmpty() const { return count == 0; }
    bool isFull() const { return count == len; }
    size_t size() const { return count; }

    void clear() {
        buffer_head = 0;
        buffer_tail = 0;
        count = 0;
    }

    T buffer[len];
    size_t buffer_head = 0;
    size_t buffer_tail = 0;
    size_t count = 0;
};

// Returns true if str[:n] == ref[:]
bool str_eq(const char *str, const char *ref, int n) {
    for (int i = 0; i < n; i++) {
        if (str[i] != ref[i]) {
            return false;
        }
    }

    return true;
}


}