#pragma once

#include <Arduino.h>

#define MILLISECONDS_TO_MICROSECONDS(X) ((unsigned long)(X) * 1000UL)
#define HALT() do {} while (true)

namespace mm {

static constexpr float CELL_SIZE_MM = 180.0f;

// Same as delay(), but keeps the gyroscope and lidar system updating. Use
// this instead of delay() everywhere once the gyroscope is initialised.
void delayWhileUpdating(unsigned long duration_ms);

// Returns true if str[:n] == ref[:n]
bool str_eq(const char *str, const char *ref, int n);

// Prints the three lidar readings to the OLED, refreshed every print_period ms.
void lidar_tester(unsigned long print_period = 50UL);

// Draws a victory flag graphic on the OLED.
void print_victory_flag(void);

// Ringbuffer, for moving-average filters and the like.
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

    bool isEmpty(void) const { return count == 0; }
    bool isFull(void) const { return count == len; }
    size_t size(void) const { return count; }

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

template <typename T, size_t len>
struct Stack {
public:
    void push(T value) {
        if (top >= len) return;
        data[top] = value;
        top++;
    }

    T pop() {
        top--;
        return data[top];
    }

    bool isEmpty() const { return top == 0; }
    bool isFull(void) const { return top == len; }
    size_t size(void) const { return top; }
    void clear(void) { top = 0; }

    T data[len];
    uint16_t top = 0;
};

}
