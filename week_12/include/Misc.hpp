#pragma once

#include "Robot.hpp"
#include "PIDController.hpp"

#define MILLISECONDS_TO_MICROSECONDS(X) ((unsigned long)(X) * 1000UL)
#define HALT() do {} while (true)

namespace mm {

static constexpr float CELL_SIZE_MM = 180.0f;

/*
    Same functionality as delay(), but it regularly updates the gyroscope and lidar system too.
    Use this instead of delay() everywhere once the gyroscope is initialised.
*/
void delayWhileUpdating(unsigned long duration_ms) {
    auto& robot = GET_ROBOT();

    unsigned long start = millis();
    while (millis() - start < duration_ms) {
        robot.gyroscope.update();
        robot.lidar_system.update();
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


// Returns true if str[:n] == ref[:]
bool str_eq(const char *str, const char *ref, int n) {
    for (int i = 0; i < n; i++) {
        if (str[i] != ref[i]) {
            return false;
        }
    }

    return true;
}


namespace {

constexpr uint8_t SCREEN_W = 128;
constexpr uint8_t SCREEN_H = 64;
constexpr uint8_t PAGES = SCREEN_H / 8;

constexpr uint8_t STRIPE_H     = 10;
constexpr uint8_t STRIPE_TOP_Y = 6;
constexpr uint8_t STRIPE_BOT_Y = SCREEN_H - 6 - STRIPE_H;  // 48

constexpr int STAR_CX = SCREEN_W / 2;
constexpr int STAR_CY = SCREEN_H / 2;
constexpr int STAR_R  = 14;
constexpr int STROKE  = 2;


inline bool in_triangle(int dx, int dy, int inset) {
    const int t = (443 * dx) / 256;

    return (STAR_R - 2 * dy >= inset) &&
           ( t + dy + STAR_R >= inset) &&
           (-t + dy + STAR_R >= inset);
}


inline bool in_star(int dx, int dy) {
    if (dx < -2 * STAR_R || dx > 2 * STAR_R) return false;
    if (dy < -STAR_R || dy > STAR_R) return false;

    const bool up_outer   = in_triangle(dx,  dy, 0);
    const bool up_inner   = in_triangle(dx,  dy, 2 * STROKE);
    const bool down_outer = in_triangle(dx, -dy, 0);
    const bool down_inner = in_triangle(dx, -dy, 2 * STROKE);

    return (up_outer && !up_inner) || (down_outer && !down_inner);
}


inline bool flag_pixel(uint8_t x, uint8_t y) {
    if (y >= STRIPE_TOP_Y && y < STRIPE_TOP_Y + STRIPE_H) return false;
    if (y >= STRIPE_BOT_Y && y < STRIPE_BOT_Y + STRIPE_H) return false;

    if (in_star(static_cast<int>(x) - STAR_CX,
                static_cast<int>(y) - STAR_CY)) {
        return false;
    }

    return true;
}

[[maybe_unused]]
void print_victory_flag(void) {
    auto& robot = GET_ROBOT();

    robot.oled.clear();

    uint8_t page_buf[SCREEN_W];

    for (uint8_t page = 0; page < PAGES; page++) {
        for (uint8_t x = 0; x < SCREEN_W; x++) {
            uint8_t bits = 0;

            // Bit n of a page byte is pixel row (page * 8 + n), LSB on top.
            for (uint8_t bit = 0; bit < 8; bit++) {
                if (flag_pixel(x, static_cast<uint8_t>(page * 8 + bit))) {
                    bits |= static_cast<uint8_t>(1u << bit);
                }
            }

            page_buf[x] = bits;
        }

        robot.oled.drawTile(0, page, SCREEN_W / 8, page_buf);
    }
}

}


void lidar_tester(unsigned long print_period = 50UL) {
    auto& robot = GET_ROBOT();

    unsigned long s = millis();
    while (true) {
        robot.lidar_system.update();

        if (millis() - s >= print_period) {
            robot.oled.clear();
            robot.oled.print(0, 0, "left: %d", robot.lidar_system.readLeft());
            robot.oled.print(0, 1, "front: %d", robot.lidar_system.readFront());
            robot.oled.print(0, 2, "right: %d", robot.lidar_system.readRight());

            s = millis();
        }
    }
}


}

