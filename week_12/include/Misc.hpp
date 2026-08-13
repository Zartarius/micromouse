#pragma once

#include "Robot.hpp"
#include "PIDController.hpp"

#define MILLISECONDS_TO_MICROSECONDS(X) ((unsigned long)(X) * 1000UL)
#define HALT() do {} while (true)

namespace mm {

static constexpr float CELL_SIZE_MM = 180.0f;

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

template <typename T, size_t len>
struct Stack {
    T data[len];
    uint16_t top = 0;

    void push(T value) {
        if (top >= len) return;
        data[top] = value;
        top++;
    }

    T pop() {
        top--;
        return data[top];
    }

    T peek() const {
        return data[top - 1];
    }
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


// Raw I2C bus scan (bypasses the VL6180X library entirely) - lists every
// address that ACKs a transmission. Expect to see 0x54/0x55/0x56 (front/
// left/right) after LidarSystem::initAll() has run; if 0x29 (the VL6180X
// default address) still shows up, that sensor's setAddress() never took
// and it's still sitting at default; if an expected address is simply
// missing, nothing is answering there at all (wiring/power/dead sensor).
void i2c_scan(void) {
    auto& robot = GET_ROBOT();

    uint8_t row = 0;
    uint8_t col = 0;
    uint8_t found = 0;

    for (uint8_t addr = 1; addr < 127; addr++) {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0) {
            robot.oled.print(col, row, "%02X", addr);
            col += 3;
            if (col >= 15) {
                col = 0;
                row++;
            }
            found++;
        }
    }

    robot.oled.print(0, row + 1, "found:%d", found);
}


// Raw 16-bit-register read, bypassing the VL6180X library. Returns 0xFF
// (with nothing available) if the device NACKs/doesn't respond.
uint8_t i2c_read_reg16(uint8_t i2c_addr, uint16_t reg) {
    Wire.beginTransmission(i2c_addr);
    Wire.write((uint8_t)((reg >> 8) & 0xFF));
    Wire.write((uint8_t)(reg & 0xFF));
    Wire.endTransmission(false); // repeated start, keep the bus held
    Wire.requestFrom((int)i2c_addr, 1);
    if (Wire.available()) {
        return Wire.read();
    }
    return 0xFF;
}


// TEMP DEBUG: probe a VL6180X directly at the register level (no library).
// IDENTIFICATION__MODEL_ID (0x0000) should read 0xB4 on a healthy,
// communicating sensor. RESULT__INTERRUPT_STATUS_GPIO (0x004F) is printed
// raw/unmasked - 0x00 means "no threshold event ever fired" (ranging never
// actually started), a nonzero-but-not-x04 value is a genuine range error
// code, and 0xFF on either read means the device isn't responding to real
// register transactions at all despite ACKing a bare address ping.
void i2c_probe(uint8_t i2c_addr) {
    auto& robot = GET_ROBOT();

    uint8_t model_id = i2c_read_reg16(i2c_addr, 0x0000);
    uint8_t status = i2c_read_reg16(i2c_addr, 0x004F);

    robot.oled.print(0, 0, "addr:%02X", i2c_addr);
    robot.oled.print(0, 1, "model:%02X", model_id);
    robot.oled.print(0, 2, "status:%02X", status);
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


void lidar_tester(void) {
    auto& robot = GET_ROBOT();

    unsigned long s = millis();
    while (true) {
        robot.lidar_system.update();

        if (millis() - s >= 1000) {
            robot.oled.clear();
            robot.oled.print(0, 0, "left: %d", robot.lidar_system.readLeft());
            robot.oled.print(0, 1, "front: %d", robot.lidar_system.readFront());
            robot.oled.print(0, 2, "right: %d", robot.lidar_system.readRight());

            s = millis();
        }
    }
}


}

