#pragma once

#include <Arduino.h>
#include <stdarg.h>
#include <U8x8lib.h>

namespace mm {

static constexpr uint8_t PROGMEM MAX_BUF_SIZE = 32;

class OLED {
public:
    // reset_pin: U8X8_PIN_NONE if the board doesn't use one
    explicit OLED(uint8_t reset_pin = U8X8_PIN_NONE)
        : display(reset_pin) {}

    bool begin() {
        if (!display.begin()) {
            Serial.println(F("OLED: U8x8 begin failed"));
            return false;
        }

        display.setFont(u8x8_font_chroma48medium8_r);
        display.clear();
        return true;
    }

    // x, y are in character cells, not pixels
    void print(uint8_t x, uint8_t y, const char *fmt, ...) {
        char buf[MAX_BUF_SIZE];

        va_list args;
        va_start(args, fmt);
        vsnprintf(buf, MAX_BUF_SIZE, fmt, args);
        va_end(args);

        display.drawString(x, y, buf);
    }

    // x, y are in character cells, not pixels
    void print(uint8_t x, uint8_t y, const __FlashStringHelper *fmt, ...) {
        char buf[MAX_BUF_SIZE];

        va_list args;
        va_start(args, fmt);
        vsnprintf_P(buf, MAX_BUF_SIZE, (const char *)fmt, args);
        va_end(args);

        display.drawString(x, y, buf);
    }

    void clear() {
        display.clear();
    }

    // U8X8_SSD1306_128X64_NONAME_HW_I2C& raw() {
    //     return display;
    // }

private:
    U8X8_SSD1306_128X64_NONAME_HW_I2C display;
};

}