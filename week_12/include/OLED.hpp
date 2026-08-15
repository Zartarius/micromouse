#pragma once

#include <Arduino.h>
#include <stdarg.h>
#include <U8x8lib.h>

namespace mm {

static constexpr uint8_t MAX_BUF_SIZE = 32;

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

    // Same as print(), but in a large 18px bold font (each character is 2x3
    // character cells, i.e. 16x24px). x, y are still in normal 8x8 character
    // cells, so leave room: a character printed at (x, y) also occupies
    // (x+1, y), (x, y+1)..(x+1, y+2). Restores the normal font afterward so
    // it doesn't affect other print() calls.
    void printLarge(uint8_t x, uint8_t y, const char *fmt, ...) {
        char buf[MAX_BUF_SIZE];

        va_list args;
        va_start(args, fmt);
        vsnprintf(buf, MAX_BUF_SIZE, fmt, args);
        va_end(args);

        display.setFont(u8x8_font_courB18_2x3_r);
        display.drawString(x, y, buf);
        display.setFont(u8x8_font_chroma48medium8_r);
    }


    // x, y are in character cells, not pixels
    // void print(uint8_t x, uint8_t y, const __FlashStringHelper *fmt, ...) {
    //     char buf[MAX_BUF_SIZE];

    //     va_list args;
    //     va_start(args, fmt);
    //     vsnprintf_P(buf, MAX_BUF_SIZE, (const char *)fmt, args);
    //     va_end(args);

    //     display.drawString(x, y, buf);
    // }

    void clear() {
        display.clear();
    }

    // x, y are in 8x8-pixel tile coordinates (0-15, 0-7 on a 128x64 screen).
    // tile_ptr must point to cnt*8 bytes: one byte per pixel column (8
    // columns per tile, tiles laid out left-to-right), bit0 = topmost pixel.
    void drawTile(uint8_t x, uint8_t y, uint8_t cnt, const uint8_t *tile_ptr) {
        display.drawTile(x, y, cnt, const_cast<uint8_t *>(tile_ptr));
    }

    // U8X8_SSD1306_128X64_NONAME_HW_I2C& raw() {
    //     return display;
    // }

private:
    U8X8_SSD1306_128X64_NONAME_HW_I2C display;
};

}