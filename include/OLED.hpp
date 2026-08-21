#pragma once

#include <Arduino.h>
#include <U8x8lib.h>

namespace mm {

class OLED {
public:
    // reset_pin: U8X8_PIN_NONE if the board doesn't use one
    explicit OLED(uint8_t reset_pin = U8X8_PIN_NONE);

    bool begin(void);

    // x, y are in character cells, not pixels
    void print(uint8_t x, uint8_t y, const char *fmt, ...);

    // Same as print(), but in a large 18px bold font (2x3 character cells
    // per glyph). Leave room: a char printed at (x, y) also occupies
    // (x+1, y) and (x, y+1)..(x+1, y+2).
    void printLarge(uint8_t x, uint8_t y, const char *fmt, ...);

    void clear(void);

    // x, y are in 8x8-pixel tile coordinates. tile_ptr must point to cnt*8
    // bytes: one byte per pixel column, bit0 = topmost pixel.
    void drawTile(uint8_t x, uint8_t y, uint8_t cnt, const uint8_t *tile_ptr);

private:
    U8X8_SSD1306_128X64_NONAME_HW_I2C display;
};

}
