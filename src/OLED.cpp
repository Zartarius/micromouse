#include "OLED.hpp"

#include <stdarg.h>

namespace mm {

static constexpr uint8_t MAX_BUF_SIZE = 32;

OLED::OLED(uint8_t reset_pin) : display(reset_pin) {}

bool OLED::begin(void) {
    if (!display.begin()) {
        Serial.println(F("OLED: U8x8 begin failed"));
        return false;
    }

    display.setFont(u8x8_font_chroma48medium8_r);
    display.clear();
    return true;
}

void OLED::print(uint8_t x, uint8_t y, const char *fmt, ...) {
    char buf[MAX_BUF_SIZE];

    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, MAX_BUF_SIZE, fmt, args);
    va_end(args);

    display.drawString(x, y, buf);
}

void OLED::printLarge(uint8_t x, uint8_t y, const char *fmt, ...) {
    char buf[MAX_BUF_SIZE];

    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, MAX_BUF_SIZE, fmt, args);
    va_end(args);

    display.setFont(u8x8_font_courB18_2x3_r);
    display.drawString(x, y, buf);
    display.setFont(u8x8_font_chroma48medium8_r);
}

void OLED::clear(void) {
    display.clear();
}

void OLED::drawTile(uint8_t x, uint8_t y, uint8_t cnt, const uint8_t *tile_ptr) {
    display.drawTile(x, y, cnt, const_cast<uint8_t *>(tile_ptr));
}

}
