#pragma once

#include <Arduino.h>
#include <stdarg.h>
#include "Adafruit_GFX.h"
#include "Adafruit_SSD1306.h"

namespace mm {

static constexpr size_t MAX_BUF_SIZE = 64;

class OLED {
public:
    OLED(uint8_t width = 128, uint8_t height = 64, TwoWire *wire = &Wire, int8_t reset_pin = -1)
        : display(width, height, wire, reset_pin) {}

    bool begin(uint8_t i2c_addr = 0x43) {
        if (!display.begin(SSD1306_SWITCHCAPVCC, i2c_addr)) {
            Serial.println("OLED: SSD1306 allocation failed");
            return false;
        }

        display.clearDisplay();
        display.setTextColor(SSD1306_WHITE);
        display.setTextSize(1);
        display.setCursor(0, 0);
        display.display();
        return true;
    }

    void print(const char *fmt, ...) {
        char buf[MAX_BUF_SIZE];

        va_list args;
        va_start(args, fmt);
        vsnprintf(buf, MAX_BUF_SIZE, fmt, args);
        va_end(args);

        display.println(buf);
        display.display();
    }

    void clear() {
        display.clearDisplay();
        display.setCursor(0, 0);
        display.display();
    }

    void setTextSize(uint8_t size) {
        display.setTextSize(size);
    }

    Adafruit_SSD1306& raw() {
        return display;
    }

private:
    Adafruit_SSD1306 display;
};

}