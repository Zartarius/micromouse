#pragma once

#include <Arduino.h>
#include <VL6180X.h>

namespace mm {

class LidarSystem {
public:
    LidarSystem(uint8_t front_en_pin, uint8_t front_addr,
                uint8_t left_en_pin, uint8_t left_addr,
                uint8_t right_en_pin, uint8_t right_addr)
        : front_en_pin{front_en_pin}, front_addr{front_addr},
          left_en_pin{left_en_pin}, left_addr{left_addr},
          right_en_pin{right_en_pin}, right_addr{right_addr} {
    }

    void initAll(void) {
        initForSingle(lidarFront, front_en_pin, front_addr);
        initForSingle(lidarLeft, left_en_pin, left_addr);
        initForSingle(lidarRight, right_en_pin, right_addr);
    }

    void initForSingle(VL6180X& lidar, int enablePin, uint8_t address) {
        pinMode(enablePin, OUTPUT);
        digitalWrite(enablePin, LOW);
        delay(50);
        digitalWrite(enablePin, HIGH);
        delay(50);

        lidar.init();
        lidar.setAddress(address);

        // uint8_t id = lidar.readReg(VL6180X::IDENTIFICATION__MODEL_ID);
        // Serial.print("Model ID = 0x");
        // Serial.println(id, HEX);

        // if (id != 0xB4) {
        //     Serial.println("Wrong model ID");
        //     return false;
        // }

        lidar.configureDefault();
        lidar.setTimeout(250);
    }

    int readFront() {
        digitalWrite(left_en_pin, LOW);
        digitalWrite(right_en_pin, LOW);
        digitalWrite(front_en_pin, HIGH);
        return lidarFront.readRangeSingleMillimeters();
    }

    int readLeft() {
        digitalWrite(left_en_pin, HIGH);
        digitalWrite(right_en_pin, LOW);
        digitalWrite(front_en_pin, LOW);
        return lidarLeft.readRangeSingleMillimeters();
    }

    int readRight() {
        digitalWrite(left_en_pin, LOW);
        digitalWrite(right_en_pin, HIGH);
        digitalWrite(front_en_pin, LOW);
        return lidarRight.readRangeSingleMillimeters();
    }

    bool timedOut(VL6180X& lidar) {
        return lidar.timeoutOccurred();
    }

private:
    uint8_t front_en_pin, front_addr;
    uint8_t left_en_pin, left_addr;
    uint8_t right_en_pin, right_addr;

    VL6180X lidarFront;
    VL6180X lidarLeft;
    VL6180X lidarRight;
};

}