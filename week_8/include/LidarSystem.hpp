#pragma once

#include <Arduino.h>
#include <VL6180X.h>

namespace mm {

static constexpr uint8_t PROGMEM LIDARRIGHT_EN_PIN = A1;
static constexpr uint8_t PROGMEM LIDARFRONT_EN_PIN = A2;
static constexpr uint8_t PROGMEM LIDARLEFT_EN_PIN = A0;
static constexpr uint8_t PROGMEM LIDARFRONT_ADD = 0x54;
static constexpr uint8_t PROGMEM LIDARLEFT_ADD = 0x55;
static constexpr uint8_t PROGMEM LIDARRIGHT_ADD = 0x56;

class LidarSystem {
public:
    LidarSystem() = default;

    void initAll() {
        initForSingle(lidarFront, LIDARFRONT_EN_PIN, LIDARFRONT_ADD);
        initForSingle(lidarLeft, LIDARLEFT_EN_PIN, LIDARLEFT_ADD);
        initForSingle(lidarRight, LIDARRIGHT_EN_PIN, LIDARRIGHT_ADD);
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
        digitalWrite(LIDARLEFT_EN_PIN, LOW);
        digitalWrite(LIDARRIGHT_EN_PIN, LOW);
        digitalWrite(LIDARFRONT_EN_PIN, HIGH);
        return lidarFront.readRangeSingleMillimeters();
    }

    int readLeft() {
        digitalWrite(LIDARLEFT_EN_PIN, HIGH);
        digitalWrite(LIDARRIGHT_EN_PIN, LOW);
        digitalWrite(LIDARFRONT_EN_PIN, LOW);
        return lidarLeft.readRangeSingleMillimeters();
    }

    int readRight() {
        digitalWrite(LIDARLEFT_EN_PIN, LOW);
        digitalWrite(LIDARRIGHT_EN_PIN, HIGH);
        digitalWrite(LIDARFRONT_EN_PIN, LOW);
        return lidarRight.readRangeSingleMillimeters();
    }

    bool timedOut(VL6180X& lidar) {
        return lidar.timeoutOccurred();
    }

private:
    VL6180X lidarFront;
    VL6180X lidarLeft;
    VL6180X lidarRight;
    // int enablePin;
    // uint8_t address;
};

}