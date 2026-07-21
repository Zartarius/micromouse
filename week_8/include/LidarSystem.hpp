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
        // Hold all three sensors in standby first, so bringing them up
        // one at a time below doesn't collide on the default I2C address.
        pinMode(front_en_pin, OUTPUT);
        pinMode(left_en_pin, OUTPUT);
        pinMode(right_en_pin, OUTPUT);
        digitalWrite(front_en_pin, LOW);
        digitalWrite(left_en_pin, LOW);
        digitalWrite(right_en_pin, LOW);
        delay(50);

        initForSingle(lidarFront, front_en_pin, front_addr);
        initForSingle(lidarLeft, left_en_pin, left_addr);
        initForSingle(lidarRight, right_en_pin, right_addr);
    }

    void initForSingle(VL6180X& lidar, int enablePin, uint8_t address) {
        // Bring only this sensor out of standby. The others are either
        // still LOW (not yet initialised) or already re-addressed and
        // left HIGH by an earlier call — either way we don't touch them.
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

    // All three sensors stay powered and individually addressed after
    // initAll(), so reads no longer need to toggle enable pins.
    int readFront() {
        return lidarFront.readRangeSingleMillimeters();
    }

    int readLeft() {
        return lidarLeft.readRangeSingleMillimeters();
    }

    int readRight() {
        return lidarRight.readRangeSingleMillimeters();
    }

    bool timedOut(VL6180X& lidar) {
        return lidar.timeoutOccurred();
    }

    bool frontTimedOut() { return lidarFront.timeoutOccurred(); }
    bool leftTimedOut()  { return lidarLeft.timeoutOccurred(); }
    bool rightTimedOut() { return lidarRight.timeoutOccurred(); }

private:
    uint8_t front_en_pin, front_addr;
    uint8_t left_en_pin, left_addr;
    uint8_t right_en_pin, right_addr;

    VL6180X lidarFront;
    VL6180X lidarLeft;
    VL6180X lidarRight;
};

}