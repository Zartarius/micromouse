#pragma once

#include <Arduino.h>
#include <VL6180X.h>
#include <Wire.h>


namespace mm {

class Lidar {
private:
    VL6180X lidar;
    int enablePin;
    uint8_t address;

public:
    Lidar(int enablePin, uint8_t address) : enablePin{enablePin}, address{address} {}

    bool init() {
        Serial.println("GOT HERE!!!!!!");
        pinMode(enablePin, OUTPUT);
        digitalWrite(enablePin, LOW);
        delay(50);
        digitalWrite(enablePin, HIGH);
        delay(50);

        Serial.println("enabled sensor");
        Serial.println("calling lidar.init()");

        Serial.println("2. About to call lidar.init()");
        lidar.init();
        Serial.println("3. Returned from lidar.init()");
        lidar.setAddress(address);

        uint8_t id = lidar.readReg(VL6180X::IDENTIFICATION__MODEL_ID);
        Serial.print("Model ID = 0x");
        Serial.println(id, HEX);

        if (id != 0xB4) {
            Serial.println("Wrong model ID");
            return false;
        }


        // lidar.init();
        // if (lidar.readReg(VL6180X::IDENTIFICATION__MODEL_ID) != 0xB4) {
        //     return false;
        // }

        lidar.configureDefault();
        lidar.setTimeout(250);

        // if (!lidar.init()) {
        //     return false;
        // }
        return true;
    }

    int read() {
        return lidar.readRangeSingleMillimeters();
    }

    bool timedOut() {
        return lidar.timeoutOccurred();
    }
};

}