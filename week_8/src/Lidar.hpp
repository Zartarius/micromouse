#pragma once

#include <Arduino.h>
#include <VL6180X.h>
#include <Wire.h>

class Lidar {
private:
    VL6180X sensor;
    int enablePin;
    uint8_t address;

public:
    LidarSensor(int enablePin, uint8_t address) {
        this->enablePin = enablePin;
        this->address = address;
    }

    void init() {
        pinMode(enablePin, OUTPUT);
        digitalWrite(enablePin, LOW);
        delay(50);
        digitalWrite(enablePin, HIGH);
        delay(50);
        sensor.init();
        sensor.configureDefault();
        sensor.setTimeout(250);
        sensor.setAddress(address);
    }

    int read() {
        return sensor.readRangeSingleMillimeters();
    }

    bool timedOut() {
        return sensor.timeoutOccurred();
    }
};