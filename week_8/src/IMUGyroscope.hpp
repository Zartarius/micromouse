#pragma once

#include <Arduino.h>
#include <MPU6050_light.h>

namespace mtrn3100 {

class IMUGyroscope {
public:
    IMUGyroscope() : mpu6050{Wire} {}

    void begin(void) {
        Wire.begin();
        if (mpu6050.begin() != 0) {
            Serial.println("Could not connect to MPU6050");
        }
    }

private:
    MPU6050 mpu6050;

};

}