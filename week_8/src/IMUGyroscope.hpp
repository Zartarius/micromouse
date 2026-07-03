#pragma once

#include <Arduino.h>
#include <MPU6050_light.h>

namespace mtrn3100 {

class IMUGyroscope {
public:
    IMUGyroscope() : mpu6050{Wire} {}

    /*
    Initialises the MPU, blocks indefinitely if unable to connect to
    the MPU. Call this method at the beginning of main.cpp:setup.
    */
    void begin(void) {
        Wire.begin();
        if (mpu6050.begin() != 0) {
            Serial.println("Could not connect to MPU6050");
            while (true);
        }
        mpu6050.calcOffsets();
    }

    /*
    Resets the IMU's gyroscope calibration, use whenever changing targets.
    */
    void reset(void) {
        mpu6050.setGyroOffsets(0.0f, 0.0f, 0.0f);
    }

    /*
    Returns heading of the IMU, yet to determine units etc.
    */
    float getHeading() {
        return mpu6050.getAngleZ();
    }

private:
    MPU6050 mpu6050;
};

}