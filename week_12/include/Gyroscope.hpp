#pragma once

#include <Arduino.h>
#include <MPU6050_light.h>

namespace mm {

class Gyroscope {
public:
    Gyroscope(void) = default;

    /*
    Initialises the MPU, blocks indefinitely if unable to connect to
    the MPU. Call this method at the beginning of main.cpp:setup.
    */
    void begin(void) {
        if (mpu6050.begin() != 0) {
            Serial.println("Couldn't connect to MPU6050");
            do {} while (true);
        }
        Wire.beginTransmission(0x68);
        Wire.write(0x1B); // gyro config register
        Wire.write(0x08); // Sets to range 1 (±500 deg/s)
        Wire.endTransmission();

        mpu6050.calcOffsets();
    }

    /*
    Resets the IMU's gyroscope calibration, use whenever changing targets.
    */
    void reset(void) {
        mpu6050.resetAngles();
    }

    /*
    Updates the IMU if enough time has passed since the last real update.
    */
    void update(void) {
        unsigned long now_ms = millis();
        if (now_ms == last_update_ms) {
            return; // not enough time has passed for a meaningful dt
        }
        last_update_ms = now_ms;
        mpu6050.update();
    }

    /*
    Returns heading of the IMU, in degrees. CW is negative, CCW is positive.
    */
    float getHeading() {
        return mpu6050.getAngleZ();
    }

private:
    MPU6050 mpu6050{Wire};
    unsigned long last_update_ms = 0;
};

}