#pragma once

#include <Arduino.h>
#include <MPU6050_light.h>
#include "Miscellaneous.hpp"

/*
    Set this to a non-zero value if you want to use a moving average
    filter to calculate IMU readings.
*/
#define USE_MOVING_AVERAGE_FILTER 0

namespace mtrn3100 {

#if USE_MOVING_AVERAGE_FILTER
/*
    Size of the window used in moving average filter calculations. Keep within
    the range 10-25.
*/
static const uint8_t WINDOW_SIZE = 10;
#endif

class IMUGyroscope {
public:
    IMUGyroscope() : mpu6050{Wire} {}

    /*
    Initialises the MPU, blocks indefinitely if unable to connect to
    the MPU. Call this method at the beginning of main.cpp:setup.
    */
    void begin(void) {
        // Wire.begin();
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

// private:
    MPU6050 mpu6050;
    #if USE_MOVING_AVERAGE_FILTER
    mtrn3100::RingBuffer<float, WINDOW_SIZE> ring_buffer;
    #endif
};

}