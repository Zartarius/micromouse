#pragma once

#include <Arduino.h>
#include <MPU6050_light.h>
#include "Motor.hpp"
#include "Miscellaneous.hpp"

/*
    Set this to a non-zero value if you want to use a moving average
    filter to calculate IMU readings.
*/
#define USE_MOVING_AVERAGE_FILTER 0

namespace mm {

#if USE_MOVING_AVERAGE_FILTER
/*
    Size of the window used in moving average filter calculations. Keep within
    the range 5-25.
*/
static constexpr uint8_t WINDOW_SIZE = 5;
#endif

class Gyroscope {
public:
#if USE_MOVING_AVERAGE_FILTER
    Gyroscope(void) {
        for (int i = 0; i < WINDOW_SIZE; i++) {
            ring_buffer.buffer[i] = 0.0f;
        }
    }
#else
    Gyroscope(void) = default;
#endif
    /*
    Initialises the MPU, blocks indefinitely if unable to connect to
    the MPU. Call this method at the beginning of main.cpp:setup.
    */
    void begin(void) {
        if (mpu6050.begin() != 0) {
            Serial.println("Could not connect to MPU6050");
            while (true);
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
    Updates the IMU, call as often as possible, ideally >= 20 Hz.
    */
    void update(void) {
        mpu6050.update();

        #if USE_MOVING_AVERAGE_FILTER
            ring_buffer.push(getHeading());
        #endif
    }

    /*
    Returns heading of the IMU, in degrees. CW is negative, CCW is positive.
    */
    float getHeading() {
        #if USE_MOVING_AVERAGE_FILTER
            float average_heading = 0.0f;

            size_t idx = ring_buffer.buffer_head;
            while (idx != ring_buffer.buffer_tail) {
                average_heading += ring_buffer.buffer[idx];
                idx = (idx + 1) % WINDOW_SIZE;
            }

            return average_heading / static_cast<float>(WINDOW_SIZE);
        #else
            return mpu6050.getAngleZ() - bias;
        #endif
    }

private:
    MPU6050 mpu6050{Wire};
    float bias{0.0f};
    #if USE_MOVING_AVERAGE_FILTER
    mm::RingBuffer<float, WINDOW_SIZE> ring_buffer;
    #endif
};