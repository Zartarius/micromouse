#pragma once

#include <Arduino.h>
#include <MPU6050_light.h>

namespace mm {

class Gyroscope {
public:
    Gyroscope(void) = default;

    // Blocks indefinitely if the MPU6050 can't be reached. Call once in setup().
    void begin(void);

    // Resets the IMU's onboard heading calibration.
    void reset(void);

    // Re-anchors "current heading" as zero without touching the hardware.
    // Cheap enough to call every control-loop tick, unlike reset().
    void rebase(void);

    // Polls the IMU if enough time has passed since the last update.
    void update(void);

    // Heading in degrees. CW is negative, CCW is positive.
    float getHeading(void);

private:
    MPU6050 mpu6050{Wire};
    unsigned long last_update_ms = 0;
    float zero_offset = 0.0f;
};

}
