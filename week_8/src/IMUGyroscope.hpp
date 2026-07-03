#pragma once

#include <Arduino.h>
#include <MPU6050_light.h>

namespace mtrn3100 {

class IMUGyroscope {
public:
    IMUGyroscope() : mpu6050{Wire} {

    }

private:
    MPU6050 mpu6050;

};

}