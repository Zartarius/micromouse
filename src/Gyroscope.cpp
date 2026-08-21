#include "Gyroscope.hpp"

namespace mm {

void Gyroscope::begin(void) {
    if (mpu6050.begin() != 0) {
        Serial.println("Couldn't connect to MPU6050");
        do {} while (true);
    }
    Wire.beginTransmission(0x68);
    Wire.write(0x1B); // gyro config register
    Wire.write(0x08); // range 1 (+-500 deg/s)
    Wire.endTransmission();

    mpu6050.calcOffsets();
}

void Gyroscope::reset(void) {
    mpu6050.resetAngles();
    zero_offset = 0.0f;
}

void Gyroscope::rebase(void) {
    zero_offset = mpu6050.getAngleZ();
}

void Gyroscope::update(void) {
    unsigned long now_ms = millis();
    if (now_ms == last_update_ms) {
        return;
    }
    last_update_ms = now_ms;
    mpu6050.update();
}

float Gyroscope::getHeading(void) {
    return mpu6050.getAngleZ() - zero_offset;
}

}
