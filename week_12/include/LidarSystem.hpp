#pragma once

#include <Arduino.h>
#include <VL6180X.h>

namespace mm {

class LidarSystem {
public:
    LidarSystem(uint8_t front_en_pin, uint8_t front_addr,
                uint8_t left_en_pin, uint8_t left_addr,
                uint8_t right_en_pin, uint8_t right_addr)
        : front_en_pin{front_en_pin}, front_addr{front_addr},
          left_en_pin{left_en_pin}, left_addr{left_addr},
          right_en_pin{right_en_pin}, right_addr{right_addr} {
    }

    void initAll(void) {
        // Hold all three sensors in standby first, so bringing them up
        // one at a time below doesn't collide on the default I2C address.
        pinMode(front_en_pin, OUTPUT);
        pinMode(left_en_pin, OUTPUT);
        pinMode(right_en_pin, OUTPUT);
        digitalWrite(front_en_pin, LOW);
        digitalWrite(left_en_pin, LOW);
        digitalWrite(right_en_pin, LOW);
        delay(50);

        initForSingle(lidarFront, front_en_pin, front_addr);
        initForSingle(lidarLeft, left_en_pin, left_addr);
        initForSingle(lidarRight, right_en_pin, right_addr);

        // Kick off free-running continuous ranging on all three sensors.
        // period_ms is the sensor's internal measurement interval; it will
        // keep updating its result register in the background without us
        // ever having to block waiting for a conversion.
        lidarFront.startRangeContinuous(RANGING_PERIOD_MS);
        lidarLeft.startRangeContinuous(RANGING_PERIOD_MS);
        lidarRight.startRangeContinuous(RANGING_PERIOD_MS);
    }

    void initForSingle(VL6180X& lidar, int enablePin, uint8_t address) {
        // Bring only this sensor out of standby. The others are either
        // still LOW (not yet initialised) or already re-addressed and
        // left HIGH by an earlier call — either way we don't touch them.
        digitalWrite(enablePin, HIGH);
        delay(50);

        lidar.init();
        lidar.setAddress(address);
        lidar.configureDefault();
        lidar.setTimeout(30); // short timeout: we never want a stall here
    }

    // Call this once per loop iteration, as early as possible. It does a
    // single non-blocking register check per sensor: if a fresh reading
    // is ready, it's latched into the cache; if not, the cache keeps its
    // last value. This never blocks on I2C waiting for a conversion.
    void update(void) {
        pollOne(lidarFront, last_front_mm, front_new_reading);
        pollOne(lidarLeft,  last_left_mm,  left_new_reading);
        pollOne(lidarRight, last_right_mm, right_new_reading);
    }

    // Instant, non-blocking accessors — just return the last cached value.
    int readFront() { return last_front_mm; }
    int readLeft()  { return last_left_mm; }
    int readRight() { return last_right_mm; }

    // True if update() has ever latched a fresh reading for that sensor
    // (i.e. whether readX() is returning real data vs a stale default).
    bool frontHasReading() const { return front_new_reading; }
    bool leftHasReading()  const { return left_new_reading; }
    bool rightHasReading() const { return right_new_reading; }

    bool frontTimedOut() { return lidarFront.timeoutOccurred(); }
    bool leftTimedOut()  { return lidarLeft.timeoutOccurred(); }
    bool rightTimedOut() { return lidarRight.timeoutOccurred(); }

private:
    static constexpr uint8_t RANGING_PERIOD_MS = 20;

    // Checks the sensor's interrupt status register for "new sample
    // ready" without blocking. Only if a sample is already waiting do
    // we call readRangeContinuousMillimeters(), which in that case
    // returns immediately (the data is already latched in the sensor).
    void pollOne(VL6180X& lidar, int& cache, bool& has_reading) {
        uint8_t status = lidar.readReg(VL6180X::RESULT__INTERRUPT_STATUS_GPIO);
        uint8_t range_status = status & 0x07;

        if (range_status == 0x04) { // new sample ready
            cache = lidar.readRangeContinuousMillimeters();
            has_reading = true;
            lidar.writeReg(VL6180X::SYSTEM__INTERRUPT_CLEAR, 0x07);
        }
        // else: no new sample yet, keep the cached value as-is
    }

    uint8_t front_en_pin, front_addr;
    uint8_t left_en_pin, left_addr;
    uint8_t right_en_pin, right_addr;

    VL6180X lidarFront;
    VL6180X lidarLeft;
    VL6180X lidarRight;

    int last_front_mm = -1;
    int last_left_mm  = -1;
    int last_right_mm = -1;

    bool front_new_reading = false;
    bool left_new_reading  = false;
    bool right_new_reading = false;
};

}