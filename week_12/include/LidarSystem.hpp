#pragma once

#include <Arduino.h>
#include <VL6180X.h>

namespace mm {

enum LidarIndex : uint8_t {
    LIDAR_FRONT = 0,
    LIDAR_LEFT  = 1,
    LIDAR_RIGHT = 2,
    LIDAR_COUNT = 3
};

class LidarSystem {
public:
    struct Config {
        uint8_t en_pin;
        uint8_t address;
    };

    LidarSystem(Config front, Config left, Config right) {
        config[LIDAR_FRONT] = front;
        config[LIDAR_LEFT]  = left;
        config[LIDAR_RIGHT] = right;
    }

    void initAll(void) {
        for (uint8_t i = 0; i < LIDAR_COUNT; i++) {
            pinMode(config[i].en_pin, OUTPUT);
            digitalWrite(config[i].en_pin, LOW);

            sensors[i].last_mm = -1;
            sensors[i].filtered_mm = -1.0f;
            sensors[i].has_reading = false;
            sensors[i].last_update_ms = 0;
        }

        delay(10);

        for (uint8_t i = 0; i < LIDAR_COUNT; i++) {
            initSingle(i);
            sensors[i].device.startRangeContinuous(RANGING_PERIOD_MS);
            sensors[i].last_update_ms = millis();
        }
    }

    void update(void) {
        for (uint8_t i = 0; i < LIDAR_COUNT; i++) {
            pollOne(i);
        }
    }

    int readFront() {
        return sensors[LIDAR_FRONT].last_mm;
    }

    int readLeft() {
        return sensors[LIDAR_LEFT].last_mm;
    }

    int readRight() {
        return sensors[LIDAR_RIGHT].last_mm;
    }

    int read(LidarIndex i) {
        return sensors[i].last_mm;
    }

    bool frontHasReading() const {
        return sensors[LIDAR_FRONT].has_reading;
    }

    bool leftHasReading() const {
        return sensors[LIDAR_LEFT].has_reading;
    }

    bool rightHasReading() const {
        return sensors[LIDAR_RIGHT].has_reading;
    }

    bool hasReading(LidarIndex i) const {
        return sensors[i].has_reading;
    }

    bool frontTimedOut() {
        return sensors[LIDAR_FRONT].device.timeoutOccurred();
    }

    bool leftTimedOut() {
        return sensors[LIDAR_LEFT].device.timeoutOccurred();
    }

    bool rightTimedOut() {
        return sensors[LIDAR_RIGHT].device.timeoutOccurred();
    }

    bool timedOut(LidarIndex i) {
        return sensors[i].device.timeoutOccurred();
    }

private:

    static constexpr uint8_t RANGING_PERIOD_MS = 50;
    static constexpr unsigned long STALE_TIMEOUT_MS = 250;
    // Exponential moving average weight for fresh samples: higher = more
    // responsive/less smoothing, lower = smoother/more lag. Readings only
    // actually refresh every RANGING_PERIOD_MS, so smoothing too heavily
    // (low alpha) adds real-world lag to wall-centering reactions on top
    // of that - 0.4 knocks down single-sample jitter without holding
    // onto stale values for long.
    static constexpr float SMOOTHING_ALPHA = 1.0f;

    struct Sensor {
        VL6180X device;
        int last_mm = -1;
        float filtered_mm = -1.0f;
        bool has_reading = false;
        unsigned long last_update_ms = 0;
    };

    Config config[LIDAR_COUNT];
    Sensor sensors[LIDAR_COUNT];

    void initSingle(uint8_t i) {
        digitalWrite(config[i].en_pin, HIGH);
        delay(50);

        VL6180X& lidar = sensors[i].device;

        lidar.init();
        lidar.setAddress(config[i].address);
        lidar.configureDefault();

        lidar.writeReg(
            VL6180X::SYSRANGE__MAX_CONVERGENCE_TIME,
            0x14
        );

        lidar.setTimeout(30);
    }

    void recoverSensor(uint8_t i) {
        digitalWrite(config[i].en_pin, LOW);
        delay(5);

        initSingle(i);

        sensors[i].device.startRangeContinuous(RANGING_PERIOD_MS);
        sensors[i].last_update_ms = millis();
        sensors[i].has_reading = false;
        sensors[i].last_mm = -1;
        sensors[i].filtered_mm = -1.0f;
    }

    void pollOne(uint8_t i) {
        Sensor& s = sensors[i];

        uint8_t status =
            s.device.readReg(
                VL6180X::RESULT__INTERRUPT_STATUS_GPIO
            );

        uint8_t range_status = status & 0x07;

        if (range_status == 0x04) {
            uint8_t range =
                s.device.readReg(
                    VL6180X::RESULT__RANGE_VAL
                );

            int raw_mm = static_cast<int>(range);

            // EMA smoothing. Snap straight to the first real reading
            // instead of blending from the -1 placeholder, so startup
            // doesn't ramp up from a meaningless default.
            if (!s.has_reading) {
                s.filtered_mm = static_cast<float>(raw_mm);
            } else {
                s.filtered_mm = s.filtered_mm * (1.0f - SMOOTHING_ALPHA)
                               + static_cast<float>(raw_mm) * SMOOTHING_ALPHA;
            }
            s.last_mm = static_cast<int>(s.filtered_mm + 0.5f);

            s.has_reading = true;
            s.last_update_ms = millis();

            s.device.writeReg(
                VL6180X::SYSTEM__INTERRUPT_CLEAR,
                0x07
            );

            return;
        }

        if (millis() - s.last_update_ms > STALE_TIMEOUT_MS) {
            recoverSensor(i);
        }
    }
};

}

