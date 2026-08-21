#pragma once

#include <Arduino.h>
#include <Wire.h>
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

    LidarSystem(Config front, Config left, Config right);

    // Returns the number of sensors that came up successfully.
    // Call Wire.begin() before this.
    uint8_t initAll(void);

    void update(void);

    // EMA smoothing weight for fresh samples (0-1): higher = more
    // responsive, lower = smoother. Front and sides are tuned separately
    // since the front gates a hard stop while the sides feed wall-centering.
    void updateSideSmoothingAlpha(float new_alpha = 0.25f);
    void updateFrontSmoothingAlpha(float new_alpha = 0.25f);

    int readFront(void);
    int readLeft(void);
    int readRight(void);
    int read(LidarIndex i);

    bool frontHasReading(void) const;
    bool leftHasReading(void) const;
    bool rightHasReading(void) const;
    bool hasReading(LidarIndex i) const;

    // True once the sensor has been addressed and verified on the bus.
    bool isOnline(LidarIndex i) const;

    bool frontTimedOut(void);
    bool leftTimedOut(void);
    bool rightTimedOut(void);
    bool timedOut(LidarIndex i);

private:
    static constexpr uint8_t RANGING_PERIOD_MS = 50;
    static constexpr unsigned long STALE_TIMEOUT_MS = 250;

    static constexpr uint8_t DEFAULT_ADDR = 0x29; // address every VL6180X wakes up at after reset
    static constexpr uint8_t MODEL_ID     = 0xB4; // expected IDENTIFICATION__MODEL_ID reading

    static constexpr uint8_t BRINGUP_ATTEMPTS = 3;
    // Runtime recovery gets one attempt per update() pass so the control
    // loop never stalls; this is the backoff before retrying a dead sensor.
    static constexpr unsigned long RECOVER_BACKOFF_MS = 500;

    float front_smoothing_alpha = 0.25f;
    float side_smoothing_alpha = 0.25f;

    struct Sensor {
        VL6180X device;
        int last_mm = -1;
        float filtered_mm = -1.0f;
        bool has_reading = false;
        bool online = false;
        unsigned long last_update_ms = 0;
        unsigned long next_bringup_ms = 0;
    };

    Config config[LIDAR_COUNT];
    Sensor sensors[LIDAR_COUNT];

    // Resets and re-addresses one sensor, verifying every step. On failure
    // the sensor is left held in reset so it can't sit on the shared
    // default address and collide with the others.
    bool bringUp(uint8_t i, uint8_t attempts);
    void pollOne(uint8_t i);
};

}
