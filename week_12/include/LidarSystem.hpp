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

    LidarSystem(Config front, Config left, Config right) {
        config[LIDAR_FRONT] = front;
        config[LIDAR_LEFT]  = left;
        config[LIDAR_RIGHT] = right;
    }

    // Returns the number of sensors that came up successfully.
    // Call Wire.begin() before this.
    uint8_t initAll(void) {
        for (uint8_t i = 0; i < LIDAR_COUNT; i++) {
            pinMode(config[i].en_pin, OUTPUT);
            digitalWrite(config[i].en_pin, LOW);

            sensors[i].last_mm = -1;
            sensors[i].filtered_mm = -1.0f;
            sensors[i].has_reading = false;
            sensors[i].online = false;
            sensors[i].last_update_ms = 0;
            sensors[i].next_bringup_ms = 0;
        }

        // Hold every sensor in reset long enough that they all definitely
        // drop back to the default address and come up fresh-out-of-reset.
        // A soft MCU reset does not power-cycle the sensors, so this is the
        // only thing that clears addresses assigned by a previous run.
        delay(50);

        uint8_t ok = 0;
        for (uint8_t i = 0; i < LIDAR_COUNT; i++) {
            if (bringUp(i, BRINGUP_ATTEMPTS)) ok++;
        }
        return ok;
    }

    void update(void) {
        for (uint8_t i = 0; i < LIDAR_COUNT; i++) {
            pollOne(i);
        }
    }

    // 0.25f is the default for both. Front and sides generally want
    // different smoothing: the front reading gates a hard stop (readFront()
    // <= threshold), so it usually wants to be more responsive (higher
    // alpha), while the sides feed wall-centering, which is more sensitive
    // to noise-driven jitter and usually wants more smoothing (lower alpha).
    void updateSideSmoothingAlpha(float new_alpha = 0.25f) {
        side_smoothing_alpha = new_alpha;
    }

    void updateFrontSmoothingAlpha(float new_alpha = 0.25f) {
        front_smoothing_alpha = new_alpha;
    }

    int readFront() { return sensors[LIDAR_FRONT].last_mm; }
    int readLeft()  { return sensors[LIDAR_LEFT].last_mm; }
    int readRight() { return sensors[LIDAR_RIGHT].last_mm; }
    int read(LidarIndex i) { return sensors[i].last_mm; }

    bool frontHasReading() const { return sensors[LIDAR_FRONT].has_reading; }
    bool leftHasReading()  const { return sensors[LIDAR_LEFT].has_reading; }
    bool rightHasReading() const { return sensors[LIDAR_RIGHT].has_reading; }
    bool hasReading(LidarIndex i) const { return sensors[i].has_reading; }

    // True once the sensor has been addressed and verified on the bus.
    bool isOnline(LidarIndex i) const { return sensors[i].online; }

    bool frontTimedOut() { return sensors[LIDAR_FRONT].device.timeoutOccurred(); }
    bool leftTimedOut()  { return sensors[LIDAR_LEFT].device.timeoutOccurred(); }
    bool rightTimedOut() { return sensors[LIDAR_RIGHT].device.timeoutOccurred(); }
    bool timedOut(LidarIndex i) { return sensors[i].device.timeoutOccurred(); }

private:

    static constexpr uint8_t  RANGING_PERIOD_MS  = 50;
    static constexpr unsigned long STALE_TIMEOUT_MS = 250;

    // Address every VL6180X wakes up at after a reset.
    static constexpr uint8_t DEFAULT_ADDR = 0x29;
    // IDENTIFICATION__MODEL_ID always reads back as this on a live chip.
    static constexpr uint8_t MODEL_ID     = 0xB4;

    static constexpr uint8_t BRINGUP_ATTEMPTS = 3;
    // Runtime recovery only gets one attempt per pass so update() never
    // stalls the control loop for hundreds of ms; this is how long to wait
    // before trying a dead sensor again.
    static constexpr unsigned long RECOVER_BACKOFF_MS = 500;

    // Exponential moving average weight for fresh samples: higher = more
    // responsive/less smoothing, lower = smoother/more lag. Readings only
    // actually refresh every RANGING_PERIOD_MS, so smoothing too heavily
    // (low alpha) adds real-world lag to wall-centering/front-stop
    // reactions on top of that. Split per sensor group since the front
    // and the sides feed very different consumers with different noise
    // tolerances - see updateFrontSmoothingAlpha/updateSideSmoothingAlpha.
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

    // Reset one sensor, re-address it, and verify every step. Safe to call
    // both at startup and mid-run. On failure the sensor is left held in
    // reset so it cannot sit on the default address and collide with the
    // others - that collision is what turns one bad sensor into three.
    bool bringUp(uint8_t i, uint8_t attempts) {
        Sensor& s = sensors[i];
        VL6180X& lidar = s.device;

        s.online = false;
        s.has_reading = false;
        s.last_mm = -1;
        s.filtered_mm = -1.0f;

        for (uint8_t attempt = 0; attempt < attempts; attempt++) {
            digitalWrite(config[i].en_pin, LOW);
            delay(20);

            // The chip is now back at DEFAULT_ADDR, but the driver object
            // still has whatever address we last assigned it cached. Point
            // it back at the default. The register write goes out to the
            // stale address and NACKs - nothing is listening there - which
            // is harmless; what we want is the object-side side effect.
            lidar.setAddress(DEFAULT_ADDR);
            lidar.setTimeout(30);

            digitalWrite(config[i].en_pin, HIGH);
            delay(20);   // datasheet only needs ~400us, be generous

            // Is anything actually there before we start configuring it?
            if (lidar.readReg(VL6180X::IDENTIFICATION__MODEL_ID) != MODEL_ID) {
                continue;
            }

            lidar.init();
            lidar.configureDefault();
            lidar.writeReg(VL6180X::SYSRANGE__MAX_CONVERGENCE_TIME, 0x14);

            if (lidar.timeoutOccurred()) continue;

            lidar.setAddress(config[i].address);
            delay(2);

            // Confirm it answers at the NEW address before moving on to the
            // next sensor. Without this a silently failed reassignment
            // leaves this chip on DEFAULT_ADDR, and the next sensor boots
            // onto the same address.
            if (lidar.readReg(VL6180X::IDENTIFICATION__MODEL_ID) != MODEL_ID) {
                continue;
            }

            lidar.startRangeContinuous(RANGING_PERIOD_MS);
            if (lidar.timeoutOccurred()) continue;

            s.last_update_ms = millis();
            s.online = true;
            return true;
        }

        digitalWrite(config[i].en_pin, LOW);
        return false;
    }

    void pollOne(uint8_t i) {
        Sensor& s = sensors[i];

        if (!s.online) {
            if ((long)(millis() - s.next_bringup_ms) >= 0) {
                if (!bringUp(i, 1)) {
                    s.next_bringup_ms = millis() + RECOVER_BACKOFF_MS;
                }
            }
            return;
        }

        uint8_t status =
            s.device.readReg(VL6180X::RESULT__INTERRUPT_STATUS_GPIO);

        if (!s.device.timeoutOccurred() && (status & 0x07) == 0x04) {
            uint8_t range = s.device.readReg(VL6180X::RESULT__RANGE_VAL);
            int raw_mm = static_cast<int>(range);

            // EMA smoothing. Snap straight to the first real reading
            // instead of blending from the -1 placeholder, so startup
            // doesn't ramp up from a meaningless default.
            float alpha = (i == LIDAR_FRONT) ? front_smoothing_alpha : side_smoothing_alpha;
            if (!s.has_reading) {
                s.filtered_mm = static_cast<float>(raw_mm);
            } else {
                s.filtered_mm = s.filtered_mm * (1.0f - alpha)
                               + static_cast<float>(raw_mm) * alpha;
            }
            s.last_mm = static_cast<int>(s.filtered_mm + 0.5f);

            s.has_reading = true;
            s.last_update_ms = millis();

            s.device.writeReg(VL6180X::SYSTEM__INTERRUPT_CLEAR, 0x07);
            return;
        }

        if (millis() - s.last_update_ms > STALE_TIMEOUT_MS) {
            s.online = false;
            s.has_reading = false;
            s.last_mm = -1;
            s.next_bringup_ms = millis();   // one immediate retry, then back off
        }
    }
};

}