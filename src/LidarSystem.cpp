#include "LidarSystem.hpp"

namespace mm {

LidarSystem::LidarSystem(Config front, Config left, Config right) {
    config[LIDAR_FRONT] = front;
    config[LIDAR_LEFT]  = left;
    config[LIDAR_RIGHT] = right;
}

uint8_t LidarSystem::initAll(void) {
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

    // Hold every sensor in reset long enough that they all drop back to the
    // default address - a soft MCU reset doesn't power-cycle the sensors,
    // so this is what clears addresses assigned by a previous run.
    delay(50);

    uint8_t ok = 0;
    for (uint8_t i = 0; i < LIDAR_COUNT; i++) {
        if (bringUp(i, BRINGUP_ATTEMPTS)) ok++;
    }
    return ok;
}

void LidarSystem::update(void) {
    for (uint8_t i = 0; i < LIDAR_COUNT; i++) {
        pollOne(i);
    }
}

void LidarSystem::updateSideSmoothingAlpha(float new_alpha) {
    side_smoothing_alpha = new_alpha;
}

void LidarSystem::updateFrontSmoothingAlpha(float new_alpha) {
    front_smoothing_alpha = new_alpha;
}

int LidarSystem::readFront(void) { return sensors[LIDAR_FRONT].last_mm; }
int LidarSystem::readLeft(void)  { return sensors[LIDAR_LEFT].last_mm; }
int LidarSystem::readRight(void) { return sensors[LIDAR_RIGHT].last_mm; }
int LidarSystem::read(LidarIndex i) { return sensors[i].last_mm; }

bool LidarSystem::frontHasReading(void) const { return sensors[LIDAR_FRONT].has_reading; }
bool LidarSystem::leftHasReading(void)  const { return sensors[LIDAR_LEFT].has_reading; }
bool LidarSystem::rightHasReading(void) const { return sensors[LIDAR_RIGHT].has_reading; }
bool LidarSystem::hasReading(LidarIndex i) const { return sensors[i].has_reading; }

bool LidarSystem::isOnline(LidarIndex i) const { return sensors[i].online; }

bool LidarSystem::frontTimedOut(void) { return sensors[LIDAR_FRONT].device.timeoutOccurred(); }
bool LidarSystem::leftTimedOut(void)  { return sensors[LIDAR_LEFT].device.timeoutOccurred(); }
bool LidarSystem::rightTimedOut(void) { return sensors[LIDAR_RIGHT].device.timeoutOccurred(); }
bool LidarSystem::timedOut(LidarIndex i) { return sensors[i].device.timeoutOccurred(); }

bool LidarSystem::bringUp(uint8_t i, uint8_t attempts) {
    Sensor& s = sensors[i];
    VL6180X& lidar = s.device;

    s.online = false;
    s.has_reading = false;
    s.last_mm = -1;
    s.filtered_mm = -1.0f;

    for (uint8_t attempt = 0; attempt < attempts; attempt++) {
        digitalWrite(config[i].en_pin, LOW);
        delay(20);

        // Chip is back at DEFAULT_ADDR; point the driver object at it too.
        // The register write NACKs if nothing's listening yet - harmless,
        // we just want the object-side address reset.
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

        // Confirm it answers at the NEW address - otherwise a silently
        // failed reassignment leaves this chip on DEFAULT_ADDR and the
        // next sensor boots onto the same address.
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

void LidarSystem::pollOne(uint8_t i) {
    Sensor& s = sensors[i];

    if (!s.online) {
        if ((long)(millis() - s.next_bringup_ms) >= 0) {
            if (!bringUp(i, 1)) {
                s.next_bringup_ms = millis() + RECOVER_BACKOFF_MS;
            }
        }
        return;
    }

    uint8_t status = s.device.readReg(VL6180X::RESULT__INTERRUPT_STATUS_GPIO);

    if (!s.device.timeoutOccurred() && (status & 0x07) == 0x04) {
        uint8_t range = s.device.readReg(VL6180X::RESULT__RANGE_VAL);
        int raw_mm = static_cast<int>(range);

        // EMA smoothing. Snap straight to the first real reading instead of
        // blending from the -1 placeholder, so startup doesn't ramp up from
        // a meaningless default.
        float alpha = (i == LIDAR_FRONT) ? front_smoothing_alpha : side_smoothing_alpha;
        if (!s.has_reading) {
            s.filtered_mm = static_cast<float>(raw_mm);
        } else {
            s.filtered_mm = s.filtered_mm * (1.0f - alpha) + static_cast<float>(raw_mm) * alpha;
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

}
