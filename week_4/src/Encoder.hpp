#pragma once

#include <Arduino.h>

namespace mtrn3100 {

enum class EncoderSide {
    LEFT, RIGHT
};

class Encoder {
public:
    /*
    Constructor for the Encoder class
        - enc1: An interrupt pin on the Arduino board, connected to channel A of the motor encoder
        by convention. The ISR is called at every rising edge on this pin.
        - enc2: A digital (input) pin on the arduino board, connected to the other channel of the
        motor encoder (usually channel B). Used to derive the direction of rotation.
        - side: The encoder which this instance is for. Two Encoder instances should be created (one left and one right)
        if you want the left and right wheels to be controlled separately.
    */
    Encoder(uint8_t enc1, uint8_t enc2, EncoderSide side) : encoder1_pin{enc1}, encoder2_pin{enc2} {
        pinMode(encoder1_pin, INPUT_PULLUP);
        pinMode(encoder2_pin, INPUT_PULLUP);

        if (side == EncoderSide::LEFT) {
            instance_left = this;
            attachInterrupt(digitalPinToInterrupt(encoder1_pin), readEncoderLeftISR, RISING);
        } else if (side == EncoderSide::RIGHT) {
            instance_right = this;
            attachInterrupt(digitalPinToInterrupt(encoder1_pin), readEncoderRightISR, RISING);
        }
    }


    /*
    Called by the ISR to increment/decrement the encoder_count
    */
    void readEncoder() {
        noInterrupts();

        if (digitalRead(encoder2_pin) == HIGH) {
            encoder_count++;
            direction = 1;
        } else {
            encoder_count--;
            direction = -1;
        }

        if (curr_pulses == max_pulses_until_next_measure) {
            unsigned long curr_time = millis();
            time_between_pulses = curr_time - prev_time;
            prev_time = curr_time;
            curr_pulses = 1;
        } else {
            curr_pulses++;
        }

        interrupts();
    }


    /*
    Sets the encoder_count of the Encoder instance to 0. Useful when you want to reset
    the encoder for different stages of movement.
    */
    void setEncoderToZero() {
        // We need to turn off interrupts when modifying volatile variables
        // to prevent data races
        noInterrupts();
        encoder_count = 0;
        interrupts();
    }


    /*
    Returns the signed angular displacement of the motor (in radians) since it was last set to 0.
    */
    float getRotation() {
        noInterrupts();
        long count = encoder_count;
        interrupts();
        return (2.0f * PI * static_cast<float>(count)) / static_cast<float>(counts_per_revolution);
    }


    /*
    Returns the signed angular speed of the motor (in radians/sec).
    */
    float getSpeed() {
        noInterrupts();
        unsigned long delta_time_msec = time_between_pulses;
        unsigned long last_pulse_time = prev_time;
        int8_t dir = direction;
        interrupts();

        // Motor hasn't completed its first block yet
        if (delta_time_msec == 0) {
            return 0.0f;
        }

        // If no pulse has arrived for longer than 10 expected block durations,
        // the motor has slowed below measurable speed or stopped entirely.
        // 10 is an arbitrary number, should probably change it.
        if (millis() - last_pulse_time > delta_time_msec * 10) {
            return 0.0f;
        }

        float delta_time_sec = static_cast<float>(delta_time_msec) / 1000.0f;
        float block_ticks = static_cast<float>(max_pulses_until_next_measure);
        float delta_radians = (2.0f * PI * block_ticks) / static_cast<float>(counts_per_revolution);

        return (delta_radians / delta_time_sec) * static_cast<float>(dir);
    }


private:
    /*
    The ISR for the left encoder.
    */
    static void readEncoderLeftISR() {
        // Ensure the instance has actually been created
        if (instance_left != nullptr) {
            instance_left->readEncoder();
        }
    }


    /*
    The ISR for the right encoder.
    */
    static void readEncoderRightISR() {
        // Ensure the instance has actually been created
        if (instance_right != nullptr) {
            instance_right->readEncoder();
        }
    }

    // The pins connected to channel A and B of the encoder
    const uint8_t encoder1_pin;
    const uint8_t encoder2_pin;
    // The current direction of rotation, either 1 or -1
    volatile int8_t direction = 0;
    // Signed number of 'ticks' counted by the encoder. E.g. A CW tick and a CCW tick
    // will cancel out, making the count 0.
    volatile long encoder_count = 0;
    // Number of pulses to complete before measuring time
    const uint8_t max_pulses_until_next_measure = 20;
    // Current number of pulses counter; 1 <= curr_pulses <= max_pulses_until_next_measure
    volatile uint8_t curr_pulses = 1;
    volatile unsigned long prev_time = 0;
    volatile unsigned long time_between_pulses = 0;
    const uint16_t counts_per_revolution = 1400;

    static Encoder* instance_left;
    static Encoder* instance_right;
};

Encoder* Encoder::instance_left = nullptr;
Encoder* Encoder::instance_right = nullptr;

}