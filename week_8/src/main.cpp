#include <Arduino.h>
#include "Motor.hpp"
#include "PIDController.hpp"
#include "Miscellaneous.hpp"
#include "Gyroscope.hpp"
#include "LidarSystem.hpp"
#include "OLED.hpp"

/*
    We assume MOT1 in "Micromouse_Kit_Information.pdf" to refer to the left motor,
    and MOT2 to refer to the right motor.
*/
static constexpr uint8_t PROGMEM LEFT_MOTOR_PWM_PIN = 11;
static constexpr uint8_t PROGMEM LEFT_MOTOR_DIR_PIN = 12;
static constexpr uint8_t PROGMEM RIGHT_MOTOR_PWM_PIN = 9;
static constexpr uint8_t PROGMEM RIGHT_MOTOR_DIR_PIN = 10;
static constexpr uint8_t PROGMEM LEFT_MOTOR_ENC_A_PIN = 2;
static constexpr uint8_t PROGMEM LEFT_MOTOR_ENC_B_PIN = 7;
static constexpr uint8_t PROGMEM RIGHT_MOTOR_ENC_A_PIN = 3;
static constexpr uint8_t PROGMEM RIGHT_MOTOR_ENC_B_PIN = 8;

mm::Motor left_motor(mm::MotorSide::LEFT, LEFT_MOTOR_PWM_PIN, LEFT_MOTOR_DIR_PIN, LEFT_MOTOR_ENC_A_PIN, LEFT_MOTOR_ENC_B_PIN);
mm::Motor right_motor(mm::MotorSide::RIGHT, RIGHT_MOTOR_PWM_PIN, RIGHT_MOTOR_DIR_PIN, RIGHT_MOTOR_ENC_A_PIN, RIGHT_MOTOR_ENC_B_PIN);
mm::Gyroscope gyroscope;
mm::PIDController rotation_controller(5.0, 0.25, 0.5);
mm::PIDController position_controller(40.0, 0.3, 0.15);
mm::PIDController heading_controller(10.0, 0.0, 0.5);
mm::LidarSystem collectiveLidars;
mm::OLED oled;


bool initOkay = false;

void setup() {
    Serial.begin(9600);
    Serial.println("Began Serial!");
    // Serial.println(BUILD_TIMESTAMP);
    Wire.begin();
    Serial.println("Began Wire!");
    delay(1000); // Give time for gyroscope to be still
    gyroscope.begin();
    Serial.println("Began Gyroscope!");
    oled.begin();
    Serial.println("Began OLED!");
    oled.print("Testing OLED: %d %d %d", 1, 2, 3);

    collectiveLidars.initAll();
    Serial.println("Began LIDARs!");
}


void turning() {
    // float og_heading = gyroscope.getHeading();
    //turn 90*
    rotate(rotation_controller, -90.0f);
    
    //wait an allotted time for demonstrator to turn
    // float og_heading = gyroscope.getHeading(); // gonna be around -90.0
    //OR we wat until we sense a change from heading
    delay(3000);
    // while (fabs(heading_after_90 - gyroscope.getHeading()) < 5) {
    //     delay(5000);
    // }

    //QUESTION - is it within 10 seconds or AFTER 10 seconds?
    float new_head = gyroscope.getHeading();
    Serial.println(new_head);
    if (new_head < 0) {
        // turned in direction of og 90* (CW)
        rotate(rotation_controller, new_head);

    } else {
        rotate(rotation_controller, -new_head);
    }
    Serial.println(gyroscope.getHeading());

    
}

void turning2(void) {
    gyroscope.reset();

    rotate(rotation_controller, -90.0f, false);

    unsigned long wait_start = millis();
    while (millis() - wait_start <= 5000) {
        gyroscope.update();
        delay(5);
    }
    
    rotate(rotation_controller, -90.0f, false);
}


void driving_and_stopping(void) {
    mm::PIDController lidar_controller(40.0, 0.3, 0.15);

    const float target = 100.0f; // mm from wall
    lidar_controller.setTarget(target);
    heading_controller.setTarget(0.0f);

    unsigned long prev_time = micros();

    bool iter = true;

    while (true) {
        unsigned long now = micros();
        float dt = (now - prev_time) * 1e-6f;
        prev_time = now;

        float distance_error = (float)collectiveLidars.readFront() - target; 
        int16_t forward_output = lidar_controller.compute_output(distance_error / 16.0f, dt, -80, 80);

        gyroscope.update();
        float heading_error = 0.0f - gyroscope.getHeading();
        int16_t heading_output = heading_controller.compute_output(heading_error, dt, -40, 40);

        int16_t left_output = -forward_output + heading_output;
        int16_t right_output = forward_output + heading_output;
        left_output = constrain(left_output, -80, 80);
        right_output = constrain(right_output, -80, 80);

        
        if (iter) {
            right_motor.setPWM(right_output);
            left_motor.setPWM(left_output);
        } else {
            left_motor.setPWM(left_output);
            right_motor.setPWM(right_output);
        }

        iter = !iter;
    }
}

// usage should be:
// chaining("lffrlrfr");       or any order of movements, should receive 8 movements
void chaining(char *movement) {
    while (*movement != '\0') {
        if (*movement == 'f') {
            driveStraight(position_controller, heading_controller, 180);
        } else if (*movement == 'r') {
            rotate(rotation_controller, -90);
        } else if (*movement == 'l') {
            rotate(rotation_controller, 90);
        } else {
            Serial.println("BAD INSTRUCTION!!!");
        }

        movement++;
    }
}


void loop() {
    // turning();
    // delay(5000);

    turning2();
    delay(5000);
}