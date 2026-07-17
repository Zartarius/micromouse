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
mm::PIDController rotation_controller(7.0, 0.1, 0.5);
mm::LidarSystem collectiveLidars;
mm::OLED oled;

/*
mm::PIDController position_controller(mm::PIDMode::POSITION, left_motor,
                                            left_motor_encoder, right_motor, right_motor_encoder, gyroscope,
                                            55.0, 0.3, 0.15);
mm::PIDController rotation_controller(mm::PIDMode::ROTATION, left_motor,
                                            left_motor_encoder, right_motor, right_motor_encoder, gyroscope,
                                            60.0, 5, 0); // changed these up a bit, was originally 30.0, 2.0, 0

// for position, original values were 30.0, 2.0, 0
// for rotation original values were 15.0, 3.2, 0
*/

/*
mm::PIDController lidar_controller(mm::PIDMode::POSITION, left_motor,
                                            left_motor_encoder, right_motor, right_motor_encoder, gyroscope,
                                            0.1, 0, 0);
*/

// mm::Lidar lidar1(LIDARFRONT_EN_PIN, 0x54);
// //lidar1 is front

// mm::Lidar lidar2(LIDARLEFT_EN_PIN, 0x55);
// //lidar2 is left

// mm::Lidar lidar3(LIDARRIGHT_EN_PIN, 0x56);
// //lidar3 is right



bool initOkay = false;

void setup() {
    Serial.begin(9600);
    Serial.println("Began Serial!");
    Wire.begin();
    Serial.println("Began Wire!");
    delay(500); // Give time for gyroscope to be still
    gyroscope.begin();
    Serial.println("Began Gyroscope!");
    oled.begin();
    Serial.println("Began OLED!");

    collectiveLidars.initAll();
    // single lidar class
    // Serial.println("Scanning...");
    // initOkay = lidar1.init();
    // if (!initOkay) {Serial.println("DID NOT CONNECT");};
    // initOkay = lidar2.init();
    // initOkay = lidar3.init();

    // digitalWrite(LIDARLEFT_EN_PIN, LOW);
    // digitalWrite(LIDARFRONT_EN_PIN, LOW);


}

// mm::RotationController imu_controller{gyroscope, left_motor, right_motor, 7.0, 0.1, 0.5};

void printLidar() {
    int distance = collectiveLidars.readFront();
    Serial.println(distance);
    delay(200);
}

/*
void lidarWallTask() {
    delay(1000);
    int dist = collectiveLidars.readFront() - 100;
    static long unsigned long prev = 0;

    while (true) {
        unsigned long now = micros();
        if (now - prev >= MILLISECONDS_TO_MICROSECONDS(10)) {
            prev = now;
            lidar_controller.updateLidarWall(0.01f, dist);
        }
        dist = collectiveLidars.readFront() - 100;
    }
}
*/

void loop() {

    // lidarWallTask();




    // if (initOkay) {
    //     int distance = lidar3.read();
    //     if (lidar3.timedOut()) {
    //         Serial.println("TIMEOUT");
    //     } else {
    //         Serial.println(distance);
    //     }
    // } else {
    //     Serial.println("NA");
    // }
    // delay(100);

}