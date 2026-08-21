#pragma once

#include "Motor.hpp"
#include "Gyroscope.hpp"
#include "PIDController.hpp"
#include "LidarSystem.hpp"
#include "OLED.hpp"

#define GET_ROBOT() (mm::Robot::get())

namespace mm {

class Robot {
public:
    static Robot& get(void);

    Motor left_motor;
    Motor right_motor;
    Gyroscope gyroscope;
    PIDController rotation_controller;
    PIDController position_controller;
    PIDController heading_controller;
    PIDController wall_centering_controller;
    LidarSystem lidar_system;
    OLED oled;
    const float wheel_radius_mm;
    const float track_width_mm;

private:
    inline static constexpr uint8_t LEFT_MOTOR_PWM_PIN = 11;
    inline static constexpr uint8_t LEFT_MOTOR_DIR_PIN = 12;
    inline static constexpr uint8_t RIGHT_MOTOR_PWM_PIN = 9;
    inline static constexpr uint8_t RIGHT_MOTOR_DIR_PIN = 10;
    inline static constexpr uint8_t LEFT_MOTOR_ENC_A_PIN = 2;
    inline static constexpr uint8_t LEFT_MOTOR_ENC_B_PIN = 7;
    inline static constexpr uint8_t RIGHT_MOTOR_ENC_A_PIN = 3;
    inline static constexpr uint8_t RIGHT_MOTOR_ENC_B_PIN = 8;

    inline static constexpr uint8_t LIDARRIGHT_EN_PIN = A1;
    inline static constexpr uint8_t LIDARFRONT_EN_PIN = A2;
    inline static constexpr uint8_t LIDARLEFT_EN_PIN = A0;
    inline static constexpr uint8_t LIDARFRONT_ADD = 0x54;
    inline static constexpr uint8_t LIDARLEFT_ADD = 0x55;
    inline static constexpr uint8_t LIDARRIGHT_ADD = 0x56;

    Robot(void);
    Robot(const Robot&) = delete;
    Robot& operator=(const Robot&) = delete;
};

}
