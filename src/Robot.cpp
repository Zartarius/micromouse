#include "Robot.hpp"

namespace mm {

Robot& Robot::get(void) {
    static Robot instance;
    return instance;
}

Robot::Robot(void) :
    left_motor(MotorSide::LEFT, LEFT_MOTOR_PWM_PIN, LEFT_MOTOR_DIR_PIN, LEFT_MOTOR_ENC_A_PIN, LEFT_MOTOR_ENC_B_PIN),
    right_motor(MotorSide::RIGHT, RIGHT_MOTOR_PWM_PIN, RIGHT_MOTOR_DIR_PIN, RIGHT_MOTOR_ENC_A_PIN, RIGHT_MOTOR_ENC_B_PIN),
    gyroscope(),
    rotation_controller(6.7f, 0.6f, 0.45f),
    position_controller(35.0f, 0.0f, 5.0f, 5.0f),
    heading_controller(15.0f, 0.0f, 0.5f),
    wall_centering_controller(0.20f, 0.0f, 0.05f),
    lidar_system((LidarSystem::Config) {.en_pin = LIDARFRONT_EN_PIN, .address = LIDARFRONT_ADD},
                (LidarSystem::Config) {.en_pin = LIDARLEFT_EN_PIN, .address = LIDARLEFT_ADD},
                (LidarSystem::Config) {.en_pin = LIDARRIGHT_EN_PIN, .address = LIDARRIGHT_ADD}),
    oled(),
    // Bumped from the geometric 15.85mm to correct a small systematic
    // overshoot (true wheel radius > coded radius). Recalibrate by driving
    // a measured distance at low speed: new = old * (actual / commanded).
    wheel_radius_mm(16.0f),
    track_width_mm(87.4f)
{}

}
