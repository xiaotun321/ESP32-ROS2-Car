#pragma once
namespace config {
constexpr const char *FIRMWARE_VERSION = "pwm-r3-wifi-jazzy";
// Schematic U8/channel A = left, U9/channel B = right (installation convention).
constexpr int PWM[2] = {14, 17}, IN1[2] = {15, 18}, IN2[2] = {16, 21};
constexpr int STBY = 47;
constexpr int ENC_A[2] = {6, 12}, ENC_B[2] = {7, 13};
// Right motor reversed: hardware test reported d=forward, w=right turn.
constexpr int MOTOR_SIGN[2] = {1, -1};
// Encoder polarity is independent; retain until measured on the actual wheels.
constexpr int ENCODER_SIGN[2] = {1, 1};
// 13 A-channel periods per motor shaft turn, decoded on all four edges.
// User specification: reduction 1:20; 260 periods = 1040 quadrature counts/output turn.
constexpr float GEAR_RATIO = 20.0f;
constexpr float COUNTS_PER_REV = 13.0f * 4.0f * GEAR_RATIO;
constexpr float WHEEL_DIAMETER_M = 0.048f;
// Rated values supplied by user; telemetry only, not a PWM speed model.
constexpr float MOTOR_RATED_VOLTAGE = 7.4f, MOTOR_RATED_RPM = 400.0f;
constexpr unsigned CONTROL_MS = 20, COMMAND_TIMEOUT_MS = 500;
constexpr unsigned PWM_HZ = 20000;
constexpr int MAX_PWM = 180; // Commissioning cap, out of 255.
constexpr int JOG_PWM = 80;
// Open-loop command gains, NOT measured velocity tracking. Matches Pi defaults.
constexpr float LINEAR_PWM_GAIN = 80.0f / 0.15f;
constexpr float ANGULAR_PWM_GAIN = 80.0f / 0.8f;
}
