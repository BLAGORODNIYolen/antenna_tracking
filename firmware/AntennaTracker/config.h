#pragma once
#include <Arduino.h>
// MUST be measured on your mechanism before enabling. No motion by default.
#ifndef ANTENNA_MECHANICS_CONFIGURED
#define ANTENNA_MECHANICS_CONFIGURED 0
#endif
constexpr bool MECHANICS_CONFIGURED = ANTENNA_MECHANICS_CONFIGURED;
constexpr float AZ_MIN_MECH = -90.0f;  // EXAMPLE ONLY
constexpr float AZ_MAX_MECH = 90.0f;   // EXAMPLE ONLY
constexpr float AZ_NORTH_OFFSET = 0.0f;
constexpr float STEPS_PER_REV = 4075.7728f; // half-steps, verify actual gearbox
constexpr float EXTERNAL_RATIO = 1.0f;
constexpr bool AZ_REVERSE = false;
constexpr uint16_t AZ_MAX_SPEED = 200; // half-steps/s; conservative starting value
constexpr uint16_t AZ_ACCELERATION = 100;
constexpr uint8_t IN1=22, IN2=23, IN3=24, IN4=25;
constexpr uint8_t SERVO_PIN=9;
constexpr float EL_MIN=0.0f, EL_MAX=90.0f;
constexpr float EL_INITIAL=0.0f; // position operator must establish before calibration
constexpr int EL_PULSE_AT_MIN=1000, EL_PULSE_AT_MAX=2000; // EXAMPLES, measure!
constexpr float EL_RATE_DEG_S=15.0f;
constexpr uint32_t STALE_MS=250;
constexpr uint16_t MAX_PACKET=384;
static_assert(AZ_MIN_MECH < AZ_MAX_MECH && AZ_MAX_MECH-AZ_MIN_MECH <= 720, "Invalid AZ limits");
static_assert(EL_MIN < EL_MAX && EL_MIN >= -90 && EL_MAX <= 90, "Invalid EL limits");
static_assert(EL_INITIAL >= EL_MIN && EL_INITIAL <= EL_MAX, "Invalid initial EL");
