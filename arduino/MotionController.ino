#include "MotionController.h"

// ============================================
// DEVICE COUNTS
// ============================================

constexpr int MOTOR_COUNT = 2;

constexpr int ENCODER_COUNT = 2;

// ============================================
// MOTOR CONFIGURATION
// ============================================

MotorConfig motors[MOTOR_COUNT] = {

    {
        .dirPin = 2,
        .stepPin = 3,
        .enablePin = 4,
        .pulseWidthUs = 2
    },

    {
        .dirPin = 5,
        .stepPin = 6,
        .enablePin = 7,
        .pulseWidthUs = 5
    }
};

// ============================================
// ENCODER CONFIGURATION
// ============================================

EncoderConfig encoders[ENCODER_COUNT] = {

    {
        .pinA = 8,
        .pinB = 9
    },

    {
        .pinA = 10,
        .pinB = 11
    }
};

// ============================================
// HARDWARE CONFIG
// ============================================

HardwareConfig hw = {

    .motors = motors,

    .motorCount = MOTOR_COUNT,

    .encoders = encoders,

    .encoderCount = ENCODER_COUNT
};

// ============================================
// CONTROLLER
// ============================================

MotionController controller(hw);

// ============================================
// SETUP
// ============================================

void setup() {

    controller.begin(115200);
}

// ============================================
// LOOP
// ============================================

void loop() {

    controller.update();
}