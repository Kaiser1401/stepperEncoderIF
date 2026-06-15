#include "MotionController.h"

// ============================================
// DEVICE COUNTS
// ============================================

constexpr int MOTOR_COUNT = 3;

constexpr int ENCODER_COUNT = 3;

// ============================================
// MOTOR CONFIGURATION
// ============================================

MotorConfig motors[MOTOR_COUNT] = {

    {
        .dirPin = 8,
        .stepPin = 9,
        .enablePin = 2,
        .pulseWidthUs = 3
    },

        {
        .dirPin = 6,
        .stepPin = 7,
        .enablePin = 2,
        .pulseWidthUs = 3
    },
        {
        .dirPin = 4,
        .stepPin = 5,
        .enablePin = 2,
        .pulseWidthUs = 3
    },
};

// ============================================
// ENCODER CONFIGURATION
// ============================================

EncoderConfig encoders[ENCODER_COUNT] = {

    {
        .pinA = 15,
        .pinB = 14
    },

    {
        .pinA = 17,
        .pinB = 16
    },

    {
        .pinA = 19,
        .pinB = 18
    },    
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