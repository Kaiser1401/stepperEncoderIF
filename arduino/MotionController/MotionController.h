#pragma once

#include <Arduino.h>

// ============================================
// CONFIG STRUCTS
// ============================================

struct MotorConfig {

    uint8_t dirPin;

    uint8_t stepPin;

    uint8_t enablePin;

    uint16_t pulseWidthUs;
};

struct EncoderConfig {

    uint8_t pinA;

    uint8_t pinB;
};

struct HardwareConfig {

    const MotorConfig* motors;

    int motorCount;

    const EncoderConfig* encoders;

    int encoderCount;
};

// ============================================
// MOTOR MODES
// ============================================

enum MotorMode {

    IDLE,
    MOVE,
    HOME
};

// ============================================
// MOTOR RUNTIME STATE
// ============================================

struct StepperMotorState {

    bool enabled = false;

    bool estop = false;

    bool direction = false;

    MotorMode mode = IDLE;

    long stepsRemaining = 0;

    uint32_t stepIntervalUs = 1000;

    uint32_t lastStepUs = 0;

    // homing

    int encoderIndex = 0;

    long stallStartEncoder = 0;

    uint32_t stallCheckStartMs = 0;

    uint32_t stallWindowMs = 300;

    long stallThreshold = 3;
};

// ============================================
// CONTROLLER
// ============================================

class MotionController {

public:

    MotionController(
        const HardwareConfig& config
    );

    void begin(
        uint32_t baudrate
    );

    void update();

private:

    // singleton ISR access

    static MotionController* instance;

    static void sharedEncoderISR();

    void handleAllEncoderInterrupts();

    // config

    const HardwareConfig& hw;

    // runtime

    StepperMotorState* motors;

    volatile long* encoderCounts;

    volatile uint8_t* lastEncoderAStates;

    // serial buffer

    static constexpr int SERIAL_BUFFER_SIZE = 96;

    char serialBuffer[
        SERIAL_BUFFER_SIZE
    ];

    int serialIndex = 0;

    // scheduler

    uint32_t lastLoopUs = 0;

    static constexpr uint32_t LOOP_HZ = 1000;

    static constexpr uint32_t LOOP_US =
        1000000UL / LOOP_HZ;

    // internal

    void processSerial();

    void processCommand(
        char* cmd
    );

    void updateMotors();

    void updateHoming(
        int motor
    );

    void sendEncoderData();

    void emergencyStop();

    void pulseStepPin(
        int motor
    );

    void attachEncoderInterrupts();

    bool validMotor(
        int index
    );

    bool validEncoder(
        int index
    );

    long atomicReadEncoder(
        int index
    );

    void atomicWriteEncoder(
        int index,
        long value
    );
};