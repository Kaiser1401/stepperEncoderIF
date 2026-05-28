#include "MotionController.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// ============================================
// STATIC INSTANCE
// ============================================

MotionController*
MotionController::instance = nullptr;

// ============================================
// CONSTRUCTOR
// ============================================

MotionController::MotionController(
    const HardwareConfig& config
)
    : hw(config)
{

    motors =
        new StepperMotorState[
            hw.motorCount
        ];

    encoderCounts =
        new volatile long[
            hw.encoderCount
        ];

    lastEncoderAStates =
        new volatile uint8_t[
            hw.encoderCount
        ];

    instance = this;
}

// ============================================
// HELPERS
// ============================================

bool MotionController::validMotor(
    int index
) {

    return (
        index >= 0
        &&
        index < hw.motorCount
    );
}

bool MotionController::validEncoder(
    int index
) {

    return (
        index >= 0
        &&
        index < hw.encoderCount
    );
}

long MotionController::atomicReadEncoder(
    int index
) {

    noInterrupts();

    long value =
        encoderCounts[index];

    interrupts();

    return value;
}

void MotionController::atomicWriteEncoder(
    int index,
    long value
) {

    noInterrupts();

    encoderCounts[index] =
        value;

    interrupts();
}

// ============================================
// BEGIN
// ============================================

void MotionController::begin(
    uint32_t baudrate
) {

    Serial.begin(baudrate);

    // motors

    for (
        int i = 0;
        i < hw.motorCount;
        i++
    ) {

        auto& m = hw.motors[i];

        pinMode(m.dirPin, OUTPUT);

        pinMode(m.stepPin, OUTPUT);

        pinMode(m.enablePin, OUTPUT);

        digitalWrite(
            m.enablePin,
            HIGH
        );
    }

    // encoders

    for (
        int i = 0;
        i < hw.encoderCount;
        i++
    ) {

        auto& e = hw.encoders[i];

        pinMode(e.pinA, INPUT_PULLUP);

        pinMode(e.pinB, INPUT_PULLUP);

        encoderCounts[i] = 0;

        lastEncoderAStates[i] =
            digitalRead(e.pinA);
    }

    attachEncoderInterrupts();

    Serial.println("READY");
}

// ============================================
// UPDATE LOOP
// ============================================

void MotionController::update() {

    uint32_t now = micros();

    if (
        now - lastLoopUs
        < LOOP_US
    )
        return;

    lastLoopUs += LOOP_US;

    processSerial();

    updateMotors();

    sendEncoderData();
}

// ============================================
// SERIAL
// ============================================

void MotionController::processSerial() {

    while (Serial.available()) {

        char c = Serial.read();

        // ignore CR

        if (c == '\r')
            continue;

        // line complete

        if (c == '\n') {

            serialBuffer[
                serialIndex
            ] = '\0';

            if (serialIndex > 0) {

                processCommand(
                    serialBuffer
                );
            }

            serialIndex = 0;

            continue;
        }

        // append char

        if (
            serialIndex
            < SERIAL_BUFFER_SIZE - 1
        ) {

            serialBuffer[
                serialIndex++
            ] = c;
        }

        // overflow protection

        else {

            serialIndex = 0;
        }
    }
}

// ============================================
// COMMANDS
// ============================================

void MotionController::processCommand(
    char* cmd
) {

    // ENABLE

    if (
        strncmp(cmd, "EN", 2) == 0
    ) {

        int m;
        int state;

        if (
            sscanf(
                cmd,
                "EN,%d,%d",
                &m,
                &state
            ) != 2
        )
            return;

        if (!validMotor(m))
            return;

        motors[m].enabled = state;

        digitalWrite(
            hw.motors[m].enablePin,
            state ? LOW : HIGH
        );

        Serial.println("OK");
    }

    // MOVE

    else if (
        strncmp(cmd, "MV", 2) == 0
    ) {

        int m;
        long steps;
        int dir;

        uint32_t speedUs;

        if (
            sscanf(
                cmd,
                "MV,%d,%ld,%d,%lu",
                &m,
                &steps,
                &dir,
                &speedUs
            ) != 4
        )
            return;

        if (!validMotor(m))
            return;

        auto& motor = motors[m];

        motor.mode = MOVE;

        motor.direction = dir;

        motor.stepsRemaining = steps;

        motor.stepIntervalUs = speedUs;

        digitalWrite(
            hw.motors[m].dirPin,
            dir
        );

        Serial.println("OK");
    }

    // HOME

    else if (
        strncmp(cmd, "HOME", 4) == 0
    ) {

        int m;
        int enc;
        int dir;

        uint32_t speedUs;
        uint32_t stallMs;

        long threshold;

        if (
            sscanf(
                cmd,
                "HOME,%d,%d,%d,%lu,%lu,%ld",
                &m,
                &enc,
                &dir,
                &speedUs,
                &stallMs,
                &threshold
            ) != 6
        )
            return;

        if (!validMotor(m))
            return;

        if (!validEncoder(enc))
            return;

        auto& motor = motors[m];

        motor.mode = HOME;

        motor.encoderIndex = enc;

        motor.direction = dir;

        motor.stepIntervalUs = speedUs;

        motor.stallWindowMs = stallMs;

        motor.stallThreshold = threshold;

        motor.stallStartEncoder =
            atomicReadEncoder(enc);

        motor.stallCheckStartMs =
            millis();

        digitalWrite(
            hw.motors[m].dirPin,
            dir
        );

        Serial.println("OK");
    }

    // STOP

    else if (
        strncmp(cmd, "ST", 2) == 0
    ) {

        int m;

        if (
            sscanf(
                cmd,
                "ST,%d",
                &m
            ) != 1
        )
            return;

        if (!validMotor(m))
            return;

        motors[m].mode = IDLE;

        Serial.println("OK");
    }

    // ZERO ENCODER

    else if (
        strncmp(
            cmd,
            "ZEROENC",
            7
        ) == 0
    ) {

        int enc;

        if (
            sscanf(
                cmd,
                "ZEROENC,%d",
                &enc
            ) != 1
        )
            return;

        if (!validEncoder(enc))
            return;

        atomicWriteEncoder(
            enc,
            0
        );

        Serial.println("OK");
    }

    // ESTOP

    else if (
        strncmp(
            cmd,
            "ESTOP",
            5
        ) == 0
    ) {

        emergencyStop();
    }

    // RESET

    else if (
        strncmp(
            cmd,
            "RESET",
            5
        ) == 0
    ) {

        for (
            int i = 0;
            i < hw.motorCount;
            i++
        ) {

            motors[i].estop = false;
        }

        Serial.println("RESET_OK");
    }
}

// ============================================
// MOTOR UPDATE
// ============================================

void MotionController::pulseStepPin(
    int motor
) {

    auto& m =
        hw.motors[motor];

    digitalWrite(
        m.stepPin,
        HIGH
    );

    delayMicroseconds(
        m.pulseWidthUs
    );

    digitalWrite(
        m.stepPin,
        LOW
    );
}

void MotionController::updateHoming(
    int i
) {

    auto& m = motors[i];

    long currentEnc =
        atomicReadEncoder(
            m.encoderIndex
        );

    uint32_t nowMs = millis();

    if (
        nowMs - m.stallCheckStartMs
        >= m.stallWindowMs
    ) {

        long delta = labs(
            currentEnc
            - m.stallStartEncoder
        );

        if (
            delta <=
            m.stallThreshold
        ) {

            m.mode = IDLE;

            atomicWriteEncoder(
                m.encoderIndex,
                0
            );

            Serial.print("HOMED,");

            Serial.println(i);

            return;
        }

        m.stallStartEncoder =
            currentEnc;

        m.stallCheckStartMs =
            nowMs;
    }
}

void MotionController::updateMotors() {

    uint32_t now = micros();

    for (
        int i = 0;
        i < hw.motorCount;
        i++
    ) {

        auto& m = motors[i];

        if (m.estop)
            continue;

        if (m.mode == IDLE)
            continue;

        if (
            now - m.lastStepUs
            >= m.stepIntervalUs
        ) {

            m.lastStepUs = now;

            pulseStepPin(i);

            if (m.mode == MOVE) {

                m.stepsRemaining--;

                if (
                    m.stepsRemaining <= 0
                ) {

                    m.mode = IDLE;

                    Serial.print("DONE,");

                    Serial.println(i);
                }
            }
        }

        if (m.mode == HOME)
            updateHoming(i);
    }
}

// ============================================
// ENCODERS
// ============================================

void MotionController::sharedEncoderISR() {

    instance->handleAllEncoderInterrupts();
}

void MotionController::handleAllEncoderInterrupts() {

    for (
        int i = 0;
        i < hw.encoderCount;
        i++
    ) {

        auto& e = hw.encoders[i];

        bool a =
            digitalRead(e.pinA);

        if (
            a ==
            lastEncoderAStates[i]
        )
            continue;

        bool b =
            digitalRead(e.pinB);

        if (a == b)
            encoderCounts[i]++;
        else
            encoderCounts[i]--;

        lastEncoderAStates[i] = a;
    }
}

void MotionController::attachEncoderInterrupts() {

    for (
        int i = 0;
        i < hw.encoderCount;
        i++
    ) {

        attachInterrupt(
            digitalPinToInterrupt(
                hw.encoders[i].pinA
            ),
            sharedEncoderISR,
            CHANGE
        );
    }
}

// ============================================
// ENCODER STREAM
// ============================================

void MotionController::sendEncoderData() {

    static uint32_t lastSend = 0;

    if (
        millis() - lastSend
        < 50
    )
        return;

    lastSend = millis();

    Serial.print("ENC");

    for (
        int i = 0;
        i < hw.encoderCount;
        i++
    ) {

        Serial.print(",");

        Serial.print(
            atomicReadEncoder(i)
        );
    }

    Serial.println();
}

// ============================================
// ESTOP
// ============================================

void MotionController::emergencyStop() {

    for (
        int i = 0;
        i < hw.motorCount;
        i++
    ) {

        motors[i].mode = IDLE;

        motors[i].estop = true;

        digitalWrite(
            hw.motors[i].enablePin,
            HIGH
        );
    }

    Serial.println("ESTOP");
}