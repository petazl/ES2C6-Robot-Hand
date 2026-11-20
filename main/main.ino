#include <Arduino.h>
#include <Servo.h>

// ES2C6 Robotic Hand Revised schematic, Arduino Uno R3.
const byte FORCE_PIN = A0;
const byte SHOULDER_POT = A1;
const byte ELBOW_POT = A2;
const byte WRIST_TILT_POT = A3;
const byte JOYSTICK_X_PIN = A4;
const byte GRIPPER_DIR = 2;
const byte GRIPPER_PWM = 3;
const byte BASE_PIN = 4;
const byte SHOULDER_PIN = 5;
const byte ELBOW_PIN = 6;
const byte WRIST_TILT_PIN = 7;
const byte TRIG_PIN = 8;
const byte ECHO_PIN = 9;
const byte LED_PIN = 10;
const byte BUTTON_PIN = 11;

// Set each direction to 1 or -1 to match the assembled joint.
const int BASE_DIRECTION = 1;
const int SHOULDER_DIRECTION = 1;
const int ELBOW_DIRECTION = 1;
const int WRIST_TILT_DIRECTION = 1;
const int JOYSTICK_X_DIRECTION = 1;
const int BASE_STOP_US = 1500;  // Adjust until the FS90R stays still.
const int MANUAL_SWING_US = 300;
const int FS90R_MIN_US = 1200;
const int FS90R_MAX_US = 1800;
const int SG90_MIN_US = 1000;
const int SG90_MAX_US = 2000;

// Starting limits from the previous sketch. Check assembled clearance.
const float SHOULDER_MIN = 20, SHOULDER_MAX = 150;
const float ELBOW_MIN = 30, ELBOW_MAX = 160;
const float WRIST_TILT_MIN = 20, WRIST_TILT_MAX = 160;
const float POT_SLEW_RATE = 120.0f;  // Commanded degrees per second.
const int STICK_CENTER = 512;
const int STICK_DEADBAND = 32;
const unsigned long CONTROL_INTERVAL_MS = 20;
const unsigned long RANGE_INTERVAL_MS = 60;
const unsigned long BUTTON_DEBOUNCE_MS = 25;
const unsigned long NEAR_ECHO_US = 875;  // About 15 cm round-trip at 343 m/s.
const unsigned long ECHO_TIMEOUT_US = 3000;

Servo baseServo;
Servo shoulderServo;
Servo elbowServo;
Servo wristTiltServo;
float shoulderAngle = 90;
float elbowAngle = 90;
float wristTiltAngle = 90;

enum GripperState { READY, CLOSING, HOLDING, OPENING, WAIT_CLEAR };

struct GripperSettings {
    int forceThreshold;
    byte speed;
    byte closeDirection;
    unsigned long closeTimeMs;
    unsigned long fullOpenTimeMs;
};

GripperState gripperState = READY;
unsigned long gripperStartedAt = 0;
unsigned long gripperClosingMs = 0;  // Motor-on time since the last release.
unsigned long gripperOpeningMs = 0;
bool gripperReleased = false;
byte clearSamples = 0;
// Force threshold, PWM, close direction, close timeout, full opening time.
// Check the two-second close timeout on the bench.
const GripperSettings GRIPPER_SETTINGS{200, 150, HIGH, 2000, 10000};

void updateGripper(unsigned long now, bool release, bool rangeUpdated, bool objectNear,
                   int force, const GripperSettings &settings);

float clampValue(float value, float low, float high) {
    return fmaxf(low, fminf(value, high));
}

float stickCommand(int reading) {
    const int offset = reading - STICK_CENTER;
    if (offset > STICK_DEADBAND) {
        return float(offset - STICK_DEADBAND) /
               (1023 - STICK_CENTER - STICK_DEADBAND);
    }
    if (offset < -STICK_DEADBAND) {
        return float(offset + STICK_DEADBAND) / (STICK_CENTER - STICK_DEADBAND);
    }
    return 0;
}

void controlBase() {
    const float command = JOYSTICK_X_DIRECTION * stickCommand(analogRead(JOYSTICK_X_PIN));
    const float pulse = BASE_STOP_US + command * MANUAL_SWING_US * BASE_DIRECTION;
    baseServo.writeMicroseconds(lroundf(clampValue(pulse, FS90R_MIN_US, FS90R_MAX_US)));
}

void writeAngle(Servo &servo, float degrees, int direction) {
    const float angle = 90 + (degrees - 90) * direction;
    const float pulse = SG90_MIN_US + (SG90_MAX_US - SG90_MIN_US) * angle / 180.0f;
    servo.writeMicroseconds(lroundf(pulse));
}

void followPot(Servo &servo, float &angle, byte pin, float low, float high,
               int direction, float seconds) {
    const float target = low + (high - low) * (analogRead(pin) / 1023.0f);
    const float step = POT_SLEW_RATE * seconds;
    angle = clampValue(angle + clampValue(target - angle, -step, step), low, high);
    writeAngle(servo, angle, direction);
}

bool releasePressed(unsigned long now) {
    static bool lastRawDown = false;
    static bool down = false;
    static unsigned long changedAt = 0;
    const bool rawDown = digitalRead(BUTTON_PIN) == LOW;

    if (rawDown != lastRawDown) {
        lastRawDown = rawDown;
        changedAt = now;
    }
    if (rawDown != down && now - changedAt >= BUTTON_DEBOUNCE_MS) {
        down = rawDown;
        return down;  // One release event per press, including a held button.
    }
    return false;
}

unsigned long readEcho(unsigned long timeoutUs) {
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
    return pulseIn(ECHO_PIN, HIGH, timeoutUs);
}

void driveGripper(byte direction, byte speed) {
    analogWrite(GRIPPER_PWM, 0);
    digitalWrite(GRIPPER_DIR, direction);
    analogWrite(GRIPPER_PWM, speed);
}

void recordGripperClose(unsigned long now, unsigned long fullOpenTimeMs) {
    const unsigned long elapsed = now - gripperStartedAt;
    const unsigned long remaining = fullOpenTimeMs - gripperClosingMs;
    gripperClosingMs += elapsed < remaining ? elapsed : remaining;
}

void waitForGripperClear() {
    analogWrite(GRIPPER_PWM, 0);
    gripperState = WAIT_CLEAR;
    clearSamples = 0;
}

void updateGripper(unsigned long now, bool release, bool rangeUpdated, bool objectNear,
                   int force, const GripperSettings &settings) {
    if (release && gripperState != OPENING) {
        if (gripperState == CLOSING) {
            recordGripperClose(now, settings.fullOpenTimeMs);
        }
        gripperOpeningMs = gripperClosingMs;
        if (gripperOpeningMs == 0 && !gripperReleased) {
            gripperOpeningMs = settings.fullOpenTimeMs;  // Starting jaw position is unknown.
        }
        if (gripperOpeningMs > 0) {
            gripperState = OPENING;
            gripperStartedAt = now;
            driveGripper(!settings.closeDirection, settings.speed);
        } else {
            waitForGripperClear();
        }
        return;
    }

    switch (gripperState) {
        case READY:
            if (rangeUpdated && objectNear) {
                if (force >= settings.forceThreshold) {
                    gripperState = HOLDING;
                } else {
                    gripperState = CLOSING;
                    gripperStartedAt = now;
                    driveGripper(settings.closeDirection, settings.speed);
                }
            }
            break;
        case CLOSING:
            if (force >= settings.forceThreshold) {
                recordGripperClose(now, settings.fullOpenTimeMs);
                analogWrite(GRIPPER_PWM, 0);
                gripperState = HOLDING;
            } else if (now - gripperStartedAt >= settings.closeTimeMs) {
                recordGripperClose(now, settings.fullOpenTimeMs);
                waitForGripperClear();
            }
            break;
        case HOLDING:
            break;
        case OPENING:
            if (now - gripperStartedAt >= gripperOpeningMs) {
                gripperClosingMs = 0;
                gripperReleased = true;
                waitForGripperClear();
            }
            break;
        case WAIT_CLEAR:
            if (rangeUpdated) {
                clearSamples = objectNear ? 0 : clearSamples + 1;
                if (clearSamples >= 3) {
                    gripperState = READY;
                }
            }
            break;
    }
}

void setup() {
    pinMode(GRIPPER_PWM, OUTPUT);
    analogWrite(GRIPPER_PWM, 0);
    pinMode(GRIPPER_DIR, OUTPUT);
    pinMode(TRIG_PIN, OUTPUT);
    digitalWrite(TRIG_PIN, LOW);
    pinMode(ECHO_PIN, INPUT);
    pinMode(BUTTON_PIN, INPUT_PULLUP);  // Joystick switch connects D11 to ground.
    pinMode(LED_PIN, OUTPUT);
    baseServo.attach(BASE_PIN);
    shoulderServo.attach(SHOULDER_PIN);
    elbowServo.attach(ELBOW_PIN);
    wristTiltServo.attach(WRIST_TILT_PIN);
    baseServo.writeMicroseconds(BASE_STOP_US);
    writeAngle(shoulderServo, shoulderAngle, SHOULDER_DIRECTION);
    writeAngle(elbowServo, elbowAngle, ELBOW_DIRECTION);
    writeAngle(wristTiltServo, wristTiltAngle, WRIST_TILT_DIRECTION);
    digitalWrite(LED_PIN, HIGH);  // Firmware is running.
}

void loop() {
    static unsigned long lastRange = 0;
    static unsigned long lastControl = 0;
    static bool objectNear = false;
    const unsigned long now = millis();
    const bool release = releasePressed(now);
    const bool rangeUpdated = now - lastRange >= RANGE_INTERVAL_MS;
    if (rangeUpdated) {
        lastRange = now;
        const unsigned long echo = readEcho(ECHO_TIMEOUT_US);
        objectNear = echo > 0 && echo <= NEAR_ECHO_US;
    }
    updateGripper(now, release, rangeUpdated, objectNear,
                  analogRead(FORCE_PIN), GRIPPER_SETTINGS);

    const unsigned long elapsed = now - lastControl;
    if (elapsed < CONTROL_INTERVAL_MS) {
        return;
    }
    lastControl = now;
    const float seconds = fminf(elapsed * 0.001f, 0.05f);
    controlBase();
    followPot(shoulderServo, shoulderAngle, SHOULDER_POT,
              SHOULDER_MIN, SHOULDER_MAX, SHOULDER_DIRECTION, seconds);
    followPot(elbowServo, elbowAngle, ELBOW_POT,
              ELBOW_MIN, ELBOW_MAX, ELBOW_DIRECTION, seconds);
    followPot(wristTiltServo, wristTiltAngle, WRIST_TILT_POT,
              WRIST_TILT_MIN, WRIST_TILT_MAX, WRIST_TILT_DIRECTION, seconds);
}
