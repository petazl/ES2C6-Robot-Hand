#include <Arduino.h>
#include <Servo.h>
#include <Wire.h>
#include "Stabilization.h"

// ES2C6 Robotic Hand schematic, Arduino Uno R3.
const byte FORCE_PIN = A0;
const byte MUX_COMMON = A1;
const byte LED_PIN = A2;
const byte GRIPPER_DIR = 2;
const byte GRIPPER_PWM = 3;
const byte BASE_PIN = 4;
const byte SHOULDER_PIN = 5;
const byte ELBOW_PIN = 6;
const byte WRIST_TILT_PIN = 7;
const byte WRIST_ROLL_PIN = 8;
const byte TRIG_PIN = 9;
const byte ECHO_PIN = 10;
const byte MUX_S0 = 11;
const byte MUX_S1 = 12;
const byte MUX_S2 = 13;

const byte SHOULDER_CHANNEL = 0;
const byte ELBOW_CHANNEL = 1;
const byte WRIST_TILT_CHANNEL = 2;
const byte JOYSTICK_X_CHANNEL = 3;
const byte JOYSTICK_Y_CHANNEL = 4;
const byte BUTTON_CHANNEL = 5;
const byte IMU_ADDRESS = 0x68;  // GY-521 AD0 is grounded, SDA/SCL are A4/A5.

struct ImuSample {
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
};

enum ButtonEvent { NO_PRESS, SHORT_PRESS, LONG_PRESS };

enum Joint { BASE, SHOULDER, ELBOW, WRIST_TILT, WRIST_ROLL, JOINT_COUNT };

// Joint signs: positive base turns left, positive pitch tips the claw down,
// and positive roll lifts the claw's left side. Verify with the arm supported.
const byte JOINT_PINS[JOINT_COUNT] = {BASE_PIN, SHOULDER_PIN, ELBOW_PIN, WRIST_TILT_PIN, WRIST_ROLL_PIN};
const int JOINT_DIRECTION[JOINT_COUNT] = {1, 1, 1, 1, 1};
const int JOYSTICK_X_DIRECTION = 1, JOYSTICK_Y_DIRECTION = 1;
const int BASE_STOP_US = 1500;        // Adjust each FS90R until it stays still.
const int WRIST_ROLL_STOP_US = 1500;
const int MANUAL_SWING_US = 300;
const int STABILIZATION_SWING_US = 100;
const int FS90R_MIN_US = 1200;
const int FS90R_MAX_US = 1800;
const int SG90_MIN_US = 1000;
const int SG90_MAX_US = 2000;

// Bench starting points from the previous sketch. Check assembled clearance.
const float SHOULDER_MIN = 20, SHOULDER_MAX = 150;
const float ELBOW_MIN = 30, ELBOW_MAX = 160;
const float WRIST_TILT_MIN = 20, WRIST_TILT_MAX = 160;
const float POT_SLEW_RATE = 120.0f;

// Map the sensor's signed axes to claw forward, left and up. Use ±1 for X,
// ±2 for Y and ±3 for Z. Keep the claw still at startup for gyro calibration.
const int IMU_FORWARD = 1, IMU_LEFT = 2, IMU_UP = 3;
bool stabilizationCalibrated = false;  // Enable once axes and servo signs are verified.
const float TRIM_RATE = 18.0f;
const float MAX_CORRECTION_RATE = 45.0f;
const float WRIST_PITCH_RATE = 30.0f;

Servo servos[JOINT_COUNT];
float shoulderAngle = 90;
float elbowAngle = 90;
float wristTiltAngle = 90;
bool stabilizing = false;
bool imuOnline = false;
byte biasSamples = 0;
Vector3 gyroBias{};
Attitude attitude;
Rotation targetOrientation{1, 0, 0, 0};
Vector3 lastPitchAxis{0, 1, 0};
Pid headingPid, pitchPid, rollPid;

const unsigned long CONTROL_INTERVAL_MS = 20;
const unsigned long RANGE_INTERVAL_MS = 60;
const unsigned long LONG_PRESS_MS = 800;
const unsigned long NEAR_ECHO_US = 875;  // 15 cm round-trip at 343 m/s.
const unsigned long ECHO_TIMEOUT_US = 3000;
const int STICK_CENTER = 512;
const int STICK_DEADBAND = 32;

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
unsigned long gripperClosingMs = 0;  // Time spent driving closed since the last release.
unsigned long gripperOpeningMs = 0;
bool gripperReleased = false;
byte clearSamples = 0;
// Force, PWM, close direction, close timeout, full-stroke opening time.
// The two-second close timeout is a starting point for bench calibration.
const GripperSettings GRIPPER_SETTINGS{200, 150, HIGH, 2000, 10000};

bool readImu(ImuSample &sample);
ButtonEvent updateButton(bool rawDown, unsigned long now, unsigned long longPressMs);
void updateGripper(unsigned long now, bool release, bool rangeUpdated, bool objectNear,
                   int force, const GripperSettings &settings);

int readMux(byte channel) {
    digitalWrite(MUX_S0, channel & 1);
    digitalWrite(MUX_S1, (channel >> 1) & 1);
    digitalWrite(MUX_S2, (channel >> 2) & 1);
    delayMicroseconds(5);
    analogRead(MUX_COMMON);  // Discard the first conversion after switching channels.
    return analogRead(MUX_COMMON);
}

bool buttonIsDown() {
    return readMux(BUTTON_CHANNEL) < 512;  // Switch to ground, external 10 kΩ pull-up.
}

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

float potAngle(int reading, float minimum, float maximum) {
    return minimum + (maximum - minimum) * (reading / 1023.0f);
}

void writeSpeed(Joint joint, int neutral, int swing, float command) {
    const float offset = clampValue(command, -1, 1) * swing * JOINT_DIRECTION[joint];
    servos[joint].writeMicroseconds(lroundf(clampValue(neutral + offset, FS90R_MIN_US, FS90R_MAX_US)));
}

void writeAngle(Joint joint, float degrees) {
    const float angle = 90 + (degrees - 90) * JOINT_DIRECTION[joint];
    const float pulse = SG90_MIN_US + (SG90_MAX_US - SG90_MIN_US) * angle / 180.0f;
    servos[joint].writeMicroseconds(lroundf(pulse));
}

float movePosition(Joint joint, float &angle, float change, float low, float high) {
    const float previous = angle;
    angle = clampValue(angle + change, low, high);
    writeAngle(joint, angle);
    return angle - previous;
}

void followPot(Joint joint, float &angle, byte channel, float low, float high, float seconds) {
    const float target = potAngle(readMux(channel), low, high);
    const float step = POT_SLEW_RATE * seconds;
    movePosition(joint, angle, clampValue(target - angle, -step, step), low, high);
}

void manualControl(float x, float y, float seconds) {
    writeSpeed(BASE, BASE_STOP_US, MANUAL_SWING_US, x);
    writeSpeed(WRIST_ROLL, WRIST_ROLL_STOP_US, MANUAL_SWING_US, y);
    followPot(SHOULDER, shoulderAngle, SHOULDER_CHANNEL, SHOULDER_MIN, SHOULDER_MAX, seconds);
    followPot(ELBOW, elbowAngle, ELBOW_CHANNEL, ELBOW_MIN, ELBOW_MAX, seconds);
    followPot(WRIST_TILT, wristTiltAngle, WRIST_TILT_CHANNEL,
              WRIST_TILT_MIN, WRIST_TILT_MAX, seconds);
}

ButtonEvent updateButton(bool rawDown, unsigned long now, unsigned long longPressMs) {
    const unsigned long debounceMs = 25;
    static bool lastRawDown = false;
    static bool down = false;
    static bool longHandled = false;
    static unsigned long changedAt = 0;
    static unsigned long pressedAt = 0;

    if (rawDown != lastRawDown) {
        lastRawDown = rawDown;
        changedAt = now;
    }
    if (rawDown != down && now - changedAt >= debounceMs) {
        down = rawDown;
        if (down) {
            pressedAt = changedAt;
            longHandled = false;
        } else if (!longHandled) {
            return changedAt - pressedAt >= longPressMs ? LONG_PRESS : SHORT_PRESS;
        }
    }
    if (down && rawDown && !longHandled && now - pressedAt >= longPressMs) {
        longHandled = true;
        return LONG_PRESS;
    }
    return NO_PRESS;
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

bool writeImuRegister(byte reg, byte value) {
    Wire.beginTransmission(IMU_ADDRESS);
    Wire.write(reg);
    Wire.write(value);
    return Wire.endTransmission() == 0;
}

bool startImu() {
    return writeImuRegister(0x6B, 0x01) &&  // Wake, use gyro X clock.
           writeImuRegister(0x1A, 0x03) &&  // Enable the ~44 Hz low-pass filter.
           writeImuRegister(0x1B, 0x00) &&  // Gyroscope: +/-250 degrees per second.
           writeImuRegister(0x1C, 0x00);    // Accelerometer: +/-2 g.
}

int16_t readImuWord() {
    uint16_t high = Wire.read();
    uint16_t low = Wire.read();
    return static_cast<int16_t>((high << 8) | low);
}

bool readImu(ImuSample &sample) {
    Wire.beginTransmission(IMU_ADDRESS);
    Wire.write(0x3B);  // First accelerometer register.
    if (Wire.endTransmission(false) != 0 || Wire.requestFrom(IMU_ADDRESS, byte(14)) != 14) {
        return false;
    }

    sample.ax = readImuWord();
    sample.ay = readImuWord();
    sample.az = readImuWord();
    readImuWord();  // Temperature.
    sample.gx = readImuWord();
    sample.gy = readImuWord();
    sample.gz = readImuWord();
    return true;
}

Vector3 clawAxes(int16_t x, int16_t y, int16_t z, float scaleFactor) {
    const float values[] = {0, float(x), float(y), float(z)};
    return {
        values[abs(IMU_FORWARD)] * (IMU_FORWARD < 0 ? -scaleFactor : scaleFactor),
        values[abs(IMU_LEFT)] * (IMU_LEFT < 0 ? -scaleFactor : scaleFactor),
        values[abs(IMU_UP)] * (IMU_UP < 0 ? -scaleFactor : scaleFactor)
    };
}

bool imuAxesValid() {
    if (abs(IMU_FORWARD) > 3 || abs(IMU_LEFT) > 3 || abs(IMU_UP) > 3) {
        return false;
    }
    const Vector3 x = clawAxes(1, 0, 0, 1);
    const Vector3 y = clawAxes(0, 1, 0, 1);
    const Vector3 z = clawAxes(0, 0, 1, 1);
    return dot(cross(x, y), z) == 1;
}

bool updateAttitude(float seconds) {
    ImuSample sample;
    if (!readImu(sample)) {
        return false;
    }
    const Vector3 acceleration = clawAxes(sample.ax, sample.ay, sample.az, 1.0f / 16384);
    Vector3 gyro = clawAxes(sample.gx, sample.gy, sample.gz, 1.0f / 131);
    if (biasSamples < 32) {
        const float gSquared = dot(acceleration, acceleration);
        if (gSquared > 0.94f && gSquared < 1.06f && dot(gyro, gyro) < 16.0f) {
            ++biasSamples;
            gyroBias.x += (gyro.x - gyroBias.x) / biasSamples;
            gyroBias.y += (gyro.y - gyroBias.y) / biasSamples;
            gyroBias.z += (gyro.z - gyroBias.z) / biasSamples;
        } else {
            biasSamples = 0;
            gyroBias = {};
        }
    }
    if (biasSamples == 32) {
        gyro = {gyro.x - gyroBias.x, gyro.y - gyroBias.y, gyro.z - gyroBias.z};
    }
    attitude.update(acceleration, gyro, seconds);
    return true;
}

void stopRotation() {
    writeSpeed(BASE, BASE_STOP_US, MANUAL_SWING_US, 0);
    writeSpeed(WRIST_ROLL, WRIST_ROLL_STOP_US, MANUAL_SWING_US, 0);
}

void controlPitch(float correctionRate, float seconds) {
    const float wristRate = clampValue(correctionRate, -WRIST_PITCH_RATE, WRIST_PITCH_RATE);
    float remaining = correctionRate * seconds;
    remaining -= movePosition(WRIST_TILT, wristTiltAngle, wristRate * seconds,
                              WRIST_TILT_MIN, WRIST_TILT_MAX);
    remaining -= movePosition(ELBOW, elbowAngle, remaining, ELBOW_MIN, ELBOW_MAX);
    movePosition(SHOULDER, shoulderAngle, remaining, SHOULDER_MIN, SHOULDER_MAX);
}

void stabilize(float x, float y, float seconds) {
    // The joystick trims the held heading and roll instead of bypassing the PID.
    if (x != 0 || y != 0) {
        const Rotation yawTrim = rotationAround({0, 0, 1}, x * TRIM_RATE * seconds);
        const Rotation rollTrim = rotationAround({1, 0, 0}, y * TRIM_RATE * seconds);
        targetOrientation = normalized(multiply(multiply(yawTrim, targetOrientation), rollTrim));
    }

    const Vector3 forward = rotate(attitude.estimate, {1, 0, 0});
    const float horizontal = sqrtf(forward.x * forward.x + forward.y * forward.y);
    if (horizontal >= 0.2f) {
        lastPitchAxis = {-forward.y / horizontal, forward.x / horizontal, 0};
    }
    const JointCorrection err = jointCorrection(angleError(targetOrientation, attitude.estimate),
                                                 forward, lastPitchAxis);
    const JointCorrection rate = jointCorrection(attitude.rate, forward, lastPitchAxis);
    float heading = 0;
    if (horizontal >= 0.2f) {
        heading = headingPid.update(err.heading, rate.heading, seconds, MAX_CORRECTION_RATE);
    } else {
        headingPid.integral = 0;
    }
    const float roll = rollPid.update(err.roll, rate.roll, seconds, MAX_CORRECTION_RATE);
    const float pitch = pitchPid.update(err.pitch, rate.pitch, seconds, MAX_CORRECTION_RATE);
    writeSpeed(BASE, BASE_STOP_US, STABILIZATION_SWING_US, heading / MAX_CORRECTION_RATE);
    writeSpeed(WRIST_ROLL, WRIST_ROLL_STOP_US, STABILIZATION_SWING_US, roll / MAX_CORRECTION_RATE);
    controlPitch(pitch, seconds);
}

void setup() {
    pinMode(GRIPPER_PWM, OUTPUT);
    analogWrite(GRIPPER_PWM, 0);
    pinMode(GRIPPER_DIR, OUTPUT);
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
    pinMode(LED_PIN, OUTPUT);
    pinMode(MUX_S0, OUTPUT);
    pinMode(MUX_S1, OUTPUT);
    pinMode(MUX_S2, OUTPUT);
    for (byte i = 0; i < JOINT_COUNT; ++i) {
        servos[i].attach(JOINT_PINS[i]);
    }
    stopRotation();
    writeAngle(SHOULDER, shoulderAngle);
    writeAngle(ELBOW, elbowAngle);
    writeAngle(WRIST_TILT, wristTiltAngle);
    Wire.begin();
    Wire.setWireTimeout(3000, true);
    imuOnline = imuAxesValid() && startImu();
}

void loop() {
    static unsigned long lastRange = 0;
    static unsigned long lastControl = 0;
    static unsigned long lastImuRetry = 0;
    static bool objectNear = false;
    static bool toggleRequested = false;

    const unsigned long now = millis();
    const ButtonEvent button = updateButton(buttonIsDown(), now, LONG_PRESS_MS);
    const bool rangeUpdated = now - lastRange >= RANGE_INTERVAL_MS;
    if (rangeUpdated) {
        lastRange = now;
        const unsigned long echo = readEcho(ECHO_TIMEOUT_US);
        objectNear = echo > 0 && echo <= NEAR_ECHO_US;
    }
    updateGripper(now, button == SHORT_PRESS, rangeUpdated, objectNear,
                  analogRead(FORCE_PIN), GRIPPER_SETTINGS);

    toggleRequested = toggleRequested || button == LONG_PRESS;
    const unsigned long elapsed = now - lastControl;
    if (elapsed < CONTROL_INTERVAL_MS) {
        return;
    }
    lastControl = now;
    const float seconds = fminf(elapsed * 0.001f, 0.05f);
    if (!imuOnline && now - lastImuRetry >= 1000) {
        lastImuRetry = now;
        imuOnline = imuAxesValid() && startImu();
    }
    if (imuOnline && !updateAttitude(seconds)) {
        imuOnline = false;
        lastImuRetry = now;
        attitude = Attitude{};
        biasSamples = 0;
        gyroBias = {};
    }
    const bool freshAttitude = imuOnline && attitude.started;
    if (toggleRequested) {
        toggleRequested = false;
        if (stabilizing) {
            stabilizing = false;
            stopRotation();
        } else if (stabilizationCalibrated && freshAttitude && biasSamples == 32) {
            targetOrientation = attitude.estimate;
            headingPid = pitchPid = rollPid = Pid{};
            stabilizing = true;
        }
    }
    digitalWrite(LED_PIN, stabilizing ? HIGH : LOW);
    if (stabilizing && !freshAttitude) {
        stabilizing = false;
        digitalWrite(LED_PIN, LOW);
        stopRotation();
        return;
    }

    const float x = JOYSTICK_X_DIRECTION * stickCommand(readMux(JOYSTICK_X_CHANNEL));
    const float y = JOYSTICK_Y_DIRECTION * stickCommand(readMux(JOYSTICK_Y_CHANNEL));
    if (stabilizing) {
        stabilize(x, y, seconds);
    } else {
        manualControl(x, y, seconds);
    }
}
