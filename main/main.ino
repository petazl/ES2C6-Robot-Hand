#include <Arduino.h>
#include <Servo.h>

// PIN ASSIGNMENTS

// Force-sensitive resistor
const int FSR = A0;

// Joystick (base rotation)
const int joyX = A1;
const int joyY = A2;  // UNUSED

// Joint potentiometers
const int potShoulder = A3;
const int potElbow = A4;
const int potWrist = A5;

// Ultrasonic sensor
const int trig = 2;
const int echo = 3;

// Servo signal pins
const int shoulder = 4;
const int elbow = 5;
const int wrist = 6;
const int base = 7;

// Gripper DC motor driver
const int dir = 8;
const int pwm = 9;

// Other peripherals
const int ledPin = 10;
const int release = 11;

// *CONFIGURATION*

// Gripper settings
const int FSR_THRESHOLD = 200;       // FSR threshold - needs calibration
const int GRIPPER_SPEED = 150;       // PWM: 0-255
const unsigned long RELEASE_TIME = 10000; // 10s release time

const int CLOSE_DIR = HIGH;
const int OPEN_DIR = LOW;

// Object detection distance
const float DETECTION_DISTANCE = 15.0;

// Servo mechanical limits 
const int SHOULDER_MIN = 20;
const int SHOULDER_MAX = 150;
const int ELBOW_MIN = 30;
const int ELBOW_MAX = 160;
const int WRIST_MIN = 20;
const int WRIST_MAX = 160;
const int BASE_MIN = 10;
const int BASE_MAX = 170;

// Joystick centre deadband
const int JOYSTICK_MIN = 480;
const int JOYSTICK_MAX = 544;

// Gripper state machine
enum GripperState {
    IDLE,
    CLOSING,
    HOLDING,
    RELEASING,
    WAIT_CLEAR
};

GripperState gripperState = IDLE;
unsigned long releaseStartTime = 0;
int previousButtonState = HIGH;

// Servo objects and positions
Servo shoulderServo;
Servo elbowServo;
Servo wristServo;
Servo baseServo;

int shoulderAngle = 90;
int elbowAngle = 90;
int wristAngle = 90;
int baseAngle = 90;

// Setup
void setup() {
    Serial.begin(9600);

    // Configure pins
    pinMode(trig, OUTPUT);
    pinMode(echo, INPUT);
    pinMode(pwm, OUTPUT);
    pinMode(dir, OUTPUT);
    pinMode(release, INPUT_PULLUP);
    pinMode(ledPin, OUTPUT);

    // Ensure gripper motor is stopped initially
    analogWrite(pwm, 0);

    // Attach servos
    shoulderServo.attach(shoulder);
    elbowServo.attach(elbow);
    wristServo.attach(wrist);
    baseServo.attach(base);

    // Set initial servo positions
    shoulderServo.write(shoulderAngle);
    elbowServo.write(elbowAngle);
    wristServo.write(wristAngle);
    baseServo.write(baseAngle);

    digitalWrite(ledPin, HIGH);
}


// Main loop
void loop() {
    joystick();
    servoControl();
    gripper();
}

// Base Joystick Control
void joystick() {
    int Xdir = analogRead(joyX);
    // Ignore small variations around joystick centre
    if (Xdir >= JOYSTICK_MIN && Xdir <= JOYSTICK_MAX) {
        return;
    }

    // Convert joystick position into servo angle
    baseAngle = map(Xdir, 0, 1023, BASE_MIN, BASE_MAX);
    baseAngle = constrain(baseAngle, BASE_MIN, BASE_MAX);
    baseServo.write(baseAngle);
}

// Arm Servo Control
void servoControl() {
    // Read potentiometers and map to joint angles
    shoulderAngle = map(
        analogRead(potShoulder), 0, 1023,
        SHOULDER_MIN, SHOULDER_MAX
    );

    elbowAngle = map(
        analogRead(potElbow), 0, 1023,
        ELBOW_MIN, ELBOW_MAX
    );

    wristAngle = map(
        analogRead(potWrist), 0, 1023,
        WRIST_MIN, WRIST_MAX
    );

    // Enforce mechanical limits
    shoulderAngle = constrain(
        shoulderAngle, SHOULDER_MIN, SHOULDER_MAX
    );

    elbowAngle = constrain(
        elbowAngle, ELBOW_MIN, ELBOW_MAX
    );

    wristAngle = constrain(
        wristAngle, WRIST_MIN, WRIST_MAX
    );

    // Update servo positions
    shoulderServo.write(shoulderAngle);
    elbowServo.write(elbowAngle);
    wristServo.write(wristAngle);
}

// Gripper Control
void gripper() {
    // Read sensors
    float distance = getDistance();
    int forceValue = analogRead(FSR);
    int buttonState = digitalRead(release);

    // Detect a new button press (INPUT_PULLUP)
    bool releasePressed =
        (previousButtonState == HIGH && buttonState == LOW);
    previousButtonState = buttonState;

    // Release button overrides everything
    if (releasePressed && gripperState != RELEASING) {
        gripperState = RELEASING;
        releaseStartTime = millis();
    }

    switch (gripperState) {

        case IDLE:
            // Detect object within 15 cm
            if (distance > 0 && distance <= DETECTION_DISTANCE) {
                gripperState = CLOSING;
            }
            break;

        case CLOSING:
            // Stop closing once sufficient force is detected
            if (forceValue >= FSR_THRESHOLD) {
                analogWrite(pwm, 0);
                gripperState = HOLDING;
            }
            else {
                digitalWrite(dir, CLOSE_DIR);
                analogWrite(pwm, GRIPPER_SPEED);
            }
            break;

        case HOLDING:
            // Keep motor stopped while holding object
            analogWrite(pwm, 0);
            break;

        case RELEASING:
            // Open gripper
            digitalWrite(dir, OPEN_DIR);
            analogWrite(pwm, GRIPPER_SPEED);

            // Stop after 10 seconds
            if (millis() - releaseStartTime >= RELEASE_TIME) {
                analogWrite(pwm, 0);
                gripperState = WAIT_CLEAR;
            }
            break;


        case WAIT_CLEAR:
            // Prevent automatically grabbing the same object again
            analogWrite(pwm, 0);
            if (distance < 0 || distance > DETECTION_DISTANCE) {
                gripperState = IDLE;
            }
            break;
    }
}

// Ultrasonic Sensor
float getDistance() {
    digitalWrite(trig, LOW);
    delayMicroseconds(2);
    digitalWrite(trig, HIGH);
    delayMicroseconds(10);
    digitalWrite(trig, LOW);

    unsigned long duration = pulseIn(echo, HIGH, 30000UL);

    // Return -1 if no echo is received
    if (duration == 0) {
        return -1;
    }

    // Convert echo duration into distance (cm)
    return duration * 0.0343 / 2.0;
}
