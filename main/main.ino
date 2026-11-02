#include <AccelStepper.h>
#include <Servo.h>
#define motorInterfaceType 4

const int joyX = A0; // Analog pin for the X-axis of the joystick
const int joyY = A1; // Analog pin for the Y-axis of the joystick

// May use a MUX switch here so we have wider range of motion for the arm. For now, we will just use 3 servos.
// Note: Will actually defo need a MUX lol, all the digital pins on the arduino are saturated and we still need to add some more functions such as release button and stuff.
const int shoulder = 11;
const int elbow = 12;
const int wrist = 13;

// Ultrasonic Sensor Pins
const int VCC = 4;
const int trig = 5;
const int echo = 6;

AccelStepper myStepper(motorInterfaceType, 7, 9, 8, 10);

void setup() {
  Serial.begin(9600);

  myStepper.setMaxSpeed(1000);
  myStepper.setAcceleration(150);
}

void loop() {

  int Xdir = analogRead(joyX);
  int Ydir = analogRead(joyY);
  int distance = getDistance();

  // Analogue readings from joystick for debugging purposes
/*
  Serial.print("X-axis: ");
  Serial.print(Xdir);
  Serial.print(" Y-axis: ");
  Serial.println(Ydir);
*/

  // Clockwise rotation of the stepper motor
  if (Xdir > 550) {
    float speed = map(Xdir, 550, 1023, 0, 800);
    myStepper.setSpeed(speed);
  }
  // Counter-clockwise rotation of the stepper motor
  else if (Xdir < 450) {
    float speed = map(Xdir, 450, 0, 0, 800);
    myStepper.setSpeed(-speed);
  }
  // Centre
  else {
    myStepper.setSpeed(0);
  }

  myStepper.runSpeed();
}