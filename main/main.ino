#include <AccelStepper.h>
#define motorInterfaceType 4

const int joyX = A0; // Analog pin for the X-axis of the joystick
const int joyY = A1; // Analog pin for the Y-axis of the joystick

AccelStepper myStepper(motorInterfaceType, 8, 10, 9, 11);

void setup() {
  Serial.begin(9600);

  myStepper.setMaxSpeed(1000);
  myStepper.setAcceleration(300);
}

void loop() {

  int Xdir = analogRead(joyX);
  int Ydir = analogRead(joyY);

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