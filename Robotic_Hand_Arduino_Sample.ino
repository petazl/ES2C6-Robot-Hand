// University of Warwick School of Engineering
// ES2C6: Electro-mechanical System Design
// 2024/2025 Term 1
//
// Robot Hand Project (Systems stream)
// Project Instructor: James Atkinson
//
// Getting Started sample code
//
// This code let's you control the direction of robot gripper (either tightening or loosening)
// based on feedback from a force sensor.
//
// This code was originally created by Dr Adam Noel

// Initialise variables
const int pwm = 3 ;       // digital pin 3 will be used for PWM (Pulse Width Modulation) output
const int dir = 8 ;       // digital pin 8 will be used for high/low output
const int forcePin = A0;  // analogue pin A0 will be used to read the voltage from the force sensor
const int ledPin = 13;    // digital pin 13 is also wired to a built-in LED
int forceValue = 0;       // value read from force pin
uint8_t motorSpeed = 30;  // 8 bit unsigned integer (0-255) defining motor speed
bool bHigh = false;       // boolean tracking direction
bool bChangeDir = false;  // boolean tracking whether we need to switch directions

// Setup function (runs when Arduino is powered on or reset
void setup()
{
  pinMode(pwm,OUTPUT) ;     //we have to set PWM pin as output
  pinMode(dir,OUTPUT) ;     // Direction pin pins are also set as output
  pinMode(ledPin, OUTPUT);  // LED pin is an output
}

// Loop function runs repeatedly after the setup function
void loop()
{
  // Read voltage from output of force sensor
  forceValue = analogRead(forcePin);

  if (forceValue > 100)
  { // There is a high force applied to force sensor
    // Loosen grip
    if(bHigh)
      // we were just loosening
      bChangeDir = false;
    else
      // we were just tightening and need to change direction
      bChangeDir = true;
    bHigh = true;   
  } else
  { // There is a low force applied to force sensor
    // Tighten grip
    if(bHigh)
      // we were just loosening and need to change direction
      bChangeDir = true;
    else
      // we were just tightening
      bChangeDir = false;
    bHigh = false;   
  }

  // Stop motor for a brief delay because we need to change directions
  if(bChangeDir)
  {
    analogWrite(pwm,0);
    delay(250); // delay is in milliseconds
  }

  // Set motor direction
  if(bHigh)
  {
    // Loosen the grip and turn on LED
    digitalWrite(ledPin, HIGH);
    digitalWrite(dir,LOW);
  } else
  {
    // Tighten the grip and turn off LED
    digitalWrite(ledPin, LOW);
    digitalWrite(dir,HIGH);
  }

  // Resume motor motion
  analogWrite(pwm,motorSpeed);
}
