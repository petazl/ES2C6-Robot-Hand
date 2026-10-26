// Initialise variables
const int pwm = 2;
const int dir = 3;

const int echo = 8;
const int trig = 9;
const int vcc = 10;

const int forcePin = A0;
const int ledPin = 13;

int forceValue = 0;
long duration;
float distance;

uint8_t motorSpeed = 160;


// Setup
void setup()
{
  pinMode(pwm, OUTPUT);
  pinMode(dir, OUTPUT);

  pinMode(echo, INPUT);
  pinMode(trig, OUTPUT);

  // Use Arduino pin 10 as 5 V supply
  pinMode(vcc, OUTPUT);
  digitalWrite(vcc, HIGH);

  pinMode(ledPin, OUTPUT);

  Serial.begin(9600);
}


// Main loop
void loop()
{
  // Read ultrasonic sensor
  digitalWrite(trig, LOW);
  delayMicroseconds(2);

  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);

  duration = pulseIn(echo, HIGH);

  // Convert time to distance in cm
  distance = duration * 0.0343 / 2;


  // Read force sensor
  forceValue = analogRead(forcePin);


  // Object detected
  if (distance < 15 && distance > 0)
  {
    // Check whether we have grabbed something
    if (forceValue > 100)
    {
      // Object grabbed → stop motor
      analogWrite(pwm, 0);
      digitalWrite(ledPin, HIGH);

      Serial.println("OBJECT GRIPPED");
    }
    else
    {
      // Object detected but not yet grabbed → close
      digitalWrite(dir, HIGH);
      analogWrite(pwm, motorSpeed);

      digitalWrite(ledPin, LOW);

      Serial.println("CLOSING");
    }
  }
  else
  {
    // No object → open
    digitalWrite(dir, LOW);
    analogWrite(pwm, motorSpeed);

    digitalWrite(ledPin, LOW);

    Serial.println("OPENING");
  }

  delay(50);
}