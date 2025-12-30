#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Servo.h>

LiquidCrystal_I2C lcd = LiquidCrystal_I2C(0x27, 16, 2);

int trigPin = A0;
int echoPin = A3;
Servo servo;

long distance;
long duration;

void setup() {
  lcd.init();
  lcd.backlight();
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  servo.attach(13);
  servo.write(90);
}

/* When object detected, move backwards till distance = 15cm 
  turn motor to right
  rightdistance = ULTRASONIC();
  turn your serov  motor to left
  left distance = ULTRASONIC()
  if ( left ditance  greater than or equal right ditstance
  turnleft
  elseturnright*/

int variable;
int rd;
int ld;

void loop() {
  variable = ULTRASONIC();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("DISTANCE CM:");
  lcd.print(distance);
  delay(500);
  if (variable < 5)
  {
  servo.write(0);
  rd = ULTRASONIC;
  delay(500);
  servo.write(180);
  ld = ULTRASONIC;
  delay(500);
  servo.write(90);
  }
  /*variable  greater than or equal to 10//logic to see the object)
       //function for moving bot forward();
       else
         //function to stop
         //look left
         //store distacne on left
         //look right
         //store distacne on right
         //compare using if for more ditacne
         //turn in respectiv direction
         //move forward*/
  }

int ULTRASONIC() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance = duration * 0.034 / 2;
  return distance;
}
