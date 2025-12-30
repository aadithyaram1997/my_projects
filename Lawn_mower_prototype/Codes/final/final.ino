// ultrasound
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Servo.h>

LiquidCrystal_I2C lcd = LiquidCrystal_I2C(0x27, 16, 2);

int trigPin = A0;
int echoPin = A3;
Servo servo;

long distance;
long duration;

int variable;
int rd;
int ld;
long ULTRASONIC();

//IR pins
//define IR module pins
#define LS 8     // left sensor
#define RS 2     // right sensor
#define MS 13     // center sensor
int IRvalueD1 = 0; //left sensor state
int IRvalueD2 = 0; //right sensor state
int IRvalueD3 = 0; //center sensor state

//define Line follower variables
int vSpeed = 180;        // MAX 255
int turn_speed = 180;    // MAX 255 
int turn_delay =5;
int vSpeedr = 180, vSpeedl=180; //Turning PWM Values
int rvSpeedr = 180, rvSpeedl=180;
int vTurnf = 180, vTurns=18; //Turning

//Aligning variables 
boolean alignFlag=0;
boolean perpendicularFlag=0;



// motor pins

// Motor A
int motor1Pin1 = 4; 
int motor1Pin2 = 5; 
int enable1Pin = 9; 

// Motor B
int motor2Pin1 = 7; 
int motor2Pin2 = 6;
int enable2Pin = 10;

void motor_function(char m_state) ;

// Setting PWM properties
const int freq = 30000;
const int pwmChannel = 0;
const int resolution = 8;
int dutyCycle = 200;

String motionState = "";


void setup() {
  // sets the pins as outputs:
  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  pinMode(enable1Pin, OUTPUT);
  pinMode(motor2Pin1, OUTPUT);
  pinMode(motor2Pin2, OUTPUT);
  pinMode(enable2Pin, OUTPUT);

   TCCR2B = TCCR2B & B11111000 | B00000111; // for PWM frequency of 30.64 Hz
  
  lcd.init();
  lcd.backlight();
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  servo.attach(12);
  servo.write(90);

    //IR module pin Definitions
  pinMode(LS, INPUT);
  pinMode(RS, INPUT);
  pinMode(MS, INPUT);
   

  Serial.begin(9600);

  // testing
  Serial.print("Testing DC Motor...");
}

void loop() {
  //control speed 
  analogWrite(enable1Pin, 255);
  analogWrite(enable2Pin, 255); 

  variable = ULTRASONIC();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("DISTANCE CM:");
  lcd.print(distance);
  
  delay(200);

  motor_function('F');
  lcd.print(F("fwd"));
  
  if (variable < 8){
     motor_function('B');
     lcd.print(F("bwd"));
     delay(1000);
  
     motor_function('S');
     lcd.print(F("stop"));
     servo.write(0);
     rd = ULTRASONIC;
     delay(500);
     servo.write(180);
     ld = ULTRASONIC;
     delay(500);
     servo.write(90);

     if (rd > ld) {
          motionState = "rgt"; //Feedback parameter
          motor_function('R');
          delay(300);
          motor_function('F');
          lcd.print(F("fwd"));
     } else {
          motionState = "lft"; //Feedback parameter
          motor_function('L');
          delay(300);
          motor_function('F');
          lcd.print(F("fwd"));
     }
  }
}

long ULTRASONIC() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance = duration * 0.034 / 2;
  return distance;
}
