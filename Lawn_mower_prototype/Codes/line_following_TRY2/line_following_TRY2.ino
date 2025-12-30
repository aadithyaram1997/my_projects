//IR pins
//define IR module pins
#define LS 8     // left sensor
#define RS 2     // right sensor
#define MS 13     // center sensor
int IRvalueD1 = 0; //left sensor state
int IRvalueD2 = 0; //right sensor state
int IRvalueD3 = 0; //center sensor state

//define Line follower variables
/*int vSpeed = 180;        // MAX 255
int turn_speed = 255;    // MAX 255 */
int turn_delay =5;
/*int vSpeedr = 180, vSpeedl=180; //Turning PWM Values
int rvSpeedr = 180, rvSpeedl=180;
int vTurnf = 180, vTurns=18; //Turning*/


//motor pins RIGHT
// Motor A
int motor1Pin1 = 4; 
int motor1Pin2 = 5; 
int enable1Pin = 9; 

// Motor B LEFT
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

   //IR module pin Definitions
  pinMode(LS, INPUT);
  pinMode(RS, INPUT);
  pinMode(MS, INPUT);


  Serial.begin(9600);
}

void loop() {
  //control speed 
  analogWrite(enable1Pin, 180);
  analogWrite(enable2Pin, 180); 

  IRvalueD1 = digitalRead(LS);
  IRvalueD2 = digitalRead(RS);
  Serial.println("Enter Line Follower mode");
  
  if(IRvalueD2 == 1 && IRvalueD1 == 0) { //right sensor detected border. Turn Right 
  
    Serial.println("turning right");
    motionState = "rgt"; //Feedback parameter
    motor_function('R');

    analogWrite(enable2Pin, 180);
    analogWrite(enable1Pin, 180);
    delay(turn_delay);
    //IRModule();
    }
  
  if(IRvalueD2 == 0 && IRvalueD1 == 1) {
    Serial.println("turning left");
    motionState = "lft"; //Feedback parameter
    motor_function('L');
    
    analogWrite(enable2Pin, 180);
    analogWrite(enable1Pin, 180);
    delay(turn_delay);
    //IRModule();
    }

  if(IRvalueD2 == 0 && IRvalueD1 == 0) {
    Serial.println("going forward");
    motionState = "fwd"; //Feedback parameter
    motor_function('F');
    

    analogWrite(enable2Pin, 180);
    analogWrite(enable1Pin, 180);
    //IRModule();
    delay(turn_delay);
   // IRModule();
   }

  if(IRvalueD2 == 1 && IRvalueD1 == 1) { 
    Serial.println("stop");
    motionState = "stop"; //Feedback parameter    
    motor_function('S');
    analogWrite(enable2Pin, 0);
    analogWrite(enable1Pin, 0);
    delay(1000);
  }
}
