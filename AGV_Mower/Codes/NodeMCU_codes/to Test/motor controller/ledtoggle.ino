
int reverse_flag = 0;
int buttonpress = 0;
const uint8_t LED_Fpin = 4; 
const uint8_t LED_Rpin = 7; 
const uint8_t buttonpin = 2;

void toggle_ISR();                        

void setup() {
  // put your setup code here, to run once:
  pinMode (LED_Fpin, OUTPUT);
  pinMode (LED_Rpin, OUTPUT);
  pinMode (buttonpin, INPUT_PULLUP);
  
  attachInterrupt (digitalPinToInterrupt(buttonpin), toggle_ISR, RISING);
}



void loop() {

  
  if (buttonpress == 1) {
     if (reverse_flag == 0) {
         digitalWrite(LED_Fpin, HIGH);
         digitalWrite(LED_Rpin, LOW);
         reverse_flag = 1;
     } else {
         digitalWrite(LED_Rpin, HIGH);
         digitalWrite(LED_Fpin, LOW);
         reverse_flag = 0;
     }
  }
   buttonpress = 0;
   delay(200);  // wait for sometime and then move on
   
}

void toggle_ISR() {
      buttonpress = 1;   // set the flag to inidicate interrupt occured
}
