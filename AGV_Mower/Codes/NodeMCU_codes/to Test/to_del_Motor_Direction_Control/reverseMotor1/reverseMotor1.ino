bool reverseState  = false;
char state;
byte reverseLine1 = 13;
byte reverseLine2 = 12;
int analogValue  = 180;

unsigned long currentMillis = 0;
unsigned long previousMillis = 0;
const long interval = 1000;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(reverseLine1, OUTPUT);
  pinMode(reverseLine2, OUTPUT);
  digitalWrite(reverseLine1, HIGH);
  digitalWrite(reverseLine2, HIGH);
// pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  currentMillis = millis();
  if (Serial.available() > 0) {
    state = Serial.read();
    // Serial.println("");
    // Serial.println(state);
    // Serial.println("");
    if (state == 'f') {
      Serial.println("in f");
      if (reverseState == true) {
        reverseState = false;
        Serial.println("in f when rev true");
        previousMillis = currentMillis;
        switchState();
        speedChange(analogValue);
      }
      speedChange(analogValue);
    }
    if (state == 'b') {
      Serial.println("in b");
      if (reverseState == false) {
        reverseState = true;
        Serial.println("in b when rev false");
        previousMillis = currentMillis;
        switchState();
        speedChange(analogValue);
      }
      speedChange(analogValue);
    }
  }
}

void switchState() {
  speedChange(0);
  delay(1500);
  digitalWrite(reverseLine1, LOW);
  digitalWrite(reverseLine2, LOW);
//  digitalWrite(LED_BUILTIN, HIGH);
  delay(1050);
  digitalWrite(reverseLine1, HIGH);
  digitalWrite(reverseLine2, HIGH);
  delay(1500);
//  digitalWrite(LED_BUILTIN, LOW);

}

void speedChange(int value) {
  dacWrite(25, value);
    dacWrite(26, value);
  //Serial.println("speed val " + value);
}
