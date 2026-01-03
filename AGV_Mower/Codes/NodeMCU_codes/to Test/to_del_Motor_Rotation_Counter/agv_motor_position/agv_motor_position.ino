int hall_u = 6;
int signal_count = 0;
int rev_count = 0;
int deg_dist = 0;
int totaldist = 0;
const int rev_dist = (165 * 3.14) / 10;
const int deg_count = 360 / 52 ;

void setup() {
  pinMode(hall_u, INPUT);
  Serial.begin(9600);
}
void loop() {
  if (digitalRead(hall_u) == 0) {
    signal_count += 1;
    deg_dist = deg_count * signal_count;
    totaldist =(rev_count * rev_dist) + deg_dist;
  }
  // when 52, it increases revcount
  if(signal_count%52==0){
  rev_count = rev_count + 1;
  }

  Serial.print("degree dist:");
  Serial.print(deg_dist);
  Serial.print("revolution count:");
  Serial.print(rev_count);
  Serial.print("total distance:");
  Serial.print(totaldist);

}
