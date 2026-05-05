#define ENCA 5
#define ENCB 6

volatile int pos = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(ENCA, INPUT);
  pinMode(ENCB, INPUT);

  attachInterrupt(digitalPinToInterrupt(ENCA), readEncoder, RISING);
}

void readEncoder(){

  int b = digitalRead(ENCB);
  if(b > 0){
    pos++;
  }else {
    pos--;
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println(pos);
  delay(100);
}
