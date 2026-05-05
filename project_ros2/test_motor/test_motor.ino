// ===== Motor Pin =====
#define l_dir1_pin 7
#define l_dir2_pin 8
#define l_pwm_pin  9


void setup() {
  Serial.begin(115200);
  pinMode(l_dir1_pin, OUTPUT);
  pinMode(l_dir2_pin, OUTPUT);
  pinMode(l_pwm_pin, OUTPUT);
}

void loop() {
  // สลับทิศทางทุก 2 วินาที

  digitalWrite(l_dir1_pin, HIGH);
  digitalWrite(l_dir2_pin, LOW);
  analogWrite(l_pwm_pin, 175);
  delay(1000);

  digitalWrite(l_dir1_pin, LOW);
  digitalWrite(l_dir2_pin, LOW);
  analogWrite(l_pwm_pin, 0);
  delay(1000);

  digitalWrite(l_dir1_pin, LOW);
  digitalWrite(l_dir2_pin, HIGH);
  analogWrite(l_pwm_pin, 175);
  delay(1000);

  digitalWrite(l_dir1_pin, LOW);
  digitalWrite(l_dir2_pin, LOW);
  analogWrite(l_pwm_pin, 0);
  delay(1000);

}