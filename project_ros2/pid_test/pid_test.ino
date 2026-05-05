
// ---------- Pin Definition ----------
#define PIN_PWM   4
#define PIN_IN1   2
#define PIN_IN2   3
#define PIN_ENCA  5
#define PIN_ENCB  6

const int PWM_MIN = 140;
const int PWM_MAX = 255;

// ---------- Encoder ----------
volatile long encoderCount = 0;

void encoderISR() {
  int b = digitalRead(PIN_ENCB);
  if (b > 0) encoderCount--;
  else       encoderCount++;
}

// ---------- Motor ----------
void setMotor(int pwmVal) {
  if (pwmVal > 0) {
    // หมุนหน้า
    int actualPWM = map(pwmVal, 1, 255, PWM_MIN, PWM_MAX);
    digitalWrite(PIN_IN1, LOW);
    digitalWrite(PIN_IN2, HIGH);
    analogWrite(PIN_PWM, actualPWM);
  } else if (pwmVal < 0) {
    // หมุนหลัง
    int actualPWM = map(-pwmVal, 1, 255, PWM_MIN, PWM_MAX);
    digitalWrite(PIN_IN1, HIGH);
    digitalWrite(PIN_IN2, LOW);
    analogWrite(PIN_PWM, actualPWM);
  }else {
    // pwmVal == 0 → หยุดสนิท
    digitalWrite(PIN_IN1, LOW);
    digitalWrite(PIN_IN2, LOW);
    analogWrite(PIN_PWM, 0);
  }
}
  
// ---------- PID Parameters ----------
float Kp = 3.0;
float Ki = 5.0;
float Kd = 0.0;

float targetRPM = 0.0;   // ← ตั้ง RPM เป้าหมายที่นี่

// ---------- PID State ----------
float integral   = 0;
float prevError  = 0;
long  prevCount  = 0;

// ---------- Timing ----------
const int   LOOP_MS  = 100;          // คำนวณทุก 100ms
const float LOOP_SEC = LOOP_MS / 1000.0;

// PPR หลัง gearbox = 7 * 139 * 4 (quadrature) = 3892
// ถ้า encoder ต่อแค่ช่อง A เดียว = 7 * 139 = 973
const float PULSES_PER_REV = 973.0;  // ใช้ channel A เดียว

// ---------- Setup ----------
void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(PIN_PWM,  OUTPUT);
  pinMode(PIN_IN1,  OUTPUT);
  pinMode(PIN_IN2,  OUTPUT);
  pinMode(PIN_ENCA, INPUT_PULLUP);
  pinMode(PIN_ENCB, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(PIN_ENCA), encoderISR, RISING);

  Serial.println("=== PID Motor Test ===");
  Serial.println("Format: time(ms), targetRPM, actualRPM, pwm, error");
  Serial.println("ส่งคำสั่งผ่าน Serial: 'r30' = setpoint 30 RPM, 's' = stop");
}

// ---------- Loop ----------
void loop() {
  static unsigned long lastTime = 0;
  unsigned long now = millis();

  // รับคำสั่งจาก Serial Monitor
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd.startsWith("r")) {
      targetRPM = cmd.substring(1).toFloat();
      integral  = 0;
      prevError = 0;
      Serial.print(">> Set target RPM = ");
      Serial.println(targetRPM);
    } else if (cmd == "s") {
      targetRPM = 0;
      integral  = 0;
      setMotor(0);
      Serial.println(">> STOP");
    } else if (cmd.startsWith("p")) {
      Kp = cmd.substring(1).toFloat();
      Serial.print(">> Kp = "); Serial.println(Kp);
    } else if (cmd.startsWith("i")) {
      Ki = cmd.substring(1).toFloat();
      Serial.print(">> Ki = "); Serial.println(Ki);
    } else if (cmd.startsWith("d")) {
      Kd = cmd.substring(1).toFloat();
      Serial.print(">> Kd = "); Serial.println(Kd);
    }
  }

  // PID loop ทุก LOOP_MS
  if (now - lastTime >= LOOP_MS) {
    lastTime = now;

    // คำนวณ RPM จาก encoder
    long currentCount = encoderCount;
    long deltaPulses  = currentCount - prevCount;
    prevCount = currentCount;

    float actualRPM = (deltaPulses / PULSES_PER_REV) * (60.0 / LOOP_SEC);

    // ถ้า target = 0 หยุดทันที
    if (targetRPM == 0) {
      setMotor(0);
      integral  = 0;
      prevError = 0;
      Serial.print(now);    Serial.print(",   ");
      Serial.print(0.0);    Serial.print(",   ");
      Serial.print(actualRPM, 2); Serial.print(",   ");
      Serial.print(0);      Serial.print(",   ");
      Serial.println(0.0);
      return;
    }

    // PID
    float error = targetRPM - actualRPM;
    integral   += error * LOOP_SEC;
    float derivative = (error - prevError) / LOOP_SEC;
    prevError = error;

    // Clamp integral (anti-windup)
    integral = constrain(integral, -100, 100);

    float output = Kp * error + Ki * integral + Kd * derivative;
    int   pwmOut = (int)constrain(output, -255, 255);

    setMotor(pwmOut);

    // Print ผลลัพธ์
    Serial.print(now);       Serial.print(",   ");
    Serial.print(targetRPM); Serial.print(",   ");
    Serial.print(actualRPM, 2); Serial.print(",   ");
    Serial.print(pwmOut);    Serial.print(",   ");
    Serial.println(error, 2);
  }
}