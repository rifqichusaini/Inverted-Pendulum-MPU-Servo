#include <Servo.h>

Servo servo;



float setpoint = 90;       // posisi target servo
float Kp = 0;              
float Ki = 0;
float Kd = 0;

float Ku = 0;              // ultimate gain (isi setelah ditemukan)
float Tu = 0;              // ultimate period (isi setelah diukur)

float error, lastError = 0;
float integral = 0;
unsigned long lastTime = 0;

// mode: 0 = cari Ku (P-only), 1 = PID ZN
int mode = 0;

// Baca sensor (pakai potensiometer di pin A0)
int readPosition() {
  int raw = analogRead(A0);
  return map(raw, 0, 1023, 0, 180);
}

// Hitung PID
float computePID(float error) {
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;
  lastTime = now;

  integral += error * dt;
  float derivative = (error - lastError) / dt;
  lastError = error;

  float output = Kp * error + Ki * integral + Kd * derivative;
  return output;
}

void setup() {
  Serial.begin(115200);
  servo.attach(9);

  lastTime = millis();

  Serial.println("=== PID Tuning – Ziegler Nichols ===");
  Serial.println("Mode 0: Cari Ku (P-only)");
  Serial.println("Mode 1: PID ZN");
}

void loop() {

  int pos = readPosition();
  error = setpoint - pos;

  float output;

  if (mode == 0) {
    // === MODE TUNING P-ONLY (CARI KU) ===
    output = Kp * error;  // hanya aksi P
    Serial.print("Error:");
    Serial.print(error);
    Serial.print(",Output:");
    Serial.println(output);

  } else {
    // === MODE PID ZIEGLER-NICHOLS ===
    output = computePID(error);
  }

  // Konversi output → posisi servo
  int pwm = constrain(pos + output, 0, 180);
  servo.write(pwm);

  delay(10);
}

// ----------------------
// Fungsi Hitung Z-N PID
// ----------------------
void calculateZN() {
  // Penuhi dulu nilai Ku & Tu
  Kp = 0.6 * Ku;
  Ki = 2 * Kp / Tu;
  Kd = Kp * Tu / 8;

  Serial.println("=== Ziegler-Nichols PID Values ===");
  Serial.print("Kp = "); Serial.println(Kp);
  Serial.print("Ki = "); Serial.println(Ki);
  Serial.print("Kd = "); Serial.println(Kd);
}
