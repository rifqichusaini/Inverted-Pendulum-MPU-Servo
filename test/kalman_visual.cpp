#include <Arduino.h>
#include <Wire.h>
#include <MPU6050.h>
#include <kalman.h>

MPU6050 mpu;

Kalman1D kalmanX(0.0005, 0.02, 0.0); // tunable
Kalman1D kalmanY(0.0005, 0.02, 0.0); // tunable

const unsigned long SAMPLE_INTERVAL_MS = 10; // ~100 Hz
unsigned long lastMillis = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  delay(100);
  mpu.initMPU();
  delay(100);
  Serial.println("READY"); // tanda ready
}

void loop() {
  unsigned long now = millis();
  if (now - lastMillis >= SAMPLE_INTERVAL_MS) {
    lastMillis = now;

    // baca sensor
    mpu.read_raw_data();
    mpu.convert_data();

    float rawX = mpu.accel_x_v;
    float rawY = mpu.accel_y_v;

    float kx = kalmanX.update(rawX);
    float ky = kalmanY.update(rawY);

    // Format: RAWX,KALX,RAWY,KALY
    // Contoh: 0.12345,0.12000,0.04567,0.04400
    Serial.print(rawX, 5);
    Serial.print(",");
    Serial.print(kx, 5);
    Serial.print(",");
    Serial.print(rawY, 5);
    Serial.print(",");
    Serial.println(ky, 5);
  }

  // optional: do nothing else
}