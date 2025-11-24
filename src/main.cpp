#include <Arduino.h>
#include <Wire.h>
#include <MPU6050.h>
#include <Servo.h>
#include <kalman.h>

MPU6050 mpu;
SERVO balanceServo;

float Kp = 15.0;    
float Ki = 0.0;     
float Kd = 105.0;     

float angle = 0.0;
float setpoint = 0.0;

float error, previousError = 0;
float integral = 0;
float derivative;
float output;

unsigned long lastTime;
float dt;

#define SERVO_PIN 18

Kalman1D kalmanX(0.05, 0.02, 0.0); // tunable
Kalman1D kalmanY(0.05, 0.02, 0.0); // tunable

void moveServo(float correction) {
  int servoAngle = 90 - correction;
  servoAngle = constrain(servoAngle, 0, 180);
  balanceServo.writeAngle(servoAngle);
}

float getAngle() {
  static float angleFiltered = 0;
  mpu.read_raw_data();
  mpu.convert_data();

  float accelY = mpu.get_accel_y();
  float accelZ = mpu.get_accel_z();
  
  // pake z sama y karna rotasi atas-bawah (z) dan kiri-kanan (y)
  float kz = kalmanX.update(accelZ);
  float ky = kalmanY.update(accelY);

  float accelAngle = atan2(ky, kz) * 180.0 / PI;

  // nilai alfa 90%
  angleFiltered = 0.90 * (angleFiltered) + 0.12 * accelAngle;
  
  return angleFiltered;
}

void setup() {
  Serial.begin(115200);
  Wire.begin();

  mpu.initMPU();
  balanceServo.initServo(SERVO_PIN);
  balanceServo.writeAngle(90);

  lastTime = micros();
  
  delay(2000);
}

void loop() {
  angle = getAngle();

  unsigned long now = micros();
  dt = (now - lastTime) / 1000000.0;
  lastTime = now;

  error = setpoint - angle;
  integral += error * dt;
  derivative = (error - previousError) / dt;
  previousError = error;

  if(abs(error) < 3.0){
    output = 0;
  } else{
    output = Kp * error + Ki * integral + Kd * derivative;
  }

  moveServo(output);

  Serial.print("Angle: "); Serial.print(angle);
  Serial.print(" | Error: "); Serial.print(error);
  Serial.print(" | Output: "); Serial.print(output);
  Serial.print(" | Servo: "); Serial.println(90 - output); // Updated for display
}