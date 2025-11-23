#include <Arduino.h>
#include <Wire.h>
#include <MPU6050.h>
#include <Servo.h>

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

void moveServo(float correction) {
  int servoAngle = 90 - correction;
  servoAngle = constrain(servoAngle, 0, 180);
  balanceServo.writeAngle(servoAngle);
}

float getAngle() {
  static float angleFiltered = 0;

  float accelX = mpu.get_accel_x();
  float accelY = mpu.get_accel_y();
  float accelZ = mpu.get_accel_z();

  float accelAngle = atan2(accelY, accelZ) * 180.0 / PI;

  angleFiltered = 0.90 * (angleFiltered) + 0.12 * accelAngle;
  // angleFiltered = 0.90 * (angleFiltered + gyroY * dt) + 0.12 * accelAngle;
  
  return angleFiltered;
}

void setup() {
  Serial.begin(115200);
  Wire.begin();

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