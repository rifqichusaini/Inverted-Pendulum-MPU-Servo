#include <Arduino.h>
#include <Wire.h>
#include <MPU6050.h>
#include <ESP32Servo.h>

MPU6050 mpu;
Servo balanceServo;

// PID Variables - Tuned for servo
float Kp = 15.0;    
float Ki = 0.0;     
float Kd = 105.0;     

float angle = 0.0;
float setpoint = 0.0;  // Target: upright position

float error, previousError = 0;
float integral = 0;
float derivative;
float output;

// Timing
unsigned long lastTime;
float dt;

// Servo Pin
#define SERVO_PIN 18

// ---- SERVO FUNCTION ----
void moveServo(float correction) {
  // REVERSE THE CORRECTION DIRECTION
  int servoAngle = 90 - correction;  // MINUS instead of PLUS
  
  // Full range 0-180° (no limits as requested)
  servoAngle = constrain(servoAngle, 0, 180);
  
  balanceServo.write(servoAngle);
}

// ---- MPU ANGLE CALCULATION: Complementary filter ----
float getAngle() {
  static float angleFiltered = 0;
  
  // Read raw data
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Convert to G and degrees/s
  float accelX = ax / 16384.0;
  float accelY = ay / 16384.0; 
  float accelZ = az / 16384.0;
  float gyroY = gy / 131.0;  // degrees per second

  // Calculate angle from accelerometer (ROLL)
  float accelAngle = atan2(accelY, accelZ) * 180.0 / PI;

  // Complementary filter
  angleFiltered = 0.90 * (angleFiltered ) + 0.12 * accelAngle;
  // angleFiltered = 0.90 * (angleFiltered + gyroY * dt) + 0.12 * accelAngle;
  
  return angleFiltered;
}

void setup() {
  Serial.begin(115200);
  Wire.begin();

  // Initialize MPU6050
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 not connected!");
    while (1);
  }

  // Attach servo
  balanceServo.attach(SERVO_PIN);
  balanceServo.write(90);  // Neutral position

  lastTime = micros();
  
  Serial.println("System Ready - Servo Balance Control");
  Serial.println("Direction: REVERSED for counter-balancing");
  delay(2000);
}

void loop() {
  // ---- READ ANGLE FROM MPU ----
  angle = getAngle();

  // ---- PID COMPUTATION ----
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



  // ---- SERVO OUTPUT ----
  moveServo(output);

  // Debug
  Serial.print("Angle: "); Serial.print(angle);
  Serial.print(" | Error: "); Serial.print(error);
  Serial.print(" | Output: "); Serial.print(output);
  Serial.print(" | Servo: "); Serial.println(90 - output); // Updated for display
}