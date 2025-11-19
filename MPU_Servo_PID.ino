#include <Wire.h>
#include <ESP32Servo.h>

Servo servo;

#define servo_pin 18
#define MPU_DATA 0x68
#define PWR_MGMT_1 0x6B
#define ACCEL_XOUT_H 0x3B

int16_t accel_x, accel_y, accel_z;
int16_t temp;
int16_t gyro_x, gyro_y, gyro_z;

float accel_x_v, accel_y_v, accel_z_v;
float temp_v;
float gyro_x_v, gyro_y_v, gyro_z_v;

float setpoint = 0;
float feedback = accel_y_v * 90;
float error = setpoint - feedback;
float kp = 0;
float ki = 0;
float kd = 0;

void PID(){

}

void convert_data(){
  accel_x_v = accel_x / 16384.0;
  accel_y_v = accel_y / 16384.0;
  accel_z_v = accel_z / 16384.0;

  temp_v = temp / 340.0 + 36.53;

  gyro_x_v = gyro_x / 131.0;
  gyro_y_v = gyro_y / 131.0;
  gyro_z_v = gyro_z / 131.0;
}

void read_raw_data(){
  Wire.beginTransmission(MPU_DATA);
  Wire.write(ACCEL_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_DATA, 16, false);

  accel_x = Wire.read() << 8 | Wire.read();
  accel_y = Wire.read() << 8 | Wire.read();
  accel_z = Wire.read() << 8 | Wire.read();

  temp = Wire.read() << 8 | Wire.read();

  gyro_x = Wire.read() << 8 | Wire.read();
  gyro_y = Wire.read() << 8 | Wire.read();
  gyro_z = Wire.read() << 8 | Wire.read();
}

void initMPU(){
  Wire.beginTransmission(MPU_DATA);
  Wire.write(PWR_MGMT_1);
  Wire.write(0);
  Wire.endTransmission(true);
}

void initServo(){
  servo.attach(servo_pin);
  servo.write(0);
}
void run_servo(){
  int angle = map(accel_y_v * 90, -90, 90, 0, 180);
  angle = constrain(angle, 0, 180);
  servo.write(angle);
  Serial.print("Servo : ");
  Serial.println(angle);
}

void autoRun(){
  read_raw_data();
  convert_data();
  run_servo();
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Wire.begin();
  initMPU();
  initServo();
}

void loop() {
  // put your main code here, to run repeatedly:
  autoRun();
  delay(10); // this speeds up the simulation
}
