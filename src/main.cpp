#include <Arduino.h>
#include <Wire.h>
#include <MPU6050.h>
#include <SERVO.h>

#define servo_pin 18

SERVO servo;
MPU6050 mpu;

void initServo(){
  servo.attachPin(servo_pin);
  servo.begin();
  servo.writeAngle(0);
}

void run_servo(){
  int angle = map(mpu.get_accel_y() * 90, -90, 90, 0, 180);
  angle = constrain(angle, 0, 180);
  servo.writeAngle(angle);
  Serial.print("Servo : ");
  Serial.println(angle);
}

void autoRun(){
  mpu.read_raw_data();
  mpu.convert_data();
  mpu.print_data();
  run_servo();
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Wire.begin();
  mpu.initMPU();
  initServo();
}

void loop() {
  // put your main code here, to run repeatedly:
  autoRun();
  delay(10); // this speeds up the simulation
}



// void setup() {
//     servo.begin();
//     Serial.begin(9600);
// }

// void loop() {
//     servo.writeAngle(0);
//     Serial.println(0);
//     delay(1000);

//     servo.writeAngle(180);
//     Serial.println(180);
//     delay(1000);
// }
