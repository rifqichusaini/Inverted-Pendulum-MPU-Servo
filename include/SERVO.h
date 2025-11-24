#pragma once
#include <Arduino.h>

class SERVO {
private:
  int servoPin;
  int pwmChannel;
  int minPulse;
  int maxPulse;

public:
  SERVO(){
    pwmChannel = 5;   // default channel
    minPulse = 500;   // default 0°
    maxPulse = 2500;  // default 180°
  }

  void begin(){
    ledcSetup(pwmChannel, 50, 16);    // 50 Hz, resolusi 16-bit
    ledcAttachPin(servoPin, pwmChannel);
  }

  void attachPin(int pin){
    servoPin = pin;
  }

  void writeMicroseconds(int us){
    us = constrain(us, minPulse, maxPulse);  // biar aman

    // periode 50 Hz = 20.000 us
    int duty = map(us, 0, 20000, 0, 65535);
    ledcWrite(pwmChannel, duty);
  }

  void writeAngle(float angle){
    angle = constrain(angle, 0, 180);
    int us = map(angle, 0, 180, minPulse, maxPulse);
    writeMicroseconds(us);
  }

  void initServo(int servo_pin){
    attachPin(servo_pin);
    begin();
  }
};
