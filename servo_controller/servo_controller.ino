#include <ESP32Servo.h>
#include <Wire.h>

#define SERVO 23
#define PIN 25
#define POT A0

void adjust() {
  int input = analogRead(POT);
  double maxInput = 4095.0;
  
  int color = 0x0000FF;

  int rotation = (int)((input / maxInput) * 360);
  return rotation;
}

Servo servo
void setup() {
    // init serial connection
    Serial.begin(115200);
    
    servo.attach(SERVO1);
}

void loop() {
    servo.write(adjust())
}