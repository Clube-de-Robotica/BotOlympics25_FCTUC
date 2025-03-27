#include <Arduino.h>
#include "FCTUC.h"

FCTUC robot;

void setup() {
    Serial.begin(115200);
    Serial.println("Hello, world!");
    
    robot.begin();
    robot.waitStart();
}

void loop() {
    delay(100);
}