#include <Arduino.h>
#include <Romi32U4.h>
#include "ClawServo.h"
#include "Timer.h"
#include <servo32u4.h>

// Define any variables here
Servo32U4 clawServo;

ClawServo::ClawServo() {}

void ClawServo::close() 
{
    clawServo.writeMicroseconds(servoClawClose);
    delay(4000);
    stop();
}

void ClawServo::open() 
{
    clawServo.writeMicroseconds(servoClawOpen);
    delay(2000);
    stop();
}

int ClawServo::getPosition()
{
    int currentPosition = analogRead(redPin);
    return currentPosition;
}

void ClawServo::setup() 
{
    clawServo.attach();
    stop();
}

void ClawServo::stop() 
{
    clawServo.writeMicroseconds(servoStop);
}