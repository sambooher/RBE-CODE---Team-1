#pragma once

// Class for the claw gripper
class ClawServo 
{
public:
    ClawServo();
    void open();
    void close();
    void setup();
    void stop();
    void move(int effort);
    int getPosition();

private:
    int servoStop = 1505; // Value to stop claw jaw from moving
    int redPin = 18;
    int servoClawOpen = 1800; // Value to open claw jaw
    int servoClawClose = 1000; // Value to close claw jaw
};