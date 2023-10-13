#pragma once

// Class for the lead screw gripper/bottom out linear slide gripper
class LeadScrewServo
{
public:
    LeadScrewServo();
    void close();
    void open();
    void move(int effort);
    void stop();
    int getPosition();
    void setup();

private:
    // int servoPin = 5;
    int linearPotPin = 18; 
    // NOTE: the servo runs from 1000 (full speed one direction) to 2000 (full speed the other direction)
    // where 1500 is stopped in the middle
    int servoStop = 1495; // Value to stop lead screw gripper from moving
    int servoJawDown = 1200; // Set effort for closing gripper
    int servoJawUp = 1800; // Set effort for opening gripper
    int printDelay = 500;
    int jawOpenPotVoltageADC = 950; // Set position to open gripper to
    int jawClosedPotVoltageADC = 200; // Set position to close gripper to
};