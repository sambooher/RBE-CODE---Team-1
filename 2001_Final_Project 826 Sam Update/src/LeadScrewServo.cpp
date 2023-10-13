#include <Arduino.h>
#include <Romi32U4.h>
#include "LeadScrewServo.h"
#include "Timer.h"
#include <servo32u4.h>

// NOTE: NEED TO MAKE IT STOP AND OPEN IF IT GETS STUCK 
//      should that be with an interrupt? 

// Write any variables here
Servo32U4 jawServo;
Timer printTimer(500);

int linearPotVoltageADC = 500;
int prevPotVoltageADC=500;
int jammedDifferenceADC = 5;


LeadScrewServo::LeadScrewServo() {

}

// Public Functions 
// Function to close the lead screw gripper
void LeadScrewServo::close() {
    linearPotVoltageADC = analogRead(linearPotPin);
    prevPotVoltageADC = linearPotVoltageADC;
    
    // Move Jaw Up
    if (linearPotVoltageADC < jawClosedPotVoltageADC) {
        stop();
        Serial.println("Jaw already too far up.");
    } else {
        // Jaw is in a position where it can move up 
        Serial.println("Jaw moving to CLOSED Position.");
        linearPotVoltageADC = analogRead(linearPotPin);
        jawServo.writeMicroseconds(servoJawUp);
        delay(200);
        while (analogRead(linearPotPin) > jawClosedPotVoltageADC) { // Sets how long the jaw should move up for
            jawServo.writeMicroseconds(servoJawUp);

            prevPotVoltageADC = linearPotVoltageADC;
            delay(350);
            linearPotVoltageADC = analogRead(linearPotPin);

            // Printouts for checking Linear Potentiometer Values
            Serial.print("Current Position: ");
            Serial.print(linearPotVoltageADC);
            Serial.print("         Prev Position: ");
            Serial.print(prevPotVoltageADC);
            Serial.print("         Difference: ");
            Serial.println(prevPotVoltageADC - linearPotVoltageADC);

            // If statement to open the gripper if it is jammed
            if ((prevPotVoltageADC - linearPotVoltageADC) <= jammedDifferenceADC && (prevPotVoltageADC - linearPotVoltageADC) > 0) {
                stop();
                Serial.println("Gripper Jammed: Now opening.");  
                open();
                break;
            }
        }

        stop();
        Serial.println("Jaw in CLOSED Position.");
    }
}

// Function to open the lead screw gripper
void LeadScrewServo::open() {
    linearPotVoltageADC = analogRead(linearPotPin);

    // Move Jaw Down
    if (linearPotVoltageADC > jawOpenPotVoltageADC) {
        stop();
        Serial.println("Jaw already too far down.");
    } else {
        Serial.println("Jaw moving to OPEN Position.");
        while (analogRead(linearPotPin) < jawOpenPotVoltageADC) {
            jawServo.writeMicroseconds(servoJawDown);
            delay(200);
            Serial.println(analogRead(linearPotPin));
        }

        stop();
        Serial.println("Jaw in OPEN Position.");
    }
}

// Function that moves lead screw gripper for a varying effort
void LeadScrewServo::move(int effort){
    jawServo.writeMicroseconds(effort);
}

// Function to stop the lead screw gripper jaw from moving
void LeadScrewServo::stop() {
    // Stop servo
    jawServo.writeMicroseconds(servoStop);
}

// Function that acquires the current position of the lead screw gripper using the potentiometer
int LeadScrewServo::getPosition() {
    int linearPotVoltageADC = analogRead(linearPotPin);
    return linearPotVoltageADC;
}

// Function to initialize the lead screw gripper
void LeadScrewServo::setup() {
    jawServo.attach();
    // attachInterrupt(digitalPinToInterrupt(linearPotPin), isr, CHANGE);
    stop();
}


