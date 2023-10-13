#include <Arduino.h>
#include <Romi32U4.h>
#include <Wire.h>
#include "BlueMotor.h"

// DEFINE VARIABLES
long oldValue = 0;
long newValue;
unsigned time = 0;
long count;
float kp = 0.3; 


// PUBLIC FUNCTIONS
BlueMotor::BlueMotor() {}

// Function to set the motor's effort
void BlueMotor::setEffort(int effort)
{
    if (effort < 0)
    {
        setEffort(-effort, true);
    }
    else
    {
        setEffort(effort, false);
    }
}

// Function to evaluate the motor's effort when not using dead-band
float BlueMotor::setEffortWithoutDB(int effort) 
{
    // Values based off Lab 4: Question 3a images
    int motorPosDB = 175;
    int motorNegDB = -265;

    int adjustedEffort;
    float diff = 400;
    
    // Determine if the inputted effort is positive or negative direction
    if (effort >= 0) {
        diff = (400.0 - motorPosDB) / 400.0;
        adjustedEffort = (effort * diff) + motorPosDB;
    } else {
        diff = (400.0 + motorNegDB) / 400.0;
        adjustedEffort = (effort * diff) + motorNegDB;
    }

    setEffort(adjustedEffort);
    return adjustedEffort;
}

// Function to record positive values of effort when using dead-band
void BlueMotor::deadBand() 
{
    // Reset Count for printout readability
    count = 0;

    // Incriment motor effort from 0 to 400
    for (int i = 0; 5*i < 400; i++) {
        setEffort(i * 5);
        Serial.print("Motor Effort: ");
        Serial.println(i*5);
        Serial.print("Encoder Count: ");
        Serial.println(getPosition());
        delay(200);
    }

    setEffort(0);
}

// Function to record negative values of effort when using dead-band
void BlueMotor::deadBandReverse() 
{
    // Reset Count for printout readability
    count = 0;

    // Incriment motor effort from 0 to -400 (Negative Direction)
    for (int i = 0; 5*i <= 400; i++) {
        setEffort(-(i * 5));
        Serial.print("Motor Effort: ");
        Serial.println(i*-5);
        Serial.print("Encoder Count: ");
        Serial.println(getPosition());
        delay(200);
    }

    setEffort(0);
}

// Function that moves motor to certain encoder position depending on the set target encoder position
bool BlueMotor::moveTo(long target)  //Move to this encoder position within the specified
{                                    //tolerance in the header file using proportional control 
    long currentPosition = getPosition();
    //Serial.println(currentPosition);
    long difference = target - currentPosition;
    // bool direction = difference < 0;
    if (abs(difference) < tolerance) {
        // clockwise if difference is positive
        // cc if the difference is negative
        // bool direction = difference > 0;
        setEffortWithoutDB(difference * kp);
        currentPosition = getPosition();
        difference = target - currentPosition;
        // Serial.println(currentPosition);
        return true;
    } else {
        return false;
    }

    Serial.println(currentPosition);
    setEffort(0);
}

// Function that acquires the current encoder position of the blue motor
long BlueMotor::getPosition()
{
    long tempCount = 0;
    noInterrupts();
    tempCount = count;
    interrupts();
    return tempCount;
}

void BlueMotor::stop() {
    setEffort(0);
}

// Function that sets the current position of the motor to an encoder count of zero
void BlueMotor::reset()
{
    setEffort(0);
    delay(200);
    moveTo(0);
    noInterrupts();
    count = 0;
    interrupts();
}

// Function that initializes the blue motor
void BlueMotor::setup()
{
    pinMode(PWMOutPin, OUTPUT);
    pinMode(AIN2, OUTPUT);
    pinMode(AIN1, OUTPUT);
    pinMode(ENCA, INPUT);
    pinMode(ENCB, INPUT);
    TCCR1A = 0xA8; //0b10101000; //gcl: added OCR1C for adding a third PWM on pin 11
    TCCR1B = 0x11; //0b00010001;
    ICR1 = 400;
    OCR1C = 0;

    attachInterrupt(digitalPinToInterrupt(ENCA), isrA, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCB), isrB, CHANGE);
    reset();
}





// PRIVATE FUNCTIONS
void BlueMotor::setEffort(int effort, bool clockwise)
{
    // clockwise = increase count
    // counter clockwise = decrease count
    if (clockwise)
    {
        digitalWrite(AIN1, HIGH);
        digitalWrite(AIN2, LOW);
    }
    else
    {
        digitalWrite(AIN1, LOW);
        digitalWrite(AIN2, HIGH);
    }
    OCR1C = constrain(effort, 0, 400);
}

// Interrupt A
void BlueMotor::isrA()
{
    if (digitalRead(ENCB) == digitalRead(ENCA)) {
        count++;
    } else {
        count--;
    }
}

// Interrupt B
void BlueMotor::isrB()
{
    if (digitalRead(ENCA) == digitalRead(ENCB)) {
        count--;
    } else {
        count++;
    }
}