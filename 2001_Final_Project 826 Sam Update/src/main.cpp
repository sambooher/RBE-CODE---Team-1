#include <Arduino.h>
#include <Romi32U4.h>
#include <servo32u4.h>
#include <Chassis.h>
#include "Timer.h"
#include "BlueMotor.h"
#include "LeadScrewServo.h"
#include "ClawServo.h"
#include <IRdecoder.h>
#include <RemoteConstants.h>
#include <Rangefinder.h>

/* 
Fiona notes:
Open and closing gripper - in wrong places
 needs to open earlier
 order incorrect
 wall distance it not the right distance most times-- need to be back some 

 Line 377 - change delay to print timer
*/
/*  WHAT IS LEFT TO DO

  - Put in the Ultrasonic rangefinder values
  - make sure the Arm movements flow correctly 
  - Work on crossing field movements 
  - Handoff but probs not
  - 

*/



// DEFINE VARIABLES:
int irRemotePin = 14;             // IR Remote Pin Number (14)
int rightReflectancePin = 20;        // Right analog input pin // one of these might be wrong
int leftReflectancePin = 22;         // Left analog input pin                     // Left reflectance sensor ADC value                    // Right reflectance sensor ADC value
int RR = 200;   
int LR = 200;
int driveSpeed = 12;
int turnSpeed = 7;
int stopSpeed = 0;
int Kp = 0.08;
int rangeFinderPin = 12;   // NEED TO CHANGE THIS VALUE
float RF = 30;                      // Range finder ADC Value 
int WHITE = 100;
int BLACK = 650;
int jawOpenPotVoltageADC = 950; // Set position to open gripper to
int jawClosedPotVoltageADC = 50; // Set position to close gripper to
int servoJawDown = 1200; // Set effort for closing gripper
int servoJawUp = 1800; // Set effort for opening gripper
int servoClawOpen = 1800; // Value to open claw jaw
int servoClawClose = 1000; // Value to close claw jaw
float distance;
float updatedDistance;

// Encoder positions for the blue motor
int TWENTYFIVE_LIFT_POSITION = 5900; // to pick up plate
int TWENTYFIVE_PICKUP_POSITION = 6400; // to align gripper with plate
int TWENTYFIVE_REPLACE_POSITION = TWENTYFIVE_PICKUP_POSITION;
int TWENTYFIVE_WALL_DIST = 6.4;
// int TWENTYFIVE_PLATFORM_DIST = -20;
int FOURTYFIVE_LIFT_POSITION = 4000; // to pick up plate
int FOURTYFIVE_PICKUP_POSITION = 3000; // to align gripper with plate
int FOURTYFIVE_REPLACE_POSITION = FOURTYFIVE_PICKUP_POSITION;
int FOURTYFIVE_WALL_DIST = 19;
// int FOURTYFIVE_PLATFORM_DIST = -15;
int DOWN_POSITION = 0; // to align gripper with supply box
int SUPPLYBOX_WALL_DIST = 12.7;
// int SUPPLYBOX_PLATFORM_DIST = 6; // with rangefinder, not driveFor
int leadScrewOpenPosition = 950;                                             // NEED TO TRANSFER THESE VALUES HERE vvvvv
int leadScrewClosedPosition = 50;
int leadScewCloseEffort = 1800;
int leadScewOpenEffort = 1200;


int TWENTYFIVE_LIFT_POSITION_2 = -3600; // to pick up plate
int TWENTYFIVE_PICKUP_POSITION_2 = -3250; // to align gripper with plate
int TWENTYFIVE_REPLACE_POSITION_2 = -3500;
int TWENTYFIVE_WALL_DIST_2 = 11;
int FOURTYFIVE_LIFT_POSITION_2 = -3600; // to pick up plate
int FOURTYFIVE_PICKUP_POSITION_2 = -2750; // to align gripper with plate
int FOURTYFIVE_REPLACE_POSITION_2 = -2900;
int FOURTYFIVE_WALL_DIST_2 = 11;
int DOWN_POSITION_2 = 0; // to align gripper with supply box
int SUPPLYBOX_WALL_DIST_2 = 12;
int clawOpenPosition = 1;                                             // NEED TO TRANSFER THESE VALUES HERE vvvvv
int clawClosedPosition = 1;
int clawCloseEffort = 1800;
int clawOpenEffort = 1200;
 
int roofPosition; //TWENTYFIVE_PICKUP_POSITION
int liftPosition; //TWENTYFIVE_PICKUP_POSITION
int replacePosition; //TWENTYFIVE_REPLACE_POSITION
int downPosition = DOWN_POSITION; //DOWN_POSITION
int boxDist = 12;
int roofDist = 19;

// Variables for tracking / moving between states
bool firstBot = true;
int botNumber = 1;    // 1, 2, (3)
int loopType = 2;     // 1 (right/25 roof), 2 (left/45 roof)
int loopCount = 1;    // 1, 2, 3, 4
int currentLoopStep = 1;    // 1, 2, 3, 4
int armLoopStep = 1; // 1, 2, 3, 4
bool closed = true;   // true if gripper is closed, false 
bool pickup = true;  // true if next grip/arm movement is to pick up plate, false
bool initialize = true; // true to start, false after 1st movements
int16_t leftEncoderValue;

Chassis chassis;                  // Chassis
BlueMotor motor;                  // Blue Motor Object / Fourbar
Romi32U4ButtonB buttonB;          
LeadScrewServo leadScrewGripper;  // Continuous Servo gripper
ClawServo clawGripper;            // Non-continuous servo gripper
IRDecoder decoder(irRemotePin);   // IR Remote Object
Rangefinder rangefinder(17, 12);

// STATE MACHINE
enum stateChoices {START, SECONDSTART, TURNTOLINE, LINEFOLLOWCROSSLINE, LINEFOLLOWDIST, TURN, DRIVEFORWARDCOUNT, DRIVEFORWARD, BACKUP, MOVEARM, OPENGRIPPER, CLOSEGRIPPER, STOP} romiState, nextRomiState;

/*
  - STOP (Emergency or otherwise)
  - START (initialize gripper and arm to down and closed as zero) 
      and move them up so we drive in (roof pickup position)
  
CHASSIS MOVEMENTS
  - TURNTOLINE
    * Direction (L or R)
    * Degree (90 or 180)
  - LINEFOLLOW
    * Stop when cross line
    * stop when hit a distance from wall
  ? TURN 
  ? DRIVEFORWARD

  ? DRIVEFORWARD

ARM MOVEMENTS
  - ARMMOVETO
    * Position to move to 
  ? May need something else to control order of movements (IDK)

GRIPPER MOVEMENTS
  - OPENGRIPPER
  - CLOSEGRIPPER
*/

void setEffortWithoutDBFunction();
void gripperControl();

void setup() {
  // Setup code to run once:
  Serial.begin(9600);
  chassis.init(); // initialize chassis
  motor.setup(); // initiaize motor
  decoder.init(); // initialize IR Remote
  rangefinder.init();
  
  // **********************************************************
  // This is the ONLY value we need to change in between robots (fingers crossed :)
  firstBot = true;

  pinMode(leftReflectancePin, INPUT);
  pinMode(rightReflectancePin, INPUT);

  leadScrewGripper.setup();  // initialize lead screw gripper
  clawGripper.setup(); // initialize claw gripper
  motor.setEffort(0); // set starting motor effort to 0
  if (!firstBot) {
     // setting the initialize state for the second robot
    romiState = SECONDSTART;  
  } else {
    // setting the beginning initialize state
    romiState = START;
  }

  // buttonB.waitForButton();
  // Serial.println("Button B Pressed.");

  delay(1500);
}

// Function to handle button presses from the IR Remote
void handleKeyPress(int keyPress) {
  if (keyPress == remotePlayPause)
  { // Emergency Stop Button
    nextRomiState = romiState;
    romiState = STOP;
    Serial.println("Play/Paused Pressed");
  } 

  if (keyPress == remoteUp) 
  { // Proceed button
    romiState = nextRomiState;
    Serial.println("Up Button pressed");
  }
}

void loop() {
   // TESTING LOOP
  // while(true) {
  //   while (buttonB.isPressed()) {
  //     // motor.setEffort(-300);
  //     // Serial.println("Button press");
  //     RR = analogRead(rightReflectancePin);
  //     LR = analogRead(leftReflectancePin);
  //     Serial.print("RR: ");
  //     Serial.print(RR);
  //     Serial.print(" LR: ");
  //     Serial.println(LR);
  //     delay(300);
  //   }
  //   motor.setEffort(0);
  //   chassis.setWheelSpeeds(0,0);
  // }

  // Main code to run repeatedly: 
  // CODE FOR THE IR REMOTE HERE

  int keyPress = decoder.getKeyCode(); //true allows the key to repeat if held down
  if(keyPress >= 0) handleKeyPress(keyPress);

  // State machine for switching desired plate pickup and placement
  switch (romiState) {
  case START: 
    Serial.println("CASE: START");

    // ASSIGN THE VALUES FOR THE VARIABLES BASED ON ROBOT
    if (botNumber == 1) {
      if (loopType == 1) {
        roofPosition = TWENTYFIVE_PICKUP_POSITION;
        liftPosition = TWENTYFIVE_LIFT_POSITION;
        replacePosition = TWENTYFIVE_REPLACE_POSITION;
        downPosition = DOWN_POSITION;
        // define wall distance & box dist
        roofDist = TWENTYFIVE_WALL_DIST;
        boxDist = SUPPLYBOX_WALL_DIST;
      } else {
        roofPosition = FOURTYFIVE_PICKUP_POSITION;
        liftPosition = FOURTYFIVE_LIFT_POSITION;
        replacePosition = FOURTYFIVE_REPLACE_POSITION;
        downPosition = DOWN_POSITION;
        roofDist = FOURTYFIVE_WALL_DIST;
        boxDist = SUPPLYBOX_WALL_DIST;
      }
    } else if (botNumber == 2) {
      if (loopType == 1) {
        roofPosition = TWENTYFIVE_PICKUP_POSITION_2;
        liftPosition = TWENTYFIVE_LIFT_POSITION_2;
        replacePosition = TWENTYFIVE_REPLACE_POSITION_2;
        downPosition = DOWN_POSITION_2;
        roofDist = TWENTYFIVE_WALL_DIST_2;
        boxDist = SUPPLYBOX_WALL_DIST_2;
      } else {
        roofPosition = FOURTYFIVE_PICKUP_POSITION_2;
        liftPosition = FOURTYFIVE_LIFT_POSITION_2;
        replacePosition = FOURTYFIVE_REPLACE_POSITION_2;
        downPosition = DOWN_POSITION_2;
        roofDist = FOURTYFIVE_WALL_DIST_2;
        boxDist = SUPPLYBOX_WALL_DIST_2;
      }
    }

      nextRomiState = OPENGRIPPER; //MOVEARM
      romiState = STOP;
    break;
  
  case SECONDSTART:
    Serial.println("CASE: SECOND START");
    // incriment loop type (switch side of field & roof angle)
    loopType = (loopType % 2) + 1;

    // ASSIGN THE VALUES FOR THE VARIABLES BASED ON ROBOT
    if (botNumber == 1) {
      if (loopType == 1) {
        roofPosition = TWENTYFIVE_PICKUP_POSITION;
        liftPosition = TWENTYFIVE_PICKUP_POSITION;
        replacePosition = TWENTYFIVE_REPLACE_POSITION;
        downPosition = DOWN_POSITION;
        // define wall distance & box dist
        roofDist = TWENTYFIVE_WALL_DIST;
        boxDist = SUPPLYBOX_WALL_DIST;
      } else {
        roofPosition = FOURTYFIVE_PICKUP_POSITION;
        liftPosition = FOURTYFIVE_PICKUP_POSITION;
        replacePosition = FOURTYFIVE_REPLACE_POSITION;
        downPosition = DOWN_POSITION;
        roofDist = FOURTYFIVE_WALL_DIST;
        boxDist = SUPPLYBOX_WALL_DIST;
      }
    } else if (botNumber == 2) {
      if (loopType == 1) {
        roofPosition = TWENTYFIVE_PICKUP_POSITION_2;
        liftPosition = TWENTYFIVE_PICKUP_POSITION_2;
        replacePosition = TWENTYFIVE_REPLACE_POSITION_2;
        downPosition = DOWN_POSITION_2;
        roofDist = TWENTYFIVE_WALL_DIST_2;
        boxDist = SUPPLYBOX_WALL_DIST_2;
      } else {
        roofPosition = FOURTYFIVE_PICKUP_POSITION_2;
        liftPosition = FOURTYFIVE_PICKUP_POSITION_2;
        replacePosition = FOURTYFIVE_REPLACE_POSITION_2;
        downPosition = DOWN_POSITION_2;
        roofDist = FOURTYFIVE_WALL_DIST_2;
        boxDist = SUPPLYBOX_WALL_DIST_2;
      }
    }

    // resume where last bot left off
    loopCount = 3;
    currentLoopStep = 1;
    nextRomiState = LINEFOLLOWCROSSLINE;
    romiState = STOP;
    break;
  
  case TURNTOLINE:
    Serial.println("CASE: TURN TO LINE");

    // Determine which direction to turn
    Serial.print("Loop step: ");
    Serial.print(currentLoopStep);
    Serial.print(" Loop type: ");
    Serial.print(loopType);
    Serial.print(" Sum: ");
    Serial.println(currentLoopStep + loopType);
    
    if ((currentLoopStep + loopType) % 3 == 0) {
      // Turn right
      chassis.setWheelSpeeds(turnSpeed, -turnSpeed);
    } else {
      // Turn left 
      chassis.setWheelSpeeds(-turnSpeed, turnSpeed);
    }

    RR = analogRead(rightReflectancePin);
    LR = analogRead(leftReflectancePin);
    Serial.print("RR: ");
    Serial.print(RR);
    Serial.print(" LR: ");
    Serial.println(LR);

    // Stop turning once reaching line
    if (RR > 500 || LR > 500) {   //Check to see if you have picked up the line
      if (currentLoopStep % 2 == 0) {
        nextRomiState = LINEFOLLOWCROSSLINE;
      } else {
        nextRomiState = LINEFOLLOWDIST;
      }

      romiState = STOP;
    }
    break;
  
  case LINEFOLLOWCROSSLINE:
    Serial.println("CASE: LINEFOLLOWCROSSLINE");

    RR = analogRead(rightReflectancePin);          
    LR = analogRead(leftReflectancePin);
    chassis.setTwist(driveSpeed,Kp*(RR-LR));

    if ((RR > 500 && LR > 500)) {
      // stop driving
      Serial.println("Crossed line");
      chassis.setWheelSpeeds(stopSpeed, stopSpeed);
      nextRomiState = DRIVEFORWARDCOUNT;
      romiState = STOP;
      leftEncoderValue = chassis.getLeftEncoderCount(true);
    }

    break;
  
  case LINEFOLLOWDIST:
    Serial.println("CASE: LINEFOLLOWDIST");

    RR = analogRead(rightReflectancePin);          
    LR = analogRead(leftReflectancePin);
    for (int i = 0; i < 10; i++) {
      rangefinder.getDistance();
      delay(100); //Fiona change to print timer :)
    }
    // Serial.print("Total distances = ");
    // Serial.println(distance);
    // updatedDistance = (distance / 10.0);
    // Serial.print("Average distance = ");
    // Serial.println(updatedDistance);
    // RF = updatedDistance;
    RF = rangefinder.getDistance();

    chassis.setTwist(driveSpeed,Kp*(RR-LR));
    Serial.print(" RF: ");
    Serial.println(RF);
    if ((RF <= boxDist && currentLoopStep == 1) || (RF <= roofDist && currentLoopStep == 3)) {   
      Serial.println("Wall Distance hit");
      // stop driving
      chassis.setWheelSpeeds(stopSpeed, stopSpeed);
      //incriment romiState
      if (closed) {
        nextRomiState = CLOSEGRIPPER; //MOVEARM
        romiState = STOP; 
      } else {
        nextRomiState = MOVEARM; //CLOSEGRIPPER
        romiState = STOP;
      }

      // Incriment loop step and count
      if (currentLoopStep == 4) {
        // Change loop type (if ready to cross to other side of the field)
        if (loopCount == 2) loopType = 2;                 // *** NEED TO CHANGE ALL THE VALUES TO THE SIDE 2 NUMBERS TOO ***

        // Reset step to 1, increase loop count
        currentLoopStep = 1;
        loopCount++;
      } else {
        // Incriment loop step number 
        currentLoopStep++;
      }
    }
    break;
  
  // START THE TURN WITH 
  case TURN:
    Serial.println("TURN");

    // Determine turn direction 
    if (loopCount == 3 && firstBot) {
      // turn 90 degrees to cross the field
      Serial.println("Ready to cross the field");
      // determine direction with math? 
      // how to make non-blocking
      
      // change state:
      nextRomiState = DRIVEFORWARD;
      romiState = STOP; 

    } else if ((currentLoopStep + loopType) % 3 == 0) {
      // Turn right
      chassis.turnFor(-45, turnSpeed*2, true);
      romiState = TURNTOLINE;
    } else {
      // Turn left 
      chassis.turnFor(45, turnSpeed*2, true);
      romiState = TURNTOLINE;
    }
    break;
    
  case DRIVEFORWARDCOUNT: 
    Serial.println("CASE: DRIVE FORWARD COUNT");

    chassis.setWheelSpeeds(driveSpeed, driveSpeed);  //Move forward at half speed for 300 counts (1440 per rev).
    //This will be about 1/5 of a revolution.  Do this so that you don't pick up the end of the line when you start the turn.
    leftEncoderValue = chassis.getLeftEncoderCount();
    if(leftEncoderValue > 450){
      chassis.setWheelSpeeds(stopSpeed, stopSpeed);
      nextRomiState = TURN;
      romiState = STOP;
      Serial.println("Stop");
      Serial.print("leftEncoderValue =  "); //Check the encoder count
      Serial.println(leftEncoderValue);
      Serial.println("End of DRIVEFORWARDCOUNT");

      // incriment loop step and count
      if (currentLoopStep == 4) {
        currentLoopStep = 0;
        loopCount++;
      } else {
        currentLoopStep++;
      }

      nextRomiState = TURN;
      romiState = STOP;
    }
    break;
  
  case DRIVEFORWARD:
    // For crossing the field
    Serial.println("CASE: DRIVEFORWARD");

    RR = analogRead(rightReflectancePin);
    LR = analogRead(leftReflectancePin);

    chassis.setWheelSpeeds(driveSpeed, driveSpeed);

    if ((RR > 500 && LR > 500)) {
      chassis.setWheelSpeeds(stopSpeed, stopSpeed);
      nextRomiState = DRIVEFORWARDCOUNT;
      romiState = STOP;
    }
    // if (nextRomiState = DRIVEFORWARDCOUNT) {
    //   nextRomiState = TURNTOLINE;
    //   romiState = STOP;
    // }
    // until hit the line 
    // NEXT STATE -> turn to line  
    break;
  
  // Similar to Drive forward count
  case BACKUP:
    Serial.println("CASE: BACKUP");

    chassis.setWheelSpeeds(-driveSpeed, -driveSpeed);  //Move backwards for 300 counts (1440 per rev).
    leftEncoderValue = chassis.getLeftEncoderCount();
    if(leftEncoderValue < -300){
      chassis.setWheelSpeeds(stopSpeed, stopSpeed);
      // NEXT STATE
      nextRomiState = TURN;
      romiState = STOP;
      Serial.println("Stop");
      Serial.print("leftEncoderValue =  "); //Check the encoder count
      Serial.println(leftEncoderValue);
      Serial.println("End of BACKUP");

      // incriment loop step and count
      if (currentLoopStep == 4) {
        currentLoopStep = 0;
        loopCount++;
      } else {
        currentLoopStep++;
      }

      nextRomiState = TURN;
      romiState = STOP;
    }
    break;

  case MOVEARM:
    Serial.println("CASE: MOVE ARM");
    int armPosition;
    // Based on arm loop step define which position to move to
    if (armLoopStep == 2 || armLoopStep == 4) {
      armPosition = liftPosition; 
    } else if (armLoopStep == 3) {
      armPosition = downPosition;
    } else if (armLoopStep == 5) {
      armPosition = replacePosition;
    } else {
      armPosition = roofPosition;
    }
      
    motor.setEffortWithoutDB((armPosition - motor.getPosition()) * 0.3);

    if (abs((armPosition - motor.getPosition())) < 1) {
      motor.stop(); 

      // incriment arm step counter
      if (armLoopStep == 5) {
        armLoopStep = 0;
      }
      armLoopStep++;

      // choose next state based on current position 
      if (armLoopStep == 2 || armLoopStep == 4) { // 2 4
        nextRomiState = BACKUP; 
      } else {
        nextRomiState = CLOSEGRIPPER; //OPENGRIPPER
      }
      romiState = STOP;
    }
    break;

  case OPENGRIPPER:
    Serial.println("CASE: OPEN GRIPPER");
   
    if (botNumber == 1) {
      // LEAD SCREW GRIPPER
      leadScrewGripper.move(leadScewOpenEffort);
      int position = leadScrewGripper.getPosition();

      if (position >= leadScrewOpenPosition) {
        leadScrewGripper.stop();
        closed = false;

        // decide next state
        if (armLoopStep == 4) {
          nextRomiState = CLOSEGRIPPER;
          romiState = STOP;
        } else {
          nextRomiState = MOVEARM; //BACKUP
          romiState = STOP;
        }
      }
    } else {
      // CLAW GRIPPER
      clawGripper.move(clawOpenEffort);
      int position = clawGripper.getPosition();

      if (position >= clawOpenPosition) {
        clawGripper.stop();
        closed = false;

        // decide next state
        if (armLoopStep == 4) {
          nextRomiState = CLOSEGRIPPER;
          romiState = STOP;
        } else {
          nextRomiState = BACKUP;
          romiState = STOP;
        }
      }
    }
    break;  
  
  case CLOSEGRIPPER:
    Serial.println("CASE: CLOSE GRIPPER");

    if (botNumber == 1) {
      // LEAD SCREW GRIPPER
      leadScrewGripper.move(leadScewCloseEffort);
      int position = leadScrewGripper.getPosition();

      if (position <= leadScrewClosedPosition) {
        leadScrewGripper.stop();
        closed = true;

        // NEXT STATE CHOICE -> move arm (lift position)
        nextRomiState = TURN; //MOVEARM---> I think we need to do else and else if statements
        romiState = STOP;
      }
    } else {
      // CLAW GRIPPER
      clawGripper.move(clawCloseEffort);
      int position = clawGripper.getPosition();

      if (position <= clawClosedPosition) {
        clawGripper.stop();
        closed = true;

        // NEXT STATE CHOICE -> move arm (lift position)
        nextRomiState = MOVEARM; 
        romiState = STOP;
      }
    }
    break;
  
  case STOP:
    // Serial.println("CASE: STOP");
    if (botNumber == 1) {
      leadScrewGripper.stop();
    } else {
      clawGripper.stop();
    }
    motor.stop();
    chassis.setWheelSpeeds(stopSpeed, stopSpeed); 
    break;

  } // END SWITCH
} // END LOOP 
