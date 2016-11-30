// Zack Bright        - zbright  _ mit _ edu,    Sept 2015
// Daniel J. Gonzalez - dgonz    _ mit _ edu,    Sept 2015
// Fangzhou Xia       - xiafz    _ mit _ edu,    Sept 2015
// Peter KT Yu        - peterkty _ mit _ edu,    Sept 2016
// Ryan Fish          - fishr    _ mit _ edu,    Sept 2016

#include "Arduino.h"
#include "helper.h"
#include "ServoTimer2.h"

#define servoWristPin  3
#define wristBumperPin      2
#define userPin     A3
#define bumperPin     A2
ServoTimer2 servoWrist;

EncoderMeasurement  encoder(26);      // encoder handler class, set the motor type 53 or 26 here
RobotPose           robotPose;        // robot position and orientation calculation class
PIController        wheelVelCtrl;     // velocity PI controller class
SerialComm          serialComm;       // serial communication class
unsigned long       prevTime = 0;

boolean usePathPlanner = true;
boolean userState = true;
boolean bumperState = false;
//serialComm.wristBumperState = false;
//serialComm.isSafe = true;

float wristState = 0; 

const int WRIST_UP = 0;
const int WRIST_DOWN = 1;
const int WRIST_UP_POS = 100;
const int WRIST_DOWN_POS = 1650;

const int BUMPER_PRESSED = false;
const int BUMPER_NOT_PRESSED = true;

const int USER_PRESSED = false;
const int USER_NOT_PRESSED = true;

void setup() {
    Serial.begin(115200);       // initialize Serial Communication
    encoder.init();  // connect with encoder
    wheelVelCtrl.init();        // connect with motor
    servoWrist.attach(servoWristPin);
    pinMode(wristBumperPin, INPUT);
    pinMode(bumperPin, INPUT);
    pinMode(userPin, INPUT);
    delay(1e3);                 // delay 1000 ms so the robot doesn't drive off without you
}

void loop() {
    //timed loop implementation
    unsigned long currentTime = micros();

    //read the limit switches
    userState = digitalRead(userPin);
    serialComm.wristBumperState = digitalRead(wristBumperPin);
    bumperState = digitalRead(bumperPin);
    
    //if either bumper switch has been pressed, it is not safe to move.
    if (bumperState == BUMPER_PRESSED){
      serialComm.isSafe = false;
    }

    //if it has been not safe to move, but the user switch is pressed, then it is safe to move again
    if ((serialComm.isSafe == false) && (userState == USER_PRESSED)){
      serialComm.isSafe = true;
    }
    
    /*Serial.print("isSafe ");
    Serial.print(isSafe);
    Serial.print("\n");*/
    
    if ((currentTime - prevTime >= PERIOD_MICROS)) {
      
        // 1. Check encoder
        encoder.update(); 

        // 2. Update position
        robotPose.update(encoder.dThetaL, encoder.dThetaR); 

        // 3. Send odometry through serial communication
        serialComm.send(robotPose); 
        serialComm.receiveSerialData();

        //update wristState
        wristState = serialComm.desiredWrist;

        if (serialComm.isSafe) {
          // 4. Send the velocity command to wheel velocity controller
          wheelVelCtrl.doPIControl("Left",  serialComm.desiredWV_L, encoder.v_L); 
          wheelVelCtrl.doPIControl("Right", serialComm.desiredWV_R, encoder.v_R);        
        
          //move the wrist
          if (wristState == WRIST_DOWN)
          {
             servoWrist.write(WRIST_DOWN_POS);
          }
          else
          {
            servoWrist.write(WRIST_UP_POS);
          }
        }
        else {
          wheelVelCtrl.doPIControl("Left",  0, encoder.v_L); 
          wheelVelCtrl.doPIControl("Right", 0, encoder.v_R);
          servoWrist.write(WRIST_UP_POS);
        }
        //Serial.print("wrist command: ");
        //Serial.print(serialComm.desiredWrist);
        //Serial.print("\n");

        prevTime = currentTime; // update time
    }
}




