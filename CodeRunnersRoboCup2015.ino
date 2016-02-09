///////////////////
//Info
//////////////////
/*
This is the final software release for the CodeRunners Robotics Team's 2015 RoboCup Junior Soccer Robot.
This software controls a robot that competed in RoboCup Junior 2015. 
The competition involves 2v2 autonomous robot soccer using an infrared ball.
To read more about the competition, see this link: http://rcj.robocup.org/soccer.html
For more information about the robot hardware, see this link: http://ben-soudry.github.io/robocup
*/


///////////////////
//Motor Driver Pins
///////////////////
  //Motor Configuration:
  /// A       B
  ///  \    /
  ///    |
  ///    C
  
#define motorA 10 //motor ID
#define motorA_PWM 4
#define motorA_IN_A 35
#define motorA_IN_B 33
#define motorA_CS A13

#define motorB 11 // motor ID
#define motorB_PWM 3
#define motorB_IN_A 29
#define motorB_IN_B 25
#define motorB_CS A14

#define motorC 12 //motor ID
#define motorC_PWM 7
#define motorC_IN_A 41
#define motorC_IN_B 37
#define motorC_CS A15

/////////
//IR Reciever Pins
////////
  ///      IR11 IR0
  ///  IR10        IR1
  /// IR9           IR2
  ///IR8             IR3
  /// IR7           IR4
  ///     IR6    IR5 
#define IR0 A1
#define IR1 A10
#define IR2 A2
#define IR3 A3
#define IR4 A4
#define IR5 A5
#define IR6 A6
#define IR7 A7
#define IR8 A8
#define IR9 A9
#define IR10 A11
#define IR11 A0

#define IR_min 16
#define IR_max 290
//approximate angle the sensors are pointed in degrees - {78.75, 56.25, 33.75, 11.25, 348.75, 303.75, 236.25, 191.25, 168.75, 146.25, 123.75, 101.25};
const float IR_Angle[12] = {1.3744, 0.9817, 0.5890, 0.1963, 6.0868, 5.3014, 4.1233, 3.3379, 2.9452, 2.5525, 2.1598, 1.7671}; //converted to radians
//Trig lookup table for IR sensors
const float IR_sin[12] = {sin(1.3744), sin(0.9817), sin(0.5890), sin(0.1963), sin(6.0868), sin(5.3014), sin(4.1233), sin(3.3379), sin(2.9452), sin(2.5525), sin(2.1598), sin(1.7671)};
const float IR_cos[12] = {cos(1.3744), cos(0.9817), cos(0.5890), cos(0.1963), cos(6.0868), cos(5.3014), cos(4.1233), cos(3.3379), cos(2.9452), cos(2.5525), cos(2.1598), cos(1.7671)};

float lastBallX_vect; //for smoother ball tracking, the last cartesian vector of the ball relative to the robot is saved and added to the newest vector
float lastBallY_vect;
/////////
//I2C Color Sensor
////////
#include <Wire.h>
#include <Adafruit_TCS34725.h>
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_2_4MS, TCS34725_GAIN_1X); //adjust gain

#define green 0 //Color ID for green
#define greenR 2  //3 //RGB color values for the Green of the field
#define greenG 3 //4
#define greenB 1 //2

#define black 1 //Color ID for black
#define blackLineR 2 // 3 //RGB color values for black lines on field
#define blackLineG 2 //2
#define blackLineB 1 //2


#define white 2 //Color ID for white
#define whiteLineR 28 //27
#define whiteLineG 29 //28
#define whiteLineB 20 //19

#define errorColor -1

#define whiteR_min 5
#define whiteG_min 6
#define whiteB_min 3

#define whiteR_max 38
#define whiteG_max 30
#define whiteB_max 30
/////////
//Laser photogate
////////
#define photocellPin A12
#define photocellThreshold 300 //350 
#define photocellMin 5 
#define photocellMax 600
/////////
//Compass Sensor
////////
#define compassCommand 0x13
float fieldNorth;
/////////
//Sonar Sensors
////////
  //    uS_F
  //uS_L     uS_R
  //    uS_B

#include <NewPing.h>
#define TRIGGER_PIN_F  46
#define ECHO_PIN_F     46
#define TRIGGER_PIN_R  47
#define ECHO_PIN_R     47
#define TRIGGER_PIN_B  48
#define ECHO_PIN_B     48
#define TRIGGER_PIN_L  49
#define ECHO_PIN_L     49

#define MAX_DISTANCE 300 //in cm
NewPing sonarF(TRIGGER_PIN_F, ECHO_PIN_F, MAX_DISTANCE);
NewPing sonarL(TRIGGER_PIN_L, ECHO_PIN_L, MAX_DISTANCE);
NewPing sonarR(TRIGGER_PIN_R, ECHO_PIN_R, MAX_DISTANCE);
NewPing sonarB(TRIGGER_PIN_B, ECHO_PIN_B, MAX_DISTANCE);

int sonarF_val;
int sonarL_val; 
int sonarR_val;
int sonarB_val;
boolean sonarToggle;


//Critical Robot Sonar Dimensions
#define F_DistFromCenter 2 //(5.05cm)-1-2
#define L_DistFromCenter 5 //(8.3cm)-1-2
#define R_DistFromCenter 9 //(8.3cm)+1
#define B_DistFromCenter 9 //(8.3cm)+1
/////////
//Debugging LED / Push Button
////////
#define ledPin 36
#define buttonPin 40
/////////
//Tactics Constants
////////
#define pursuitCorrection 1.4  //2 //how much to adjust the pursuit angle of the robot relative to the ball when tracking the ball - lower = more correction
#define captureRight 80 //Right boundary for no pursuit compensation
#define captureLeft 100 //Left boundary for no pursuit compensation
/////////
//Game State Variables
/////////
boolean hasBall = false;
int hasBall_counter = 0; //stores how long since the ball was seen
boolean stopped = false;
boolean avoidingOutOfBounds = false;
  int lastDriveDir; //stores the last drive angle 
  int avoidingOutOfBounds_steps = -1; //counts how many steps of out of bounds avoidance has been taken - 
boolean kicking = false;
  int kicking_steps = -1;
boolean avoidingGoal = false;
  int avoidingGoal_steps = -1; //counts how many steps of goal avoidance has been taken 
  int goalCounter = 0; //counts how long the goal is in front of the robot
  boolean rearBeamSeen = false;
//Critical Field Dimensions
#define FIELD_X 182
#define FIELD_Y 244
#define GOAL_TO_GOAL 184
#define FIELD_CENTER_X 91 //91 from the side
#define FIELD_CENTER_Y 122 //122 from the front/back
#define GOAL_X_Corner_L -35 //left corner of the goal
#define GOAL_X_Corner_R 35 //right corner of the goal

long startTime;
void setup()
{
  Serial.begin(9600);
  Serial1.begin(9600);
  
  pinMode(motorA_CS, INPUT);
  pinMode(motorA_IN_A, OUTPUT);
  pinMode(motorA_IN_B, OUTPUT);
  pinMode(motorA_PWM, OUTPUT);
  
  pinMode(motorB_CS, INPUT);
  pinMode(motorB_IN_A, OUTPUT);
  pinMode(motorB_IN_B, OUTPUT);
  pinMode(motorB_PWM, OUTPUT);
  
  pinMode(motorC_CS, INPUT);
  pinMode(motorC_IN_A, OUTPUT);
  pinMode(motorC_IN_B, OUTPUT);
  pinMode(motorC_PWM, OUTPUT);
  
  pinMode(ledPin, OUTPUT);
  pinMode(buttonPin, INPUT);
 
  motor(motorA, 0);
  motor(motorB, 0);
  motor(motorC, 0);
  
  //initialize color sensor
    if (tcs.begin())
    {
      Serial.println("Found sensor");
    } 
    else{
      Serial.println("No TCS34725 found ... check your connections");
   }
  digitalWrite(ledPin, LOW);
  delay(300); //get rid of this before competition
  //getting compass calibration value
  fieldNorth = readCompass();
  Serial.print("Field North:");
  Serial.println(fieldNorth);
  
  startTime = millis();
}
//PID Values
float kp = 2.3; //1.9 - very stable, sometimes stops the robot too angled - 2.1-2.3 wobbles off of center, but otherwise stable + better at maintaining heading - 2.5 great
float kd  = 0.1; //0.6

float lastErr = 0.0;

int currentPWR = 60;
int currentDir = 0;  
void loop()
{
  long loopStart = millis();
  
  int IR_sorted[12];
  int IR_unsorted[12];
  int IR_ID[12];
  
  //Read Sensors
  int colorVal1 = readColorSensor();
  
  if(sonarToggle == true)
  {
    sonarF_val = sonarF.ping_cm(); //read front and back sonars
    sonarB_val = sonarB.ping_cm();
    sonarToggle = false;
  }
  else
  {
    sonarR_val = sonarR.ping_cm(); //read left and right sonars
    sonarL_val = sonarL.ping_cm();
    sonarToggle = true;
  }

  int colorVal2 = readColorSensor();
  
  //Serial.print("ColorVal1: ");
  //Serial.println(colorVal1);
  //Serial.print("ColorVal2: ");
  //Serial.println(colorVal2);

 
  //int sonarF_val = 0; //Sonars
  //int sonarR_val = 0;
  //int sonarB_val = 0;
  //int sonarL_val = 0;
  float ballAngle = readIR(IR_sorted, IR_unsorted, IR_ID); //IR
  //hasBall Logic
  //Serial.print("IR Sorted-0: "); Serial.println(IR_sorted[0]);
  if(readPhotogate() == true && (ballAngle > 30 && ballAngle < 120) && IR_sorted[0] < 40) //photocell triggered, IR sensors indicate ball is close and in the front
  {
    hasBall = true; //Photogate
    hasBall_counter = 0;
    //digitalWrite(ledPin, HIGH);
    //Serial.println("Robot Has Ball + light gate triggered");
  }
  else if((ballAngle > 30 && ballAngle < 120) && IR_sorted[0] < 40 && hasBall == true && hasBall_counter < 3)//photocell triggered recently but not now, ball is close and in the front
  {
    //digitalWrite(ledPin, HIGH);
    hasBall = true;
    hasBall_counter++;
    //Serial.println("Robot Has Ball + light gate not triggered");
  }
  else{
    //digitalWrite(ledPin,LOW);
    hasBall = false;
    hasBall_counter = 0;
    //Serial.println("Robot does not have the ball");
  }
 //State Machine triggers
  //avoidingGoal state trigger logic 
  if(avoidingGoal_steps > 40){
    avoidingGoal = false;
    rearBeamSeen = false;
    avoidingGoal_steps = -1;
    //Serial.println("avoidingGoal stop");
    //digitalWrite(ledPin, LOW);
  }
    //goalCounter logic
     if(sonarF_val < 7 && sonarF_val >= 0 && avoidingGoal == false) //consider adding - hasBall == true actually
     {
       //Serial.println("this ran");
       goalCounter++;
       if(sonarB_val > 160 && sonarB_val < 186)
       {
         rearBeamSeen = true;
       }
     }
     else
     {
       goalCounter = 0;

     }
  if(goalCounter > 8 && rearBeamSeen == true)
  {
    avoidingGoal = true;
    avoidingGoal_steps = -1;
    //Serial.println("avoidingGoal start");
    //digitalWrite(ledPin, HIGH);
    goalCounter = 0;
  }
  else if(goalCounter > 30)  
  {
     goalCounter = 0;
  }
  //kicking state trigger logic 
  if(kicking_steps > 16)
  {
    kicking = false;
    kicking_steps = -1;
    //Serial.println("kick stop");
    //digitalWrite(ledPin, LOW);
  }
  if(hasBall == true && kicking == false && avoidingOutOfBounds == false)
  {
    kicking = true;
    
    kicking_steps = -1;
    
    avoidingGoal = false;
    avoidingGoal_steps = -1;
    //Serial.println("kick start");
    //digitalWrite(ledPin, HIGH);
  }
  
  //Avoiding Out of Bounds state trigger logic
  if((avoidingOutOfBounds_steps > 30))
  {
    avoidingOutOfBounds = false;
    avoidingOutOfBounds_steps = -1;
    //digitalWrite(ledPin, LOW);
  }
  if((colorVal1 == white || colorVal2 == white) && avoidingOutOfBounds == false) //white line detected
  {
    avoidingOutOfBounds = true; 
    avoidingOutOfBounds_steps = -1; //just added
    kicking = false;
    kicking_steps = -1; 
    avoidingGoal = false;
    avoidingGoal_steps = -1; 
    //digitalWrite(ledPin, HIGH);
  }

  //////////
  //Timeline
  //////////
  /*
  long currentTime = millis();
  if((currentTime - startTime) > 20000)
  {
    currentPWR = 0;
    currentDir = 0;
    stopped = true;
    digitalWrite(ledPin, LOW);
  }
  if((currentTime - startTime) > 22000)
  {
    Serial.println("Stop");
    Serial.print("hasBall: "); Serial.println(hasBall);
    brakeMotors();
    
    while(true)
    {
      //stop
    }
  }*/
  //////
  //Game State Machine:
  //////
  if(avoidingOutOfBounds == false && kicking == false && avoidingGoal == false) //ball pursuit state - replaced hasBall w/ kicking
  {
    currentDir = ballPursuitAngle(ballAngle, IR_sorted[0]);
    currentPWR = 60;
    drivePID(currentDir, currentPWR);
    lastDriveDir = currentDir;
    //digitalWrite(ledPin, HIGH);
  }
  else if(avoidingGoal == true && avoidingOutOfBounds == false) //avoiding the goal state - move back and stop
  {
    //digitalWrite(ledPin, LOW);
    
    avoidingGoal_steps++;
    if(avoidingGoal_steps < 30){ //just added (changed)
      currentDir = 270;
      currentPWR = 75;
    }
    else if(avoidingGoal_steps < 33){
      currentDir = 90; //reverse thrust
      currentPWR = 60;
    }
    else{
      currentDir = 0; //stop
      currentPWR = 0;
    }
    drivePID(currentDir, currentPWR);
    lastDriveDir = 270;
  }
  else if(kicking == true && avoidingOutOfBounds == false) //attempting a goal state  replaced hasBall w/ kicking
  {
    //digitalWrite(ledPin, LOW);
    //currentDir = goalPursuitAngle(sonarF_val,sonarR_val,sonarB_val,sonarL_val); //to be changed
    //drivePID(currentDir, currentPWR);
    
    kicking_steps++;
    
    //consider reverting to simple kick (test side cases to see if they ever run using "this ran")
    
    //Serial.print("sonarL: ");
    //Serial.println(sonarL_val);
    //Serial.print("sonarR");
    //Serial.println(sonarR_val);
    
    if(sonarL_val > sonarR_val + 60) //on the right side of the field
    {
      //Serial.println("Left Kick (Right of Field)");  
      if(kicking_steps < 7){
        currentDir = 130;
        currentPWR = 150; //130
      }
      else if(kicking_steps < 12){
        currentDir = 290; //reverse thrust
        currentPWR = 60;
      }
      else{
        currentDir = 0; //stop
        currentPWR = 0;
      }
      drivePID(currentDir, currentPWR);
      lastDriveDir = 110; 
      
    }
    else if(sonarR_val > sonarL_val + 60)//on the left side of the field
    {
      //Serial.println("Right Kick (Left Field)");
      if(kicking_steps < 7){
         currentDir = 70;
         currentPWR = 150;
      }
      else if(kicking_steps < 12){
        currentDir = 250; //reverse thrust
        currentPWR = 60;
      }
      else{
        currentDir = 0; //stop
        currentPWR = 0;
      }
      drivePID(currentDir, currentPWR);
      lastDriveDir = 70;       
    }
    else //in the middle of the field
    {
      //Serial.println("Normal Kick (Center Field)");
      if(kicking_steps < 7){
        currentDir = 90;
        currentPWR = 150;
      }
      else if(kicking_steps < 12){
        currentDir = 270; //reverse thrust
        currentPWR = 60;
      }
      else{
        currentDir = 0; //stop
        currentPWR = 0;
      }
      drivePID(currentDir, currentPWR);
      lastDriveDir = 90; 
    }
    
  }
  else if(avoidingOutOfBounds == true) //avoiding out of bounds - with/without ball
  {
    //Serial.println("Avoiding Out of Bounds");
   //digitalWrite(ledPin, LOW);
    avoidingOutOfBounds_steps++;
    if(avoidingOutOfBounds_steps < 24) //raise this number - just changed
    {
      drivePID((lastDriveDir+180),65);
    }
    else if(sonarF_val < 40){
      drivePID(270, 55); //back
      //Serial.println("wall forward");
    }
    else if(sonarB_val < 40){
      drivePID(90, 55); //forward
      //Serial.println("wall back");
    }
    else if(sonarL_val < 40){
      drivePID(0, 55); //right
      //Serial.println("wall left");
    }
    else if(sonarR_val < 40){
      drivePID(180, 55); //left
      //Serial.println("wall right");
    }
    else
    {
      drivePID(0,0); //consider stopping avoidOutOfBounds here.
    }
  }
   long loopStop = millis();
   int t = loopStop - loopStart; 
   //Maintains a 33 hz refresh rate on the loop
   while(t < 30) //34
   {
     loopStop = millis();
     t = loopStop - loopStart;
   }
   //Serial.print("t: ");
   //Serial.println(t);
   //Serial.print("avoidingOut var:");
  // Serial.println(avoidingOutOfBounds);
   
   //Serial.print("avoidingGoal_steps: ");
   //Serial.println(avoidingGoal_steps);
   //Serial.print("stopped: ");
   //Serial.println(stopped);
   //Serial.print("kicking ");
   //Serial.println(kicking);
   //Serial.print("photogate");
   //Serial.println(readPhotogate());
}
//////////////
//Tactics//
//////////////
float ballPursuitAngle(float ballAngle, int ballDistance)
{
  //adjustments based on ball distance
  //Serial.print("ballDistance: "); Serial.println(ballDistance);
  int distanceCorrection = 0;
  if(ballDistance < 40) //ball is close
  {
     currentPWR = 55;
    if(ballAngle > 90 && ballAngle < 270)//on the left side
    {
      distanceCorrection = 0; //8
    } 
    else
    {
      distanceCorrection = 0;//-8
    }
  }
  else
  {
    currentPWR = 65;
  }
  //adjustments based on ball angle
  if(ballAngle > captureRight && ballAngle < captureLeft) //ball is directly in front, no correction needed
  {
    return ballAngle;
  }
  else if(ballAngle > captureLeft && ballAngle < 270) //ball is on the left, the pursuit angle must be increased}
  {
    float correctionAngle = (ballAngle - captureLeft)/pursuitCorrection;
    return ballAngle + correctionAngle + distanceCorrection;
  }
  else if(ballAngle > 270 && ballAngle < 360)//ball is on the Right (270-360 degrees), the pursuit angle must be decreased
  {
    float correctionAngle = ((360-ballAngle)+captureRight)/pursuitCorrection;
    return ballAngle - correctionAngle + distanceCorrection;
  }
  else if(ballAngle > 0 && ballAngle < captureRight) //ball is on the Right(0-60 degrees), the pursuit angle must be decreased
  {
    float correctionAngle = (captureRight- ballAngle)/pursuitCorrection;
    return ballAngle - correctionAngle + distanceCorrection;
  }
}
float goalPursuitAngle(int sonarF, int sonarR, int sonarB, int sonarL)//uses localization to find a path to the goal
{
  ///////////////////
  //Localization
  ///////////////////
  //Critical Localization Values
  int x1; //Based on L
  int y1; //Based on F
  int x2; //Based on R
  int y2;  //Based on B

  int xPos;
  int yPos;

  boolean isBetweenGoals;
  boolean isInCorner;
  boolean yBeamError = false;
  boolean xBeamError = false;

    //Check if robot is between goals or not (Y cases)
    int yTotal = sonarF + F_DistFromCenter + sonarB + B_DistFromCenter;
    if(yTotal < GOAL_TO_GOAL + 10 && yTotal > GOAL_TO_GOAL - 10)
    {
       //Serial.println("Between Goals");
       isBetweenGoals = true;
       yBeamError = false;
    }
    else if(yTotal < FIELD_Y + 10 && yTotal > FIELD_Y - 10)
    {
       //Serial.println("Not Between the Goals");
       isBetweenGoals = false;
       yBeamError = false;
    }
    else
    {
      //Serial.println("Robot in Front or Behind / Rotated / Error");
       yBeamError = true;
    }
    //Check if robot is in a corner (X cases)
    int xTotal = sonarL + L_DistFromCenter + sonarR + R_DistFromCenter;
    if(xTotal < FIELD_X + 10 && xTotal > FIELD_X -10)
    {
       isInCorner = false;
       xBeamError = false;
    }
    else
    {
      xBeamError = true;
    }
    //Localization Calculations
    if(yBeamError == false)
    {
      if(isBetweenGoals)
      {
        //creates centers of (0,0) for the field)
        y1 = FIELD_CENTER_Y - (F_DistFromCenter + sonarF + 30); //calculated with uS_F
        y2 = (B_DistFromCenter + sonarB + 30)- FIELD_CENTER_Y; //calculated with uS_B
      }
      else
      {
        y1 = FIELD_CENTER_Y - (F_DistFromCenter + sonarF); //calculated with uS_F
        y2 = (B_DistFromCenter + sonarB) - FIELD_CENTER_Y; //calculated with uS_B
      }
    }
    if(xBeamError == false)
    {
       x1 = (L_DistFromCenter+ sonarL) - FIELD_CENTER_X; //calculated with uS_L
       x2 = FIELD_CENTER_X - (R_DistFromCenter+ sonarR);//calculated with uS_R
    }
  //Serial.print("y1: ");
  //Serial.println(y1);
  
  //Serial.print("y2: ");
  //Serial.println(y2);
  
  //Serial.print("yTotal: ");
  //Serial.println(yTotal);
  
  //Serial.print("x1: ");
  //Serial.println(x1);
  
  //Serial.print("x2: ");
  //Serial.println(x2);
  
  //Serial.print("xTotal: ");
  //Serial.println(xTotal);
  
  if(xBeamError){
    //Serial.println("xBeamError");
  }
  else{
    //Serial.println("No xBeamError");
  } 
  ////////////////////////////////////////////////////////////
 //Generating drive speed and angle to goal for a variety of cases
 ////////////////////////////////////////////////////////////
    //adjust drive speed based on proximity to goal
    if(yBeamError == false) 
      {
        int yAvg = (y1+y2)/2;
        if(yAvg < 0) //on the other half of the field - far goal
        {
          currentPWR = 65;
        }
        else if(yAvg < 30){
          currentPWR = 65;
        }
        else if(sonarF < 60){
          currentPWR = 60;
        }
        else{
          currentPWR = 55;
        } 
      }
      else //robots in the front or rear - no reliable y data
      {
        if(sonarF < 18){
          currentPWR = 55;
        }
        else{
          currentPWR = 60;
        } 
      }
    //delay(500);
    //generating drive angle
    if(xBeamError == false && isBetweenGoals == true) //no robots on the flank + between goals - simple case #1
    {
      //Serial.println("Simple Case #1");
      //finding Angle
      int xAvg = (x1+x2)/2; //average of the x coordinates found with the L and R sonars
      if(xAvg > -15 && xAvg <  15) //directly facing the middle of the goal
      {
        //Serial.println("Goal Straight Ahead");
        return 90; //drive straight ahead
      }
      else if(xAvg > -35 && xAvg <= -15) //facing the left of the goal
      {
        //Serial.println("Facing left side of Goal");
        return 80; //drive right slightly
      }
      else if(xAvg < 35 && xAvg >=  1) //facing the right of the goal
      {
        //Serial.println("Facing right side of goal");
        return 100; //drive left slightly
      } 
    }
    else if(xBeamError == false && isBetweenGoals == false) //no robots on the flank + not between goals - simple case #2
    {
      //Serial.println("Simple Case #2");
      //finding Angle
      int xAvg = (x1+x2)/2; //average of the x coordinates found with the L and R sonars
      if(xAvg > 25 && xAvg < 55) //to the right of the goal slightly
      {
        //Serial.println("Right of the goal slightly");
        return 125;
      }
      else if(xAvg < -25 && xAvg > -55) //to the left of the goal slightly
      {
         //Serial.println("left of the goal slightly");
         return 55; 
      }
      else if(xAvg >= 55 && xAvg < 80) //to the right of the goal
      {
        //Serial.println("Far Right of goal");
        return 135;
      }
      else if(xAvg <= -55 && xAvg > -80) //to the left of the goal
      {
         //Serial.println("Far Left of Goal");
         return 45; 
      }
    }
    else if(xBeamError == true && isBetweenGoals == true) //robots on the flank + between goals - complex case #1
    {
      //Serial.println("Complex Case #1");
      return 90; //drive straight - more complex behavior can be added here
    } 
    else if(xBeamError == true && isBetweenGoals == false) //robots on the flank + between goals - complex case #2
    {
      //Serial.println("Complex Case #2");
      return 90; //drive straight - more complex behavior can be added here
    }
    else //unknown case
    {
      //Serial.println("Unknown Case");
      return 90;
    }   
}
//////////////
//Functions for Reading Sensors//
//////////////
float readIR(int get_IR_sorted[], int get_IR_unsorted[], int get_IR_ID[]) //the float that is returned is the calculated angle to the ball
{
  //Reading from the 12 IR recievers
  get_IR_unsorted[0] = analogRead(IR0); get_IR_ID[0] = 0;
  get_IR_unsorted[1] = analogRead(IR1); get_IR_ID[1] = 1;
  get_IR_unsorted[2] = analogRead(IR2); get_IR_ID[2] = 2;
  get_IR_unsorted[3] = analogRead(IR3); get_IR_ID[3] = 3;
  get_IR_unsorted[4] = analogRead(IR4); get_IR_ID[4] = 4;
  get_IR_unsorted[5] = analogRead(IR5); get_IR_ID[5] = 5;
  get_IR_unsorted[6] = analogRead(IR6); get_IR_ID[6] = 6;
  get_IR_unsorted[7] = analogRead(IR7); get_IR_ID[7] = 7;
  get_IR_unsorted[8] = analogRead(IR8); get_IR_ID[8] = 8;
  get_IR_unsorted[9] = analogRead(IR9); get_IR_ID[9] = 9;
  get_IR_unsorted[10] = analogRead(IR10); get_IR_ID[10] = 10;
  get_IR_unsorted[11] = analogRead(IR11); get_IR_ID[11] = 11;
  //copy data to the sorted array before sorting
  for(int i = 0; i < 12; i++){
    get_IR_sorted[i] = get_IR_unsorted[i];
  }
  //perform insertion sort
  insertSort(get_IR_sorted, get_IR_ID); 
  //check for errors
  if(get_IR_sorted[0] < IR_min || get_IR_sorted[0] > IR_max) //if none of the sensors see the ball
  {
    stopped = true;
  }
  else
  {
    stopped = false;
  }
  //create a cartesian vector for the ball location using IR sensor data
  float ballX_vect = 0;
  float ballY_vect = 0;
  float rankMultiplier[12] = {7,6,5,4,3,2,1,-1,-2,-3,-4,-5}; //This sets what magnitudes each of the IR sensors will get for calculation - Ex. The sensor with the strongest strength gets x8 
  for(int i = 0; i < 12; i++) //loop adds in vectors from the strongest signal strength sensor to the weakest
  {
    ballX_vect = ballX_vect + rankMultiplier[i] * IR_cos[get_IR_ID[i]];
    ballY_vect = ballY_vect + rankMultiplier[i] * IR_sin[get_IR_ID[i]];
    //Serial.print("IR"); + Serial.print(i); 
    //Serial.print(" X: "); Serial.println(rankMultiplier[i] * cos(IR_Angle[i])); 
    //Serial.print(" Y: "); Serial.println(rankMultiplier[i] * sin(IR_Angle[i])); 
  }
  //Serial.print("ballX_vect: "); Serial.println(ballX_vect);
  //Serial.print("ballY_vect: "); Serial.println(ballY_vect);
  //Add in the last cartesian vector (generates smoother results for ball tracking)
  float avgBallX_vect = ballX_vect + lastBallX_vect;
  float avgBallY_vect = ballY_vect + lastBallY_vect; 
  //Serial.print("avgBallX_vect: "); Serial.println(avgBallX_vect);
  //Serial.print("avgBallY_vect: "); Serial.println(avgBallY_vect);
  //Convert to an angle
  float calculatedAngle = (atan2(avgBallY_vect, avgBallX_vect)*4068)/71; //converted to degrees
  if(calculatedAngle < 0){
    calculatedAngle = calculatedAngle + 360;
  }
  //save the x and y vector values
  lastBallX_vect = ballX_vect;
  lastBallY_vect = ballY_vect;
  
  return calculatedAngle; 
}
boolean readPhotogate()
{
  int photocellVal = analogRead(photocellPin);
  //Serial.print(photocellVal);
  //error checking
  if(photocellVal < photocellMin || photocellVal > photocellMax)
  {
    //Serial.println(" - photocell error!");
    //digitalWrite(ledPin, LOW);
    return false;
  }
  if(photocellVal > photocellThreshold) //the photocell sees light
  { 
    //Serial.println(" - no ball in photogate");
    //digitalWrite(ledPin, LOW);
    return false; //nothing is in the photogate
  }
  else
  {
    //Serial.println(" - ball  is in  the photogate!");
    //digitalWrite(ledPin, HIGH);
    return true; //something is blocking the photogate
  }
}
int readColorSensor()
{
   uint16_t r, g, b, c, colorTemp, lux;
  
  tcs.getRawData(&r, &g, &b, &c);
  colorTemp = tcs.calculateColorTemperature(r, g, b);
  lux = tcs.calculateLux(r, g, b);
  if((r >= whiteR_min && r <= whiteR_max)&&(g >= whiteG_min && g <= whiteG_max)&&(b >= whiteB_min && b <= whiteB_max))
  {
    //Serial.println("On the White");
    //digitalWrite(ledPin, HIGH);
    return white;
  }
  else if((r >= blackLineR-1 && r <= blackLineR+1)&&(g >= blackLineG-1 && g <= blackLineG+1)&&(b >= blackLineB-1 && b <= blackLineB+1) && (c <= 5))
  {
    //Serial.println("On the Black");
    //digitalWrite(ledPin, LOW);
    return black;
  }
  else if((r >= greenR-1 && r <= greenR+1)&&(g >= greenG-1 && g <= greenG+1)&&(b >= greenB-1 && b <= greenB+1))
  {
    //Serial.println("On the Green");
    //digitalWrite(ledPin, LOW);
    return green;
  }
  else
  {
    //Serial.println("Error");
    //digitalWrite(ledPin, LOW);
    return errorColor;
  }

}
float readCompass()
{
  Serial1.write(compassCommand);
  delay(5); //try lowering this
  if(Serial1.available())
  {  
    byte byte1 = Serial1.read();
 
    byte byte2 = Serial1.read();
    
    word compassVal = word(byte1,byte2);
    
    float processedVal =  (float) compassVal/10 ;
    return processedVal;
  }
  else
  {
    Serial.println("Error - compass not available");
    Serial.println();
    return -1; //
  }
}
//////////////
//Drive Functions//
//////////////
// The process of converting a simple drive vector into the correct motor commands was broken up into several functions:
// Hierarchy:
// drivePID(dir, PWR)
//    |
// drivePolar(dir, PWR, rot)
//    |
// driveCartesian(xVec, yVec, rot, PWR)
//    |                    |
// motor(PWR, motorID) OR brakeMotors()


//Once a drive vector is chosen, this function will use data from the compass sensor to maintain that vector and stabilize the robot's heading using a PD controller
void drivePID(int dir, int PWR)
{
  //Serial.print("drivePID: dir: ");
  //Serial.print(dir);
  //Serial.print(" PWR: ");
  //Serial.println(PWR);
  
  //check if the robot is stopped
  if(stopped == true)
  {
    PWR = 0;
    dir = 0;
  }
  float targetVal = fieldNorth;
  //Serial.print("targetVal: ");
  //Serial.print(targetVal);
  
  float currentVal = readCompass();
  //Serial.print(" currentVal: ");
  //Serial.print(currentVal);
  
  //P - Proportional
  float pErr = angleDif(currentVal,targetVal); //difference of the angles
  
  //Serial.print(" Dif: ");
  //Serial.print(pErr);
  if(pErr > 180) //if the compass passes over 0 and 360 going counterclockwise
  {
    pErr = pErr-360;
  }
  else if(pErr < -180) //if the compass passes over 0 and 360 going clockwise
  {
    pErr = 360-pErr;
  }
  float pGain = pErr * kp;
  //D - Derivative
  float dErr = lastErr + pErr;
  float dGain = dErr * kd;
  //total
  float totalRotGain = pGain + dGain;
  //if the robot is moving
  if(PWR != 0 && currentVal != -1)
  {
    totalRotGain = constrain(totalRotGain,-50,50);
    //Serial.print(" Driving - Total Gain:");
    //Serial.println(totalRotGain);
    //adjust heading
    int correctedDir = dir-pErr;
    //Serial.print("correctedDir: ");
    //Serial.println(correctedDir);
    drivePolar(correctedDir,-totalRotGain,currentPWR);
  }
  else if(PWR == 0)//if the robot is set to stop
  {
    if((totalRotGain > 24 || totalRotGain < -24) && currentVal != -1) //20
    {  
      //Serial.print(" Total Gain:");
      //Serial.println(totalRotGain);
      totalRotGain = constrain(totalRotGain,-50,50);
      //adjust heading
      drivePolar(dir,-totalRotGain,PWR);
    }
    else
    {
      //Serial.print(" Total Gain:");
      //Serial.println(totalRotGain);
      brakeMotors();
    }
  }  
  //store the last pErr
  lastErr = pErr; 
}
//Takes polar drive vector, scales it and converts it to cartesian for easier conversion to the final motor commands
void drivePolar(int dir, int rot,  int PWR)
{
  int scale = 150; //this keeps the calculated x and y components between 0 and 1500 
  int xVec = 0;
  int yVec = 0;
  //  90
  //180   0
  //  270  
  if(dir >= 360){
    dir = dir - 360;
  }
  else if(dir < 0){
    dir = dir + 360;
  }
  //Cases
  if(dir == 0){
    xVec = 1* scale;
    yVec = 0;
  }
  else if(dir == 90){
    xVec = 0;
    yVec = 1*scale;
  }
  else if(dir == 180){
    xVec = -1*scale;
    yVec = 0;
  }
  else if(dir == 270)
  {
     xVec = 0;
     yVec = -1*scale;
  }
  else{
     float dirRad = (float)dir * 71 / 4068;
     //Serial.print("dirRad:");
     //Serial.println(dirRad);
     xVec = (float)(scale*cos(dirRad));
     yVec = (float)(scale*sin(dirRad));
  }
  //Serial.print("dir:");
  //Serial.println(dir);
  
  //Serial.print("xVec: ");
  //Serial.println(xVec);
  //Serial.print("yVec: ");
  //Serial.println(yVec);

  driveCartesian(xVec, yVec, rot ,PWR);  
}

void driveCartesian(int xVec, int yVec, int rot, int PWR)
{
  //Converting Cartesian coordinates into motor vectors
  int motorA_PWR =  (xVec*-100/300 - (yVec * 100/173));
  int motorB_PWR = (xVec*-100/300 + (yVec * 100/173));
  int motorC_PWR = (xVec* 200/300);
  //scaling motor powers based on PWR value
  motorA_PWR = map(motorA_PWR,-100, 100, -PWR, PWR);
  motorB_PWR = map(motorB_PWR,-100, 100, -PWR, PWR);
  motorC_PWR = map(motorC_PWR, -100,100, -PWR, PWR);
  //Adding rotation 
  if(PWR + rot <= 255)
  {
    motorA_PWR = motorA_PWR+rot;
    motorB_PWR = motorB_PWR+rot;
    motorC_PWR = motorC_PWR+rot;
  }
  else
  {
    Serial.println("Rotational constant too high!");
  }
  //Serial.print("motorA_PWR: ");
  //Serial.println(motorA_PWR);
  
  //Serial.print("motorB_PWR: ");
  //Serial.println(motorB_PWR);
  
  //Serial.print("motorC_PWR: ");
  //Serial.println(motorC_PWR);
  
  motor(motorA, motorA_PWR);
  motor(motorB, motorB_PWR);
  motor(motorC, motorC_PWR);
}
void brakeMotors()
{
  digitalWrite(motorA_IN_A, LOW);
  digitalWrite(motorA_IN_B, LOW);
  analogWrite(motorA_PWM, 0); 
  
  digitalWrite(motorB_IN_A, LOW);
  digitalWrite(motorB_IN_B, LOW);
  analogWrite(motorB_PWM, 0); 
  
  digitalWrite(motorC_IN_A, LOW);
  digitalWrite(motorC_IN_B, LOW);
  analogWrite(motorC_PWM, 0); 
  
  //Serial.println("Brake");
}
//Low level command to send the proper signals to one of the three motor drivers that power the motors.
void motor(int motorID , int PWR)
{
  boolean motorClockwise;
  if(PWR <= 255 && PWR >= 0){
    motorClockwise = true;
   
  }
  else if(PWR >= -255 && PWR < 0){
    motorClockwise = false;
    PWR = -(PWR);
  }
  else{
    Serial.println("Invalid motor power");
    return; //error 
  }
  //Selecting a motor to run
  if(motorID == motorA){
      if(motorClockwise == true){
        digitalWrite(motorA_IN_A, HIGH);
        digitalWrite(motorA_IN_B, LOW);
      }    
      else{
        digitalWrite(motorA_IN_A, LOW);
        digitalWrite(motorA_IN_B, HIGH);
      }
      analogWrite(motorA_PWM, PWR);

      if(PWR != 0)
      {
        //Serial.print("MotorA set to ");
        //Serial.println(PWR);
        //Current sense
        float motorACurrent = (((float)analogRead(motorA_CS)/(float)1023)*(float)5)/(float)0.13;
        //Serial.print("Current of MotorA: ");
        //Serial.print(motorACurrent);
        //Serial.println("A");
      }
  }
  else if(motorID == motorB){
      if(motorClockwise == true){
        digitalWrite(motorB_IN_A, HIGH);
        digitalWrite(motorB_IN_B, LOW);
      }    
      else{
        digitalWrite(motorB_IN_A, LOW);
        digitalWrite(motorB_IN_B, HIGH);
      }
      analogWrite(motorB_PWM, PWR);
      
      if(PWR != 0)
      {
        //Serial.print("MotorB set to ");
        //Serial.println(PWR);
        //Current sense
        float motorBCurrent = (((float)analogRead(motorB_CS)/(float)1023)*(float)5)/(float)0.13;
        //Serial.print("Current of MotorB: ");
        //Serial.print(motorBCurrent);
        //Serial.println("A");
      }
  }
  else if(motorID == motorC)
  {
      if(motorClockwise == true){
        digitalWrite(motorC_IN_A, HIGH);
        digitalWrite(motorC_IN_B, LOW);
      }    
      else{
        digitalWrite(motorC_IN_A, LOW);
        digitalWrite(motorC_IN_B, HIGH);
      }
      analogWrite(motorC_PWM, PWR);      

      if(PWR != 0)
      {
        //Serial.print("MotorC set to ");
        //Serial.println(PWR);
        //Current sense
        float motorCCurrent = (((float)analogRead(motorC_CS)/(float)1023)*(float)5)/(float)0.13;
        //Serial.print("Current of MotorC: ");
        //Serial.print(motorCCurrent);
        //Serial.println("A");
      }
  }
}
//////////////
//Math/Misc Functions//
//////////////
float angleDif(float x, float y) //used to calculate the difference between actual heading and the target heading in drivePID()
{
  float radianDif = ((x-y)*71)/4068;
  float a = (float) atan2(sin(radianDif), cos(radianDif));
  a =  (a * 4068) / 71; //to degrees 

  return a;
}
void insertSort(int array[], int arrayKey[])
{
  //array[] holds the data to be sorted numerically, arrayKey holds the original position of each value
    for (int i = 1 ; i < 12; i++) 
    {
      int j = i;
      while (j > 0 && array[j] < array[j-1]) {
        //perform swap - value
        int savedVal = array[j]; //keeps the value in index j so it can be used to complete the swap after it is overode
        array[j]  = array[j-1];
        array[j-1] = savedVal;
        //perform swap - key
        int savedKey = arrayKey[j]; //keeps the position in index j so it can be used to complete the swap after it is overode
        arrayKey[j] = arrayKey[j-1];
        arrayKey[j-1] = savedKey;
        j--;
      }
    }
}
void shiftArray(int array[]) //shifts all values in a array of length 50
{
    int originalArray[50];
    //copy array into originalArray
    for (int i = 0 ; i < 50; i++){
      originalArray[i] = array[i];
    }
    for (int i = 0 ; i < 49; i++) 
    {
      array[i+1] = originalArray[i];
    }
}


