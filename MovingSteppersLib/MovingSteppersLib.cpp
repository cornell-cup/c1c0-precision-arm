#include "Arduino.h"
#include "MovingSteppersLib.h"

MovingSteppersLib::MovingSteppersLib(stepPinIn, dirPinIn)
{
    stepPin = stepPinIn;
    dirPin = dirPinIn;
}

void moveJ1(dir,angle){
  //moves the J1 stepper motor
  //this motor has a gearRatio of 20, so 20:1 speed reducer
  //microstepping 1 means 200 steps per revolution
  //microstepping 2 then 400 steps/rev, and so on
  //use microstep = 2 on this motor

  int microStep = 2;
  int StepsPerRev = 200;
  int gearRatio = 50;
  //steps to move one degree, known from testing
  double stepsPerDegree = microStep*stepsPerRev/360;
  double stepsToTurn = stepsPerDegree * angle;
  for(int x=0; x < stepsToTurn; x++){
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(500);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(500);
  }
}

void moveJ2(dir,angle){
  //moves the J2 stepper motor
  //this motor has a gearRatio of 50, so 50:1 speed reducer
  //microstepping 1 means 200 steps per revolution
  //microstepping 2 then 400 steps/rev, and so on
  //use microstep = 1 on this motor

  int microStep = 1;
  int StepsPerRev = 200;
  int gearRatio = 50;
  // INPUT CALCULATION steps to move one degree, known from testing
  double stepsPerDegree = microStep*stepsPerRev/360;
  double stepsToTurn = stepsPerDegree * angle;
  for(int x=0; x < stepsToTurn; x++){
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(500);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(500);
  }
}


void moveJ3(int dir, double angle){
  //moves the J3 stepper motor
  //this motor has a gearRatio of 50, so 50:1 speed reducer
  //microstepping 1 means 200 steps per revolution
  //microstepping 2 then 400 steps/rev, and so on
  //use microstep = 1 on this motor

  int microStep = 1;
  int StepsPerRev = 200;
  int gearRatio = 50;
  //INPUT CALCULATION steps to move one degree, known from testing
  double stepsPerDegree = microStep*stepsPerRev/360;
  double stepsToTurn = stepsPerDegree * angle;
  for(int x=0; x < stepsToTurn; x++){
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(500);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(500);
  }
}

void moveJ4(int dir, double angle){
  //moves the J4 stepper motor
  //this motor has a gearRatio of 14, so 14:1 speed reducer
  //microstepping 1 means 200 steps per revolution
  //microstepping 2 then 400 steps/rev, and so on
  //use microstep = 2 on this motor
  int microStep = 2;
  int StepsPerRev = 200;
  int gearRatio = 14;
  //21.1111 steps to move one degree, known from testing
  double stepsPerDegree = microStep*stepsPerRev/360;
  double stepsToTurn = stepsPerDegree * angle;
  for(int x=0; x < stepsToTurn; x++){
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(500);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(500);
  }
}

void moveJ5(int dir,double angle){
  //moves the J5 stepper motor
  //this motor has a gearRatio of 1, so 1:1 speed reducer
  //microstepping 1 means 200 steps per revolution
  //microstepping 2 then 400 steps/rev, and so on
  //use microstep = 2 on this motor
  int microStep = 800;
  int StepsPerRev = 200;
  int gearRatio = 1;
  //21.1111 steps to move one degree, known from testing
  double stepsPerDegree = microStep*stepsPerRev/360;
  double stepsToTurn = stepsPerDegree * angle;
  for(int x=0; x < stepsToTurn; x++){
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(500);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(500);
  }
}

void moveJ6(int dir,double angle){
  //moves the J6 stepper motor
  //this motor has a gearRatio of 19, so 19:1 speed reducer
  //microstepping 1 means 200 steps per revolution
  //microstepping 2 then 400 steps/rev, and so on
  //use microstep = 2 on this motor
  int microStep = 2;
  int StepsPerRev = 200;
  int gearRatio = 19;
  //21.1111 steps to move one degree, known from testing
  double stepsPerDegree = microStep*stepsPerRev/360;
  double stepsToTurn = stepsPerDegree * angle;
  for(int x=0; x < stepsToTurn; x++){
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(500);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(500);
  }
}
