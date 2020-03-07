#include "Arduino.h"
#include "MovingSteppersLib.h"
#include "MotorEncoderLib.h"
#include <SPI.h>

int stepPin;
int dirPin;
int encoderPin;

// Using `` for setup and in movement function - not sure if it's needed
// but it's possible that it fixed some problems
MovingSteppersLib::MovingSteppersLib(int stepPinIn, int dirPinIn, int encoderPinIn)
{
    // setup step and direction pins
    stepPin = stepPinIn;
    dirPin  = dirPinIn;
    encoderPin = encoderPinIn;
    encoder.setChipSelect(encoderPin);
    // set output mode for pins
    pinMode(stepPin, OUTPUT);
    pinMode(dirPin, OUTPUT);
    prevAngle = 0.0;

    SPI.setClockDivider(SPI_CLOCK_DIV32);
    SPI.begin();
}


void MovingSteppersLib::moveJ1(double curAngle){
  //moves the J1 stepper motor
  // motor has a gearRatio of 20, so 20:1 speed reducer
  //microstepping 1 means 200 steps per revolution
  //microstepping 2 then 400 steps/rev, and so on
  //use microstep = 2 on  motor

  int microStep = 2;
  int stepsPerRev = 200;
  int gearRatio = 50;

  //compare prevAngle to curAngle to set direction in which we move
  double diffAngle = curAngle - prevAngle;
  if(diffAngle >= 0.0){
      digitalWrite(dirPin, HIGH);
  }
  else{
      digitalWrite(dirPin, LOW);
      diffAngle = -diffAngle;
  }

  //compare prevAngle to curAngle to set direction in which we move
  int encoderTarget = curAngle * 45.51111;
  int encoderPosition = encoder.getPositionSPI(14);
  // int stepCounter = 55;
  int encoderDiff = encoderTarget - encoderPosition;
  int sign = 1;

  if(encoderDiff >= 0.0){
      digitalWrite(dirPin, HIGH);
      sign = 1;
  }
  else{
      digitalWrite(dirPin, LOW);
      sign = -1;
  }
  encoderDiff = sign * encoderDiff;

  while (encoderDiff >  10) {    // tolerance for the diff
      digitalWrite(stepPin, HIGH);
      delayMicroseconds(300);
      digitalWrite(stepPin, LOW);
      delayMicroseconds(300);
      encoderPosition = encoder.getPositionSPI(14);
      encoderDiff = encoderTarget - encoderPosition;
      if(encoderDiff >= 0.0){
          digitalWrite(dirPin, HIGH);
          sign = 1;
      }
      else{
          digitalWrite(dirPin, LOW);
          sign = -1;
      }
      encoderDiff = sign * encoderDiff;
  }

}

void MovingSteppersLib::moveJ2(double curAngle){
  //moves the J2 stepper motor
  // motor has a gearRatio of 50, so 50:1 speed reducer
  //microstepping 1 means 200 steps per revolution
  //microstepping 2 then 400 steps/rev, and so on
  //use microstep = 1 on  motor

  int microStep = 1;
  int stepsPerRev = 200;
  int gearRatio = 50;

  //compare prevAngle to curAngle to set direction in which we move
  int encoderTarget = curAngle * 45.51111;
  int encoderPosition = encoder.getPositionSPI(14);
  // int stepCounter = 55;
  int encoderDiff = encoderTarget - encoderPosition;
  int sign = 1;

  if(encoderDiff >= 0.0){
      digitalWrite(dirPin, HIGH);
      sign = 1;
  }
  else{
      digitalWrite(dirPin, LOW);
      sign = -1;
  }
  encoderDiff = sign * encoderDiff;

  while (encoderDiff >  10) {    // tolerance for the diff
      digitalWrite(stepPin, HIGH);
      delayMicroseconds(300);
      digitalWrite(stepPin, LOW);
      delayMicroseconds(300);
      encoderPosition = encoder.getPositionSPI(14);
      encoderDiff = encoderTarget - encoderPosition;
      if(encoderDiff >= 0.0){
          digitalWrite(dirPin, HIGH);
          sign = 1;
      }
      else{
          digitalWrite(dirPin, LOW);
          sign = -1;
      }
      encoderDiff = sign * encoderDiff;
  }

}


void MovingSteppersLib::moveJ3( double curAngle){
  //moves the J3 stepper motor
  // motor has a gearRatio of 50, so 50:1 speed reducer
  //microstepping 1 means 200 steps per revolution
  //microstepping 2 then 400 steps/rev, and so on
  //use microstep = 1 on  motor

  int microStep = 2;
  int stepsPerRev = 200;
  int gearRatio = 50;

  //compare prevAngle to curAngle to set direction in which we move
  int encoderTarget = curAngle * 45.51111;
  int encoderPosition = encoder.getPositionSPI(14);
  // int stepCounter = 55;
  int encoderDiff = encoderTarget - encoderPosition;
  int sign = 1;

  if(encoderDiff >= 0.0){
      digitalWrite(dirPin, HIGH);
      sign = 1;
  }
  else{
      digitalWrite(dirPin, LOW);
      sign = -1;
}
    encoderDiff = sign * encoderDiff;

  while (encoderDiff >  10) {    // tolerance for the diff
      digitalWrite(stepPin, HIGH);
      delayMicroseconds(300);
      digitalWrite(stepPin, LOW);
      delayMicroseconds(300);
      encoderPosition = encoder.getPositionSPI(14);
      encoderDiff = encoderTarget - encoderPosition;
      if(encoderDiff >= 0.0){
          digitalWrite(dirPin, HIGH);
          sign = 1;
      }
      else{
          digitalWrite(dirPin, LOW);
          sign = -1;
      }
      encoderDiff = sign * encoderDiff;
  }

}

void MovingSteppersLib::moveJ4( double curAngle){
  //moves the J4 stepper motor
  // motor has a gearRatio of 14, so 14:1 speed reducer
  //microstepping 1 means 200 steps per revolution
  //microstepping 2 then 400 steps/rev, and so on
  //use microstep = 2 on  motor
  int microStep = 2;
  int stepsPerRev = 200;
  int gearRatio = 14;

  //compare prevAngle to curAngle to set direction in which we move
  int encoderTarget = curAngle * 45.51111;
  int encoderPosition = encoder.getPositionSPI(14);
  // int stepCounter = 55;
  int encoderDiff = encoderTarget - encoderPosition;
  int sign = 1;

  if(encoderDiff >= 0.0){
      digitalWrite(dirPin, HIGH);
      sign = 1;
  }
  else{
      digitalWrite(dirPin, LOW);
      sign = -1;
  }
  encoderDiff = sign * encoderDiff;

  while (encoderDiff >  10) {    // tolerance for the diff
      digitalWrite(stepPin, HIGH);
      delayMicroseconds(300);
      digitalWrite(stepPin, LOW);
      delayMicroseconds(300);
      encoderPosition = encoder.getPositionSPI(14);
      encoderDiff = encoderTarget - encoderPosition;
      if(encoderDiff >= 0.0){
          digitalWrite(dirPin, HIGH);
          sign = 1;
      }
      else{
          digitalWrite(dirPin, LOW);
          sign = -1;
      }
      encoderDiff = sign * encoderDiff;
  }

}

void MovingSteppersLib::moveJ5(double curAngle){
  //moves the J5 stepper motor
  // motor has a gearRatio of 1, so 1:1 speed reducer
  //microstepping 1 means 200 steps per revolution
  //microstepping 2 then 400 steps/rev, and so on
  //use microstep = 2 on  motor
  int microStep = 4; // TODO: change to actual desired miro step. Current setup (as of 2/21)
                     // moves J5 the correct curAngle when using  function
  int stepsPerRev = 200;
  int gearRatio = 1;

  //compare prevAngle to curAngle to set direction in which we move
  int encoderTarget = curAngle * 45.51111;
  int encoderPosition = encoder.getPositionSPI(14);
  // int stepCounter = 55;
  int encoderDiff = encoderTarget - encoderPosition;
  int sign = 1;

  if(encoderDiff >= 0.0){
      digitalWrite(dirPin, HIGH);
      sign = 1;
  }
  else{
      digitalWrite(dirPin, LOW);
      sign = -1;
  }
  encoderDiff = sign * encoderDiff;

  while (encoderDiff >  10) {    // tolerance for the diff
      digitalWrite(stepPin, HIGH);
      delayMicroseconds(300);
      digitalWrite(stepPin, LOW);
      delayMicroseconds(300);
      encoderPosition = encoder.getPositionSPI(14);
      encoderDiff = encoderTarget - encoderPosition;
      if(encoderDiff >= 0.0){
          digitalWrite(dirPin, HIGH);
          sign = 1;
      }
      else{
          digitalWrite(dirPin, LOW);
          sign = -1;
      }
      encoderDiff = sign * encoderDiff;
  }

}

void MovingSteppersLib::moveJ6(double curAngle){
  //moves the J6 stepper motor
  // motor has a gearRatio of 19, so 19:1 speed reducer
  //microstepping 1 means 200 steps per revolution
  //microstepping 2 then 400 steps/rev, and so on
  //use microstep = 2 on  motor
  int microStep = 2;
  int stepsPerRev = 200;
  int gearRatio = 19;

  //compare prevAngle to curAngle to set direction in which we move
  int encoderTarget = curAngle * 45.51111;
  int encoderPosition = encoder.getPositionSPI(14);
  // int stepCounter = 55;
  int encoderDiff = encoderTarget - encoderPosition;
  int sign = 1;

  if(encoderDiff >= 0.0){
      digitalWrite(dirPin, HIGH);
      sign = 1;
  }
  else{
      digitalWrite(dirPin, LOW);
      sign = -1;
  }
  encoderDiff = sign * encoderDiff;

  while (encoderDiff >  10) {    // tolerance for the diff
      digitalWrite(stepPin, HIGH);
      delayMicroseconds(300);
      digitalWrite(stepPin, LOW);
      delayMicroseconds(300);
      encoderPosition = encoder.getPositionSPI(14);
      encoderDiff = encoderTarget - encoderPosition;
      if(encoderDiff >= 0.0){
          digitalWrite(dirPin, HIGH);
          sign = 1;
      }
      else{
          digitalWrite(dirPin, LOW);
          sign = -1;
      }
      encoderDiff = sign * encoderDiff;
  }

}
