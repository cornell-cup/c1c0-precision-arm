#include "Arduino.h"
#include "MovingSteppersLib.h"
#include "MotorEncoderLib.h"
#include <SPI.h>

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


int MovingSteppersLib::move(double curAngle, int flag){
  //moves the stepper motor

  if (flag == -1) {
    return -1;
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
      // Increase attempt count when motor is not moving and raise a flag if too many times
       if ((prevEncoder - encoderPosition >= 10) || (encoderPosition - prevEncoder >= 10)) {
         attempt += 1;
         if (attempt >= 5) {
           return -1;  // raise a flag
         }
       } else {
         attempt = 0;
       }
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

  return 1;
}
