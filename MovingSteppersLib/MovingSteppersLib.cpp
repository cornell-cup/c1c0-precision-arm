#include "Arduino.h"
#include "MovingSteppersLib.h"
#include "MotorEncoderLib.h"
#include <SPI.h>

// Using `` for setup and in movement function - not sure if it's needed
// but it's possible that it fixed some problems
MovingSteppersLib::MovingSteppersLib(int stepPinIn, int dirPinIn, int encoderPinIn)
{
    // --> setup step and direction pins
    stepPin = stepPinIn;
    dirPin  = dirPinIn;
    encoderPin = encoderPinIn;
    encoder.setChipSelect(encoderPin);
    // --> set output mode for pins
    pinMode(stepPin, OUTPUT);
    pinMode(dirPin, OUTPUT);
    prevEncoder = 0.0;

    SPI.setClockDivider(SPI_CLOCK_DIV32);
    SPI.begin();
}


int MovingSteppersLib::move(double curAngle, int flag){

  // --> pass the flag from previous stepper to current stepper
  // --> don't move if there's a -1 flag raised before it
  // if (flag == -1) {
  //   return -1;
  // }

  // --> convert angle to step count for the stepper
  int encoderTarget = curAngle * 45.51111;
  int encoderPosition = encoder.getPositionSPI(14);
  int encoderDiff = encoderTarget - encoderPosition;
  // int prevEncoder = encoderPosition;
  int sign = 1;
  // int attempt = 0;

  // --> set moving direction
  if(encoderDiff >= 0){
      digitalWrite(dirPin, LOW);
      sign = 1;
  }
  else{
      digitalWrite(dirPin, HIGH);
      sign = -1;
  }
  encoderDiff = sign * encoderDiff;

  while (encoderDiff >  10 || encoderDiff < -10) {    // tolerance for the diff
      digitalWrite(stepPin, HIGH);
      delayMicroseconds(300);
      digitalWrite(stepPin, LOW);
      delayMicroseconds(300);
      encoderPosition = encoder.getPositionSPI(14);

      // --> Increase attempt count when motor is not moving and raise a flag if too many times
      //  if ((prevEncoder - encoderPosition <= 10) || (encoderPosition - prevEncoder <= 10)) {
      //    attempt += 1;
      //    if (attempt >= 5) {
      //      return -1;  // raise a flag
      //    }
      //  } else {
      //    attempt = 0;
      //  }
      encoderDiff = encoderTarget - encoderPosition;
      // prevEncoder = encoderPosition;

      // --> check direction every iteration
      // if(encoderDiff >= 0){
      //     digitalWrite(dirPin, HIGH);
      //     sign = 1;
      // }
      // else{
      //     digitalWrite(dirPin, LOW);
      //     sign = -1;
      // }
      // encoderDiff = sign * encoderDiff;
  }

  return 1;
}
