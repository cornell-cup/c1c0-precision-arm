#include "Arduino.h"
#include "StepperLib.h"
#include "EncoderLib.h"
#include <SPI.h>

// Using `` for setup and in movement function - not sure if it's needed
// but it's possible that it fixed some problems

StepperEncoderLib::StepperEncoderLib(int step_pin, int dir_pin, int cs_pin, dir_t positive_dir){
    this->encoder = EncoderLib();
    this->encoder.setChipSelect(cs_pin);
    this->stepper = StepperLib(step_pin, dir_pin, positive_dir);
    this->prevEncoder = 0.0;
    SPI.setClockDivider(SPI_CLOCK_DIV32);
    SPI.begin();
}
int StepperEncoderLib::move(double curAngle, int flag){

  // --> convert angle to step count for the stepper
  int encoderTarget = curAngle * 45.51111;
  int encoderPosition = this->encoder.getPositionSPI(14);
  int encoderDiff = encoderTarget - encoderPosition;
  // int prevEncoder = encoderPosition;
  int sign = 1;
  // int attempt = 0;

  // --> set moving direction
  if(encoderDiff > 0){
      digitalWrite(dirPin, HIGH);
      sign = 1;
  }
  else{
      digitalWrite(dirPin, LOW);
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

  }

  return 1;
}
