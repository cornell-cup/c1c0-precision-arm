#ifndef StepperEncoderLib_h
#define StepperEncoderLib_h

#include "Arduino.h"
#include "EncoderLib.h"
#include "StepperLib.h"

class MovingSteppersLib
{
    public:
        StepperEncoderLib(int step_pin, int dir_pin, int cs_pin, dir_t positive_dir);
    private:
        StepperLib stepper;
        EncoderLib encoder;
        float prevEncoder;
};

#endif
    