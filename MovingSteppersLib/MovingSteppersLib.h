#ifndef MovingSteppersLib_h
#define MovingSteppersLib_h

#include "Arduino.h"
#include "MotorEncoderLib.h"

class MovingSteppersLib
{
    public:
	    MovingSteppersLib(int stepPinIn, int dirPinIn, int encoderPinIn);
        int move(double curAngle, int flag);
        int stepPin;
        int dirPin;
        int encoderPin;
        double prevEncoder;
        MotorEncoderLib encoder;
};

#endif
    