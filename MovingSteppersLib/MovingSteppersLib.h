#ifndef MovingSteppersLib_h
#define MovingSteppersLib_h

#include "Arduino.h"
#include "MotorEncoderLib.h"

class MovingSteppersLib
{
	public:
		MovingSteppersLib(int stepPinIn, int dirPinIn, int encoderPinIn);
        int moveJ1(double curAngle, int flag);
        int moveJ2(double curAngle, int flag);
        int moveJ3(double curAngle, int flag);
        int moveJ4(double curAngle, int flag);
        int moveJ5(double curAngle, int flag);
        int moveJ6(double curAngle, int flag);
        int stepPin;
        int dirPin;
        double prevEncoder;
        MotorEncoderLib encoder;
};

#endif
