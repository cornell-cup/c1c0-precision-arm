#ifndef MovingSteppersLib_h
#define MovingSteppersLib_h

#include "Arduino.h"
#include "MotorEncoderLib.h"

class MovingSteppersLib
{
	public:
		MovingSteppersLib(int stepPinIn, int dirPinIn, int encoderPinIn);
        void moveJ1(double curAngle);
        void moveJ2(double curAngle);
        void moveJ3(double curAngle);
        void moveJ4(double curAngle);
        void moveJ5(double curAngle);
        void moveJ6(double curAngle);
        int stepPin;
        int dirPin;
        double prevAngle;
        MotorEncoderLib encoder;
};

#endif
