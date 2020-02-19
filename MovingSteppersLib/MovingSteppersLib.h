#ifndef MovingSteppersLib_h
#define MovingSteppersLib_h

#include "Arduino.h"

class MovingSteppersLib
{
	public:
		MovingSteppersLib(int stepPinIn, int dirPinIn);
        void moveJ1(int dir, double angle);
        void moveJ2(int dir, double angle);
        void moveJ3(int dir, double angle);
        void moveJ4(int dir, double angle);
        void moveJ5(int dir, double angle);
        void moveJ6(int dir, double angle);
        int stepPin;
        int dirPin;
};

#endif
