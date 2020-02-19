#ifndef MovingSteppersLib_h
#define MovingSteppersLib_h

#include "Arduino.h"

class MovingSteppersLib
{
	public:
		MovingSteppersLib(stepPinIn, dirPinIn);
        void moveJ1(dir,angle);
        void moveJ2(dir,angle);
        void moveJ3(dir,angle);
        void moveJ4(dir,angle);
        void moveJ5(dir,angle);
        void moveJ6(dir,angle);
        int stepPin;
        int dirPin;
};

#endif
