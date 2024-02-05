#ifndef StepperLib_H
#define StepperLib_H

#include "Arduino.h"

typedef enum dir_t{
    CW = 0,
    CCW = 1
};
class StepperLib
{
    public:
	    StepperLib(int stepPin, int dirPin, dir_t positive_dir);
	    StepperLib(int stepPin, int dirPin, dir_t positive_dir, float gearRatio, int steps_per_rev);
        void step_motor_no_encoder(int target_steps);
        void step_motor(dir_t dir);

        uint16_t AngleToSteps(float motorAngle);

        float StepsToAngle(uint16_t motorSteps);

    private:
        uint8_t stepPin;
        uint8_t dirPin;
        dir_t positive_dir;


        //if not using encoder
        int16_t currentSteps;
        float gearRatio;
        float steps_per_rev;

        int16_t currentStepState;
        int16_t currentDirState;


};

#endif
    