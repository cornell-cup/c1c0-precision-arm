#include "StepperLib.h"

StepperLib::StepperLib(int stepPin, int dirPin, dir_t positive_dir) :
stepPin(stepPin),
dirPin(dirPin),
positive_dir(positive_dir),

currentSteps(0),
gearRatio(0),
steps_per_rev(0),

currentStepState(0)
{
    this->currentDirState = this->positive_dir;
    pinMode(this->stepPin, OUTPUT);
    pinMode(this->dirPin, OUTPUT);
    digitalWrite(this->stepPin,currentStepState);
    digitalWrite(this->stepPin,currentDirState);
}

StepperLib::StepperLib(int stepPin, int dirPin, dir_t positive_dir, float gearRatio, int steps_per_rev)
    : StepperLib(stepPin, dirPin, positive_dir)
{
    this->gearRatio = steps_per_rev;
    this->steps_per_rev = steps_per_rev;
}

uint16_t StepperLib::AngleToSteps(float motorAngle)
{
    return motorAngle / 360.0 * (this->gearRatio * this->steps_per_rev);
}

float StepperLib::StepsToAngle(uint16_t motorSteps)
{
    return motorSteps * 360.0 / (this->gearRatio * this->steps_per_rev);
}


void StepperLib::step_motor(dir_t dir){
    if(dir != this->currentDirState){
        this->currentDirState = dir;
        digitalWrite(this->dirPin,dir);
    }
    this->currentStepState = !this->currentStepState;
    digitalWrite(this->stepPin,this->currentStepState);
}

void StepperLib::step_motor_no_encoder(int target_steps){

    //Calculate steps to go
    int16_t steps_diff = target_steps - this->currentSteps;
    dir_t dir = (steps_diff > 0) ? this->positive_dir : !this->positive_dir;
    step_motor(dir);

    //Full step has occurred
    if(this->currentStepState == 0){
        //Update steps taken
        if(dir == positive_dir)
            this->currentSteps++;
        else
            this->currentSteps--;
    }
}