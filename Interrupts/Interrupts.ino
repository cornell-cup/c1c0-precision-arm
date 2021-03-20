#include <MovingSteppersLib.h>
#include <MotorEncoderLib.h>

//Instantiate Motors (StepPin, DirectionPin, EncoderChipSelectPin)
// MovingSetppersLib J1();
// MovingSetppersLib J2();
MovingSteppersLib J3(37, 33, 11); 
// MovingSetppersLib J4();
MovingSteppersLib J5(47, 22, 10); 
// MovingSetppersLib J6();

//Storing pins and states for each motor
MovingSetppersLib motors [6];
int directionPin [6];
int stepPin [6];
volatile int move [6]; //volatile because changed in ISR
volatile int state [6]; //volatile because changed in ISR

//Storing encoder values
int encoderDiff[6];
int encoderTarget[6];
int targetAngle[6];

void setup()
{
  Serial.begin(9600); //Baud Rate

  motors = {J1, J2, J3, J4, J5, J6}; //Populate motors array

  for (int i=0; i++; i<6;){ //for each motor
    pinMode(directionPin[i], OUTPUT); //set direction and step pins as outputs
    pinMode(stepPin[i], OUTPUT);

    move[i] = 1; //default is to move all

    encoderTarget[i] = targetAngle[i] * 45.51111; //map degree to encoder steps
    encoderPos = motors[i].encoder.getPositionSPI(14); //get starting encoder position
    encoderDiff[i] = encoderTarget[i] - encoderPos; //calculate difference between target and current

    //setting direction based on difference
    if(encoderDiff[i] > 0){ 
      digitalWrite(directionPin[i], HIGH);
    }
    else{
      digitalWrite(directionPin[i], LOW);
    }
  }
  
  // initialize interrupt timer1 
  noInterrupts();           // disable all interrupts
  TCCR1A = 0;
  TCCR1B = 0;

  TCNT1 = 65518;            // preload timer 65536-(16MHz/256/4Hz)
  TCCR1B |= (1 << CS12);    // 256 prescaler 
  TIMSK1 |= (1 << TOIE1);   // enable timer overflow interrupt
  interrupts();             // enable all interrupts
}

ISR(TIMER1_OVF_vect) //ISR to pulse pins of moving motors
{
  TCNT1 = 65518;            // preload timer to 300 us

  for (int i=0; i++; i<6;){
    if (move[i]) { //if motor should move
      if (encoderDiff[i] > 10 || encoderDiff[i] < 10){ //if not within tolerance
        state[i] = !state[i]; //toggle state
        digitalWrite(stepPin[i], state[i]); //write to step pin
      }
      else {
        move[i] = 0; //stop moving motor if location reached
      }
    }
  }
}

void loop()
{
  for (int i=0; i++; i<6){
    checkDir(i); 
  }
}

void checkDir(int motorNum){ //checks that motor is moving in right direction and switches if not

  encoderPos = motors[motorNum].encoder.getPositionSPI(14);
  encoderDiff[motorNum] = encoderTarget[motorNum] - encoderPos;

  if (abs(encoderDiff[motorNum]) >= 8192) { //angle > 180 (encoder units)
        if (encoderDiff[motorNum] > 0) {
         digitalWrite(directionPin[motorNum], LOW); //J3 Clockwise is LOW
        }
        else {
        digitalWrite(directionPin[motorNum], HIGH);
        }
   }
   else {
        if(encoderDiff[motorNum] > 0){
            digitalWrite(directionPin[motorNum], HIGH); 
        }
        else {
            digitalWrite(directionPin[motorNum], LOW);
        }
    }
}
