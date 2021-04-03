#include <MovingSteppersLib.h>
#include <MotorEncoderLib.h>

#define MAX_ENCODER_VAL 16383

//Storing pins and states for each motor
MovingSteppersLib motors[6] {{31,37,10},{23,27,11},{37, 33, 11},{0,0,0},{47, 22, 10},{0,0,0}};  //Instantiate Motors (StepPin, DirectionPin, EncoderChipSelectPin)
int stepPin[6] = {31,23,0,0,0,0}; 
int directionPin[6] = {37,27,0,0,0,0}; 
volatile int move [6]; //volatile because changed in ISR
volatile int state [6]; //volatile because changed in ISR

int reversed[6] = {1, 0, 0, 1, 0, 0}; // motors that have encoders facing the wrong way must pick direction changes slightly differently (opposite of normal)

//Storing encoder values
float encoderDiff[6];  // units of encoder steps
float encoderTarget[6];  // units of encoder steps
float targetAngle[6];   // units of degrees
float encoderPos;   // units of encoder steps

volatile int nottolerant; // motor not within expected position

void setup()
{
  Serial.begin(9600); //Baud Rate

  //motors[1].encoder.setZeroSPI(11);
  for (int i=0; i<6; i++){ //for each motor

    targetAngle[i] = 0;   // used for testing, this will be an input from object detection
    targetAngle[0] = 30;
    targetAngle[1] = 90;
    
    pinMode(directionPin[i], OUTPUT); //set direction and step pins as outputs
    pinMode(stepPin[i], OUTPUT);

    move[i] = 0; //default is to move none
   
    move[0] = 1;
    move[1] = 1;
   
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
  for (int i=0; i<6; i++){

    nottolerant = abs(encoderDiff[i]) > 10 && ((abs(encoderDiff[i]) + 10) < (MAX_ENCODER_VAL + encoderTarget[i]));
    
    if (move[i]) { //if motor should move
      if (nottolerant){ //if not within tolerance
        state[i] = !state[i]; //toggle state
        digitalWrite(stepPin[i], state[i]); //write to step pin
      }
      else {
        move[i] = 0; //stop moving motor if location reached
      }
    }
  }
  //Serial.println(motors[1].encoder.getPositionSPI(14));
}

void loop()
{
  for (int i=0; i<6; i++){
    if (i == 0) {
        checkDirThroughZero(i); //J1
    }
    else {
        checkDirLongWay(i); //J2
    }
  }
}

void checkDirLongWay(int motorNum){ //checks that motor is moving in right direction and switches if not

  encoderPos = motors[motorNum].encoder.getPositionSPI(14);
  
  if ((MAX_ENCODER_VAL - 10 < encoderPos) && (encoderPos < MAX_ENCODER_VAL)) {
    encoderPos = 0;
  }
  
  encoderDiff[motorNum] = encoderTarget[motorNum] - encoderPos;

  if(encoderDiff[motorNum] > 0){
      digitalWrite(directionPin[motorNum], !reversed[motorNum]); 
  }
  else {
      digitalWrite(directionPin[motorNum], reversed[motorNum]);
  }
}


void checkDirThroughZero(int motorNum){ //checks that motor is moving in right direction and switches if not

  encoderPos = motors[motorNum].encoder.getPositionSPI(14);
  encoderDiff[motorNum] = encoderTarget[motorNum] - encoderPos;

  if (abs(encoderDiff[motorNum]) >= 8192) { //angle > 180 (encoder units)
        if (encoderDiff[motorNum] > 0) {
         digitalWrite(directionPin[motorNum], reversed[motorNum]); //J3 Clockwise is LOW
        }
        else {
        digitalWrite(directionPin[motorNum], !reversed[motorNum]);
        }
   }
   else {
        if(encoderDiff[motorNum] > 0){
            digitalWrite(directionPin[motorNum], !reversed[motorNum]); 
        }
        else {
            digitalWrite(directionPin[motorNum], reversed[motorNum]);
        }
    }
}
