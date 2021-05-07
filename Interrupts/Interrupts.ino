#include <MovingSteppersLib.h>
#include <MotorEncoderLib.h>

#define MAX_ENCODER_VAL 16383

/* PROBLEMS LIST

  1. DO NOT HAVE TWO MOTORS HAVE SAME DIRECTION OR STEP PINS AS ANOTHER MOTOR EVERRRRR IT MESSES UP CODE

*/

// step pins 
int s0 = 0;
int s1 = 23;
int s2 = 0;
int s3 = 0;
int s4 = 0;
int s5 = 0;
// direction pins
int d0 = 0;
int d1 = 26;
int d2 = 0;
int d3 = 0;
int d4 = 0;
int d5 = 0;
//chip select pins
int c0 = 8;
int c1 = 9;
int c2 = 10;
int c3 = 11;
int c4 = 12;
int c5 = 13;

// find position with buffer
uint16_t bufpos[6];
uint16_t targs[6];
uint8_t send_buf[2];



volatile int counter = 0;

//Storing pins and states for each motor
MovingSteppersLib motors[6] {{s0,d0,c0},{s1,d1,c1},{s2,d2,c2},{s3,d3,c3},{s4,d4,c4},{s5,d5,c5}};  //Instantiate Motors (StepPin, DirectionPin, EncoderChipSelectPin)
int stepPin[6] = {s0,s1,s2,s3,s4,s5}; 
int directionPin[6] = {d0,d1,d2,d3,d4,d5};  
volatile int move [6]; //volatile because changed in ISR
volatile int state [6]; //volatile because changed in ISR

int reversed[6] = {1, 0, 0, 1, 1, 0}; // motors that have encoders facing the wrong way must pick direction changes slightly differently (opposite of normal)

//Storing encoder values
volatile float encoderDiff[6];  // units of encoder steps
volatile float encoderTarget[6];  // units of encoder steps
volatile float targetAngle[6];   // units of degrees
float encoderPos[6];   // units of encoder steps

volatile int nottolerant; // motor not within expected position

void setup()
{
  Serial.begin(9600); //Baud Rate
  Serial1.begin(9600); 

//  motors[0].encoder.setZeroSPI(c0); // zero the encoder at desired position
//  motors[1].encoder.setZeroSPI(c1);
//  motors[2].encoder.setZeroSPI(c2);
//  motors[3].encoder.setZeroSPI(c3);
  //motors[4].encoder.setZeroSPI(c4);
  //motors[5].encoder.setZeroSPI(c5);
  for (int i=0; i<6; i++){ //for each motor
     
    
    pinMode(directionPin[i], OUTPUT); //set direction and step pins as outputs
    pinMode(stepPin[i], OUTPUT);

    move[i] = 0; //default is to move none
   
    //move[0] = 1; //enable j1 // send move to the jetson and recieve the encoder directions from the jetson
//    move[1] = 1; // enable j2
//    move[2] = 1; // enable j3 
    //move[3] = 1; //enable j4
    //move[4] = 1; //enable j5
   
    encoderTarget[i] = targetAngle[i] * 45.51111; //map degree to encoder steps
    encoderPos[i] = motors[i].encoder.getPositionSPI(14); //get starting encoder position
    encoderDiff[i] = encoderTarget[i] - encoderPos[i]; //calculate difference between target and current
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

    nottolerant = abs(encoderDiff[i]) > 10 && ((abs(encoderDiff[i]) + 10) < (MAX_ENCODER_VAL + encoderTarget[i])); // 2nd condition to check if 359degrees is close enough to 0
    //nottolerant = abs(encoderDiff[i]) > 10; // we dont need the extra condition above bc we never pass through zero
    
    if (move[i]) { //if motor should move
      if (nottolerant){ //if not within tolerance
        state[i] = !state[i]; //toggle state
        digitalWrite(stepPin[i], state[i]); //write to step pin
      }
      else {
        move[i] = 0; //stop moving motor if location reached
      }
    }
  writePos();
  
  
  }
 
}

void loop()
{
  for (int i=0; i<6; i++){
     checkDirLongWay(i); 
  }
}

void checkDirLongWay(int motorNum){ //checks that motor is moving in right direction and switches if not

  encoderPos[motorNum] = motors[motorNum].encoder.getPositionSPI(14);
  
  if ((MAX_ENCODER_VAL - 1000) < encoderPos[motorNum]) {  // if motor goes past zero incorrectly, we want to make sure it moves back in the correct direction
    encoderPos[motorNum] = 0;
  }
  
  encoderDiff[motorNum] = encoderTarget[motorNum] - encoderPos[motorNum];
//  Serial.println(eencoderDiff[0]);

  if(encoderDiff[motorNum] > 0){
      digitalWrite(directionPin[motorNum], !reversed[motorNum]); 
  }
  else {
      digitalWrite(directionPin[motorNum], reversed[motorNum]);
  }
}

void writePos(){

  for (int i=0; i<6; i++){
     bufpos[i] = getPos(i); 
  }
  
//  bufpos[0] = getPos(0);
//  bufpos[1] = getPos(1); 
//  bufpos[2] = getPos(2);
//  bufpos[3] = getPos(3);
//  bufpos[4] = getPos(4);
//  bufpos[5] = getPos(5);
  
  Serial1.write(bufpos, sizeof(bufpos));
}

int getPos(int motorNum){

  send_buf[0] =  motors[motorNum].encoder.getPositionSPI(14) >> 8) & 255;  
  send_buf[1] =  motors[motorNum].encoder.getPositionSPI(14) & 255; 

  return send_buf;
  
}

void setTargets(){
  targs = Serial1.read()
  
  for (int i=0; i<6; i++){
     targetAngle[i] = targs[i]; 
  }
  
//  targetAngle[0] = targs[0];
//  targetAngle[1] = targs[1];
//  targetAngle[2] = targs[2];
//  targetAngle[3] = targs[3];
//  targetAngle[4] = targs[4];
//  targetAngle[5] = targs[5];
}
