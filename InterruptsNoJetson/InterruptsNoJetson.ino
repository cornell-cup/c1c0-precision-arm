#include <MovingSteppersLib.h>
#include <MotorEncoderLib.h>
//#include <R2Protocol.h>

// This file is used for testing purposes
// You manually set the target angles in the setup() instead of reading values from object detection
#define MAX_ENCODER_VAL 16383

/* PROBLEMS LIST
  1. DO NOT HAVE TWO MOTORS HAVE SAME DIRECTION OR STEP PINS AS ANOTHER MOTOR EVERRRRR IT MESSES UP CODE
*/
// J4 rotates great until it gets to 90 degrees

// step (pulse) pins 
int s0 = 49;
int s1 = 8;
int s2 = 43;
int s3 = 40;
int s4 = 37;
int s5 = 34;
// direction pins
int d0 = 48;
int d1 = 10;
int d2 = 42;
int d3 = 39;
int d4 = 36;
int d5 = 33;
//chip select pins
int c0 = 47;
int c1 = 9;
int c2 = 41;
int c3 = 38;
int c4 = 35;
int c5 = 32;


int i = 0; 
volatile int counter = 0;
volatile int fill_serial_buffer = false;

//Storing pins and states for each motor
MovingSteppersLib motors[6] {{s0,d0,c0},{s1,d1,c1},{s2,d2,c2},{s3,d3,c3},{s4,d4,c4},{s5,d5,c5}};  //Instantiate Motors (StepPin, DirectionPin, EncoderChipSelectPin)
int stepPin[6] = {s0,s1,s2,s3,s4,s5}; 
int directionPin[6] = {d0,d1,d2,d3,d4,d5};  
volatile int move [6]; //volatile because changed in ISR
volatile int state [6]; //volatile because changed in ISR

int reversed[6] = {0, 0, 0, 1, 0, 0}; // motors that have encoders facing the wrong way must pick direction changes slightly differently (opposite of normal)

//Storing encoder values
volatile float encoderDiff[6];  // units of encoder steps
volatile float encoderTarget[6];  // units of encoder steps
volatile float targetAngle[6];   // units of degrees
float encoderPos[6];   // units of encoder steps

volatile int nottolerant; // motor not within expected position

void reset_input_buffer() {
  while (Serial1.available() > 0 ) Serial1.read();
  delay(100);
}

void setup()
{
  Serial.begin(115200); //Baud Rate
  Serial1.begin(115200); 
  delay(1000);
  reset_input_buffer();
  
// Only uncomment when you want to zero the encoders
//  motors[0].encoder.setZeroSPI(c0); 
  motors[1].encoder.setZeroSPI(c1);  
//  motors[2].encoder.setZeroSPI(c2);     
//  motors[3].encoder.setZeroSPI(c3);
//  motors[4].encoder.setZeroSPI(c4);
//  motors[5].encoder.setZeroSPI(c5);
  for (int i=0; i<6; i++){ //for each motor
  // initialized to something that isn't valid
   targetAngle[i] = -1; 
   targetAngle[1] = 100; 


//    targetAngle[1] = 80;
   //targetAngle[2] = 200;
   
//   targetAngle[1] = 100;
//   targetAngle[2] = 90; 
////   
//   targetAngle[1] = 130;
//   targetAngle[2] = 40; 
////   
  // targetAngle[1] = 80;
//   targetAngle[2] = 10;  
   
  //  targetAngle[3] = 300;
   //targetAngle[4] = 100; 
    
    pinMode(directionPin[i], OUTPUT); //set direction and step pins as outputs
    pinMode(stepPin[i], OUTPUT);

   
    move[i] = 0; //default is to move none
   
//    move[0] = 1; //enable j1 // send move to the jetson and recieve the encoder directions from the jetson
    move[1] = 1; // enable j2
//    move[2] = 1; // enable j3 
//    move[3] = 1; //enable j4
//    move[4] = 1; //enable j5
//    move[5] = 1; // enable j6
 
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
  fill_serial_buffer = true; //check
  
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
  }
  
}




void loop()
{
  Serial.println(motors[1].encoder.getPositionSPI(14));
  Serial.println(encoderTarget[1]);
  Serial.println(move[1]);
  for (int i=0; i<6; i++){
     checkDirLongWay(i); 
  }
  //delay(2500);
}


void checkDirLongWay(int motorNum){ //checks that motor is moving in right direction and switches if not

  encoderPos[motorNum] = motors[motorNum].encoder.getPositionSPI(14);
  if (encoderPos[motorNum] == 65535){
    move[motorNum] = 0; //stop moving if encoder reads error message
  }
  
  if ((MAX_ENCODER_VAL - 1000) < encoderPos[motorNum]) {  // if motor goes past zero incorrectly, we want to make sure it moves back in the correct direction
    encoderPos[motorNum] = 0;
  }
  
  encoderDiff[motorNum] = encoderTarget[motorNum] - encoderPos[motorNum];

  if(encoderDiff[motorNum] > 0){
      digitalWrite(directionPin[motorNum], !reversed[motorNum]); 
  }
  else {
      digitalWrite(directionPin[motorNum], reversed[motorNum]);
  }
  
}
