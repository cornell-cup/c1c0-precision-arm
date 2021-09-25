// Jetson to Arduino Communication 
#include <MovingSteppersLib.h>
#include <MotorEncoderLib.h>

#define MAX_ENCODER_VAL 16383

/* PROBLEMS LIST

  1. DO NOT HAVE TWO MOTORS HAVE SAME DIRECTION OR STEP PINS AS ANOTHER MOTOR EVERRRRR IT MESSES UP CODE

*/

// step pins 
int s0 = 26;
int s1 = 35;
int s2 = 0;
int s3 = 0;
int s4 = 0;
int s5 = 0;
// direction pins
int d0 = 27;
int d1 = 34;
int d2 = 0;
int d3 = 0;
int d4 = 0;
int d5 = 0;
//chip select pins
int c0 = 10;
int c1 = 9;
int c2 = 0;
int c3 = 0;
int c4 = 0;
int c5 = 0;

uint8_t receive_buf[10];

//Storing pins and states for each motor
MovingSteppersLib motors[6] {{s0,d0,c0},{s1,d1,c1},{s2,d2,c2},{s3,d3,c3},{s4,d4,c4},{s5,d5,c5}};  //Instantiate Motors (StepPin, DirectionPin, EncoderChipSelectPin)
int stepPin[6] = {s0,s1,s2,s3,s4,s5}; 
int directionPin[6] = {d0,d1,d2,d3,d4,d5};  
volatile int move [6]; //volatile because changed in ISR
volatile int state [6]; //volatile because changed in ISR

int reversed[6] = {0, 1, 1, 1, 1, 0}; // motors that have encoders facing the wrong way must pick direction changes slightly differently (opposite of normal)

//Storing encoder values
volatile float encoderDiff[6];  // units of encoder steps
volatile float encoderTarget[6];  // units of encoder steps
volatile float targetAngle[6];   // units of degrees
float encoderPos[6];   // units of encoder steps

volatile int nottolerant; // motor not within expected position

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600); //Baud Rate
  Serial1.begin(9600); 
  Serial1.flush();

}

void loop() {
  // put your main code here, to run repeatedly:

}
