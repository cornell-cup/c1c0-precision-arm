#include <MovingSteppersLib.h>
#include <MotorEncoderLib.h>
// #include <R2Protocol.h>

// This file is used for testing purposes
// You manually set the target angles in the setup() instead of reading values from object detection
#define MAX_ENCODER_VAL 16383
// #define USING_ENCODER
#define NUM_MOTORS 6
#define STEPS_PER_REV 400
float gearRatios[NUM_MOTORS] = {20, 50, 50, 14 * 14 / 5, 60.96, 19};

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
// chip select pins
int c0 = 47;
int c1 = 9;
int c2 = 41;
int c3 = 38;
int c4 = 35;
int c5 = 32;

int i = 0;
volatile int counter = 0;
volatile int fill_serial_buffer = false;
// Storing pins and states for each motor
MovingSteppersLib motors[NUM_MOTORS]{{s0, d0, c0}, {s1, d1, c1}, {s2, d2, c2}, {s3, d3, c3}, {s4, d4, c4}, {s5, d5, c5}}; // Instantiate Motors (StepPin, DirectionPin, EncoderChipSelectPin)
int stepPin[NUM_MOTORS] = {s0, s1, s2, s3, s4, s5};
int directionPin[NUM_MOTORS] = {d0, d1, d2, d3, d4, d5};
volatile int move[NUM_MOTORS];  // volatile because changed in ISR
volatile int state[NUM_MOTORS]; // volatile because changed in ISR

int reversed[NUM_MOTORS] = {0, 0, 0, 1, 0, 0}; // motors that have encoders facing the wrong way must pick direction changes slightly differently (opposite of normal)

// Storing encoder values
#ifdef USING_ENCODER
volatile float encoderDiff[NUM_MOTORS];   // units of encoder steps
volatile float encoderTarget[NUM_MOTORS]; // units of encoder steps
volatile float targetAngle[NUM_MOTORS];   // units of degrees
float encoderPos[NUM_MOTORS];             // units of encoder steps
#else
volatile float stepsDiff[NUM_MOTORS]; // units of encoder steps
volatile int stepsTaken[NUM_MOTORS] = {0};
volatile int motor_dir[NUM_MOTORS] = {0, 0, 0, 0, 0, 0};
#endif

volatile int nottolerant; // motor not within expected position

int convertAngle(float motorAngle, int motorNum)
{
  return motorAngle / 360.0 * gearRatios[motorNum] * STEPS_PER_REV;
}

void reset_input_buffer()
{
  while (Serial1.available() > 0)
    Serial1.read();
  delay(100);
}

void setup()
{
  Serial.begin(115200); // Baud Rate
  Serial1.begin(115200);
  delay(1000);
  reset_input_buffer();

// Only uncomment when you want to zero the encoders
//  motors[0].encoder.setZeroSPI(c0);
#ifdef USING_ENCODER
  motors[1].encoder.setZeroSPI(c1);
//  motors[2].encoder.setZeroSPI(c2);
//  motors[3].encoder.setZeroSPI(c3);
//  motors[4].encoder.setZeroSPI(c4);
//  motors[5].encoder.setZeroSPI(c5);
#endif
  for (int i = 0; i < NUM_MOTORS; i++)
  { // for each motor
    // initialized to something that isn't valid
    targetAngle[i] = 0;
#define TargetDegreeAngle 90
    targetAngle[5] = convertAngle(90, 5);
    targetAngle[3] = convertAngle(90, 3);

    //    targetAngle[1] = 80;
    // targetAngle[2] = 200;

    //   targetAngle[1] = 100;
    //   targetAngle[2] = 90;
    ////
    //   targetAngle[1] = 130;
    //   targetAngle[2] = 40;
    ////
    // targetAngle[1] = 80;
    //   targetAngle[2] = 10;

    //  targetAngle[3] = 300;
    // targetAngle[4] = 100;

    pinMode(directionPin[i], OUTPUT); // set direction and step pins as outputs
    pinMode(stepPin[i], OUTPUT);

    move[i] = 0; // default is to move none

    // move[0] = 1; // enable j1 // send move to the jetson and recieve the encoder directions from the jetson
    //  move[1] = 1; // enable j2
    //      move[2] = 1; // enable j3
    move[3] = 1; // enable j4
                 //     move[4] = 1; //enable j5
    move[5] = 1; // enable j6

#ifdef USING_ENCODER
    encoderTarget[i] = targetAngle[i] * 45.51111;         // map degree to encoder steps
    encoderPos[i] = motors[i].encoder.getPositionSPI(14); // get starting encoder position
    encoderDiff[i] = encoderTarget[i] - encoderPos[i];    // calculate difference between target and current
#endif
  }

  // initialize interrupt timer1
  noInterrupts(); // disable all interrupts
  TCCR1A = 0;
  TCCR1B = 0;

  TCNT1 = 65518;          // preload timer 65536-(16MHz/256/4Hz)
  TCCR1B |= (1 << CS12);  // 256 prescaler
  TIMSK1 |= (1 << TOIE1); // enable timer overflow interrupt
  interrupts();           // enable all interrupts
}

ISR(TIMER1_OVF_vect) // ISR to pulse pins of moving motors
{
  TCNT1 = 65518;             // preload timer to 300 us
  fill_serial_buffer = true; // chec

  for (int i = 0; i < NUM_MOTORS; i++)
  {
#ifdef USING_ENCODER
    nottolerant = abs(encoderDiff[i]) > 10 && ((abs(encoderDiff[i]) + 10) < (MAX_ENCODER_VAL + encoderTarget[i])); // 2nd condition to check if 359degrees is close enough to 0
#else
    stepsDiff[i] = targetAngle[i] - stepsTaken[i];
    nottolerant = abs(stepsDiff[i]) > 0; // 2nd condition to check if 359degrees is close enough to 0
#endif
    if (move[i])
    { // if motor should move
      if (nottolerant)
      {                       // if not within tolerance
        state[i] = !state[i]; // toggle state
        // Serial.println(stepPin[i]);
        digitalWrite(stepPin[i], state[i]); // write to step pin
#ifndef USING_ENCODER
        if (state[i] == 1)
        {
          if (motor_dir[i])
            stepsTaken[i]++;
          else
            stepsTaken[i]--;
        }
#endif
      }
      else
      {
        move[i] = 0; // stop moving motor if location reached
      }
    }
  }
}

void loop()
{
  // Serial.println(stepsTaken[5]);
#ifdef USING_ENCODER
  Serial.println(motors[1].encoder.getPositionSPI(14));
  Serial.println(encoderTarget[1]);
#endif
  // Serial.println(stepsTaken[1]);
  for (int i = 0; i < NUM_MOTORS; i++)
  {
    checkDirLongWay(i);
  }
  // delay(2500);
}

void checkDirLongWay(int motorNum)
{ // checks that motor is moving in right direction and switches if not

#ifdef USING_ENCODER
  encoderPos[motorNum] = motors[motorNum].encoder.getPositionSPI(14);
  if (encoderPos[motorNum] == 65535)
  {
    move[motorNum] = 0; // stop moving if encoder reads error message
  }

  if ((MAX_ENCODER_VAL - 1000) < encoderPos[motorNum])
  { // if motor goes past zero incorrectly, we want to make sure it moves back in the correct direction
    encoderPos[motorNum] = 0;
  }

  encoderDiff[motorNum] = encoderTarget[motorNum] - encoderPos[motorNum];

  if (encoderDiff[motorNum] > 0)
#else
  stepsDiff[i] = targetAngle[i] - stepsTaken[i];
  if (stepsDiff[motorNum] > 0)
#endif
  {
    digitalWrite(directionPin[motorNum], !reversed[motorNum]);
    motor_dir[motorNum] = !reversed[motorNum];
  }
  else
  {
    digitalWrite(directionPin[motorNum], reversed[motorNum]);
    motor_dir[motorNum] = reversed[motorNum];
  }
}
