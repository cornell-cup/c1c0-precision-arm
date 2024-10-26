#include <MovingSteppersLib.h>
#include <MotorEncoderLib.h>
#include "R2Protocol.h"

/* PROBLEMS LIST
  1. DO NOT HAVE TWO MOTORS HAVE SAME DIRECTION OR STEP PINS AS ANOTHER MOTOR EVERRRRR IT MESSES UP CODE
*/
// J4 rotates great until it gets to 90 degrees

// This file is used for testing purposes
// You manually set the target angles in the setup() instead of reading values from object detection

// R2Protocol Definitions
// Jetson to Arduino r2p decode constants
#define NUM_MOTORS 6
#define DATA_SIZE (NUM_MOTORS * 2)
#define MAX_BUFFER_SIZE (R2P_HEADER_SIZE + DATA_SIZE)
uint8_t recv_buffer[MAX_BUFFER_SIZE];
uint8_t send_buffer[MAX_BUFFER_SIZE];
uint16_t checksum;
char type[5];
uint8_t data[DATA_SIZE];
uint32_t data_len;

#define DEBUG
#define MAX_ENCODER_VAL 16383
#define STEPS_PER_REV 400
float gearRatios[NUM_MOTORS] = {20 * 4, 50, 50, 14 * 14 / 5, 60.96, 19};

// step (pulse) pins
int s0 = 49;
int s1 = 46;
int s2 = 43;
int s3 = 40;
int s4 = 37;
int s5 = 34;
// direction pins
int d0 = 48;
int d1 = 45;
int d2 = 42;
int d3 = 39;
int d4 = 36;
int d5 = 33;
// chip select pins
int c0 = 47;
int c1 = 44;
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
int flip_encoder[NUM_MOTORS] = {0, 1, 0, 0, 0, 0}

// Storing encoder values
volatile float encoderDiff[NUM_MOTORS];   // units of encoder steps
volatile float encoderTarget[NUM_MOTORS]; // units of encoder steps
volatile float targetAngle[NUM_MOTORS];   // units of degrees
float encoderPos[NUM_MOTORS];             // units of encoder steps

volatile int nottolerant; // motor not within expected position

int convertAngle(float motorAngle, int motorNum)
{
  return motorAngle / 360.0 * gearRatios[motorNum] * STEPS_PER_REV;
}

void reset_input_buffer()
{
  while (Serial2.available() > 0)
    Serial2.read();
  delay(100);
}

void setup()
{
  Serial.begin(115200); // Baud Rate
  Serial2.begin(115200);
  delay(1000);
  reset_input_buffer();

// Only uncomment when you want to zero the encoders
  // motors[0].encoder.setZeroSPI(c0); // Zero J1
  motors[1].encoder.setZeroSPI(c1); // Zero J2
  // motors[2].encoder.setZeroSPI(c2); // Zero J3
  // motors[3].encoder.setZeroSPI(c3); // Zero J4
  // motors[4].encoder.setZeroSPI(c4); // Zero J5
  // motors[5].encoder.setZeroSPI(c5); // Zero J6

  for (int i = 0; i < NUM_MOTORS; i++)
  { // for each motor
    // initialized to something that isn't valid
    targetAngle[i] = 0;

// Modify below to change motor and target angle
#define TargetDegreeAngle 30
#define motorJ 2
    targetAngle[motorJ - 1] = TargetDegreeAngle;

    pinMode(directionPin[i], OUTPUT); // set direction and step pins as outputs
    pinMode(stepPin[i], OUTPUT);

    move[i] = 0; // default is to move none

    move[0] = 0; // enable j1 
    move[1] = 0; // enable j2
    move[2] = 1; // enable j3
    move[3] = 0; // enable j4
    move[4] = 0; // enable j5
    move[5] = 0; // enable j6

    encoderTarget[i] = targetAngle[i] * 45.51111;         // map degree to encoder steps
    encoderPos[i] = motors[i].encoder.getPositionSPI(14); // get starting encoder position
    encoderDiff[i] = encoderTarget[i] - encoderPos[i];    // calculate difference between target and current
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
    nottolerant = abs(encoderDiff[i]) > 100 && ((abs(encoderDiff[i]) + 10) < (MAX_ENCODER_VAL + encoderTarget[i])); // 2nd condition to check if 359degrees is close enough to 0
    if (move[i])
    { // if motor should move
      if (nottolerant)
      {                       // if not within tolerance
        state[i] = !state[i]; // toggle state
        digitalWrite(stepPin[i], state[i]); // write to step pin
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
#ifdef DEBUG
  Serial.print("Motor J");
  Serial.println(motorJ);
  Serial.print("Encoder Position: ");
  Serial.println(encoderPos[motorJ-1]);
  Serial.print("Target: ");
  Serial.println(encoderTarget[motorJ-1]);
  Serial.print("Encoder Diff: ");
  Serial.println(encoderDiff[motorJ-1]);
  Serial.println("");
#endif
  
  for (int i = 0; i < NUM_MOTORS; i++)
  {
    checkDirLongWay(i, flip_encoder);
  }
  // delay(2500);

  // R2P Communication Code - Jetson to Arduino
  if (Serial2.available() > 0)
  { 
    Serial.println("Receiving command");
    Serial2.readBytes(recv_buffer, MAX_BUFFER_SIZE);
    if (r2p_decode(recv_buffer, MAX_BUFFER_SIZE, &checksum, type, data, &data_len))
    {
      Serial.println("message received");
      Serial.println(type);
      if (!strcmp(type, "PRMR"))
      {
        // Serial.println("current angles requested");
        // uint16_t new_data[6] = {};
        // uint8_t stepsTakenB8[DATA_SIZE];
        // convert_b16_to_b8(stepsTaken, stepsTakenB8, NUM_MOTORS);
        // send("prm", stepsTakenB8, DATA_SIZE, send_buffer);
      }
      else if (!strcmp(type, "PRM"))
      {
        Serial.println("angles commanded");

        // uint16_t data_final[NUM_MOTORS];
        // convert_b8_to_b16(data, data_final, DATA_SIZE);
        // controlMovement(data_final);
      }
    }
  }
}

void checkDirLongWay(int motorNum, int flip=0)
{ // checks that motor is moving in right direction and switches if not
  if (flip) {
    encoderPos[motorNum] = MAX_ENCODER_VAL - motors[motorNum].encoder.getPositionSPI(14);
  } else {
    encoderPos[motorNum] = motors[motorNum].encoder.getPositionSPI(14);
  }
  if (encoderPos[motorNum] == 65535)
  {
    move[motorNum] = 0; // stop moving if encoder reads error message
  }

  if ((MAX_ENCODER_VAL - 300) < encoderPos[motorNum])
  { // if motor goes past zero incorrectly, we want to make sure it moves back in the correct direction
    encoderPos[motorNum] = 0;
  }

  encoderDiff[motorNum] = encoderTarget[motorNum] - encoderPos[motorNum];

  if (encoderDiff[motorNum] > 0)
  {
    digitalWrite(directionPin[motorNum], !reversed[motorNum]);
  }
  else
  {
    digitalWrite(directionPin[motorNum], reversed[motorNum]);
  }
}
