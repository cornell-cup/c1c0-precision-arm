#include "StepperLib.h"
#include "R2Protocol.h"

// 10/5/23 ISSUES:
// 1. (RESOLVED?) Incrementing and decrementing of stepsTaken is not present in blocks marked out for not using the encoder
// 2. (RESOLVED?) Stopping of motor once it hits the target angle is questionable
// Wiring note: OPTO on stepper driver is 5V; stepper driver voltage is 24V
// Bug fix at start of checkDirLongWay: said i instead of motorNum

// R2Protocol Definitions
// Jetson to Arduino r2p decode constants
#define NUM_MOTORS 7
#define DATA_SIZE (NUM_MOTORS * 2)
#define MAX_BUFFER_SIZE (R2P_HEADER_SIZE + DATA_SIZE)
uint8_t recv_buffer[MAX_BUFFER_SIZE];
uint8_t send_buffer[MAX_BUFFER_SIZE];
uint16_t checksum;
char type[5];
uint8_t data[DATA_SIZE];
uint32_t data_len;

#define MAX_ENCODER_VAL 16383
#define STEPS_PER_REV 400
float gearRatios[NUM_MOTORS] = {20, 50, 50, 14 * 14 / 5, 60.96, 19, 100};

// Step (pulse) pins
int s0 = 49;
int s1 = 24;
int s2 = 46;
int s3 = 40; // changed from 40
int s4 = 37;
int s5 = 34;

// Direction pins
int d0 = 48;
int d1 = 42;
int d2 = 45;
int d3 = 39;
int d4 = 36;
int d5 = 33;

// Chip select pins
int c0 = 47;
int c1 = 9;
int c2 = 41;
int c3 = 38;
int c4 = 35;
int c5 = 32;

// End effector pin
int endEffectorPin = 13;
StepperLib J1 = StepperLib(s0,d0,CCW,20,STEPS_PER_REV);
// Storing pins and states for each motor
int stepPin[NUM_MOTORS] = {s0, s1, s2, s3, s4, s5, endEffectorPin};
int directionPin[NUM_MOTORS] = {d0, d1, d2, d3, d4, d5, 0};
int reversed[NUM_MOTORS] = {0, 0, 1, 0, 0, 0, 0};
volatile int state[NUM_MOTORS] = {0}; // volatile because changed in ISR

int minAngle[NUM_MOTORS] = {0};                                // units of degrees
int maxAngle[NUM_MOTORS] = {160, 100, 240, 360, 240, 360, 50}; // units of degrees
volatile int targetAngle[NUM_MOTORS] = {0};                    // units of degrees
volatile int stepsDiff[NUM_MOTORS] = {0};                      // units of encoder steps
volatile uint16_t stepsTaken[NUM_MOTORS] = {0};
volatile int motor_dir[NUM_MOTORS] = {0};

void setup()
{
  Serial.begin(115200);
  Serial2.begin(115200); // TX/RX with Jetson
  delay(1000);
  Serial.println("Precise Arm Begin");
  reset_input_buffer();

  for (int i = 0; i < NUM_MOTORS; i++)
  {
    pinMode(directionPin[i], OUTPUT); // set direction and step pins as outputs
    pinMode(stepPin[i], OUTPUT);
    digitalWrite(directionPin[i], LOW);
    digitalWrite(stepPin[i], LOW);
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
  TCNT1 = 65518; // preload timer to 300 us

  for (int i = 0; i < NUM_MOTORS; i++)
  {
    if (abs(targetAngle[i] - stepsTaken[i]) > 0)
    { // if not within tolerance
      // Write PWM signal for end effector since servo instead of stepper
      state[i] = !state[i];
      if (i < 6)
      {
        digitalWrite(stepPin[i], state[i]); // write to step pin
      }
      else
      {
        int move_val = (motor_dir[i] == 1) ? 245 : 25; // write pwm to forward or backward duty cycle
        analogWrite(stepPin[i], move_val);
      }
      if (state[i] == 1)
      {
        if (motor_dir[i])
        {
          stepsTaken[i]++;
        }
        else
        {
          stepsTaken[i]--;
        }
      }
    }
    else if (i == 6)
    {
      analogWrite(stepPin[i], 0);
    }
  }
}

void loop()
{
  for (int i = 0; i < NUM_MOTORS; i++)
  {
    checkDirLongWay(i);
  }

  if (Serial2.available() > 0)
  { // Jetson to Arduino
    Serial.println("Receiving command");
    Serial2.readBytes(recv_buffer, MAX_BUFFER_SIZE);
    if (r2p_decode(recv_buffer, MAX_BUFFER_SIZE, &checksum, type, data, &data_len))
    {
      Serial.println("message received");
      Serial.println(type);
      if (!strcmp(type, "PRMR"))
      {
        Serial.println("current angles requested");
        uint16_t new_data[6] = {};
        uint8_t stepsTakenB8[DATA_SIZE];
        convert_b16_to_b8(stepsTaken, stepsTakenB8, NUM_MOTORS);
        send("prm", stepsTakenB8, DATA_SIZE, send_buffer);
      }
      else if (!strcmp(type, "PRM"))
      {
        Serial.println("angles commanded");

        uint16_t data_final[NUM_MOTORS];
        convert_b8_to_b16(data, data_final, DATA_SIZE);
        controlMovement(data_final);
      }
    }
  }
}

void checkDirLongWay(int motorNum)
{ // checks that motor is moving in right direction and switches if not
  if (targetAngle[motorNum] - stepsTaken[motorNum] > 0)
  {
    motor_dir[motorNum] = !reversed[motorNum];
  }
  else
  {
    motor_dir[motorNum] = reversed[motorNum];
  }
  // write direction pin if not using end effector
  if (motorNum != 6)
  {
    Serial.println(motor_dir[1]);
  }
  digitalWrite(directionPin[motorNum], motor_dir[motorNum]);
}

void convert_b8_to_b16(uint8_t *databuffer, uint16_t *data, int len)
{
  int data_idx;
  for (int i = 0; i < len; i++)
  {
    data_idx = i / 2;
    if ((i & 1) == 0)
    {
      // Even
      data[data_idx] = databuffer[i] << 8;
    }
    else
    {
      // Odd
      data[data_idx] |= databuffer[i];
    }
  }
}

inline void send(char type[5], const uint8_t *data, uint32_t data_len, uint8_t *send_buffer)
{
  uint32_t written = r2p_encode(type, data, data_len, send_buffer, MAX_BUFFER_SIZE);
  Serial2.write(send_buffer, written);
  Serial.println("NUMBER OF BYTES WRITTEN! READ ME " + String(written));
}

int AngleToSteps(float motorAngle, int motorNum)
{
  return motorAngle / 360.0 * gearRatios[motorNum] * STEPS_PER_REV;
}

float StepsToAngle(int motorSteps, int motorNum)
{
  return motorSteps * 360.0 / (gearRatios[motorNum] * STEPS_PER_REV);
}

void reset_input_buffer()
{
  while (Serial2.available() > 0)
    Serial2.read();
  delay(100);
}

void controlMovement(uint16_t data[])
{
  for (int i = 0; i < NUM_MOTORS; i++)
  {
    int newAngle = data[i];
    // Clamp angle to max and minimum angles
    if (newAngle > maxAngle[i])
      newAngle = maxAngle[i];
    if (newAngle < minAngle[i])
      newAngle = minAngle[i];
    int newAngleSteps = AngleToSteps(newAngle, i);

    targetAngle[i] = newAngleSteps;

    Serial.print("J");
    Serial.print(i + 1);
    Serial.print(" target angle: ");
    Serial.print(data[i]);
    Serial.print(" Target steps: ");
    Serial.print(newAngleSteps);
    // Serial.print(targetAngle[i]);
    Serial.print(" Current steps:");
    Serial.println(stepsTaken[i]);
  }
  Serial.println();
}
