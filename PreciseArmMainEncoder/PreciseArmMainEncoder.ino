#include "MovingSteppersLib.h"
#include "joint.h"
#include "R2Protocol.h"

// use interrupts file for jetson to arduino communicaiton

#define MAX_ENCODER_VAL 16383

/* PROBLEMS LIST
  1. DO NOT HAVE TWO MOTORS HAVE SAME DIRECTION OR STEP PINS AS ANOTHER MOTOR EVERRRRR IT MESSES UP CODE
*/

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

int motorIndex;
int input;
bool moving[] = {false, false, false, false, false, false};

int8_t data_angles[] = {0,0,0,0,0,0};

motor_t motors1[] = {{.pulse_pin = s0, .dir_pin = d0},
                     {.pulse_pin = s1, .dir_pin = d1},
                     {.pulse_pin = s2, .dir_pin = d2},
                     {.pulse_pin = s3, .dir_pin = d3},
                     {.pulse_pin = s4, .dir_pin = d4},
                     {.pulse_pin = s5, .dir_pin = d5}};

float targetAngles[] = {0,0,0,0,0,0};

//TODO MAX/MIN ANGLE TRUNCATION DOESNT WORK ON FIRST ANGLE SEE BELOW
encoder_t encoders[] = {{.cs = c0, .resolution = 14, .correctDir = 0, .target_angle = 0, .max_angle = 35, .min_angle = -35},
                        {.cs = c1, .resolution = 14, .correctDir = 0, .target_angle = 0, .max_angle = 50, .min_angle = -50},
                        {.cs = c2, .resolution = 14, .correctDir = 1, .target_angle = 0, .max_angle = 90, .min_angle = -90},
                        {.cs = c3, .resolution = 14, .correctDir = 0, .target_angle = 0, .max_angle = 90, .min_angle = -90},
                        {.cs = c4, .resolution = 14, .correctDir = 0, .target_angle = 0, .max_angle = 90, .min_angle = -90},
                        {.cs = c5, .resolution = 14, .correctDir = 0, .target_angle = 0, .max_angle = 90, .min_angle = -90}};


// Jetson to Arduino Set up
// uint16_t checksum;
// char type[5];
// uint8_t data[6];
// uint32_t data_len = 6;
// uint8_t send_buf[10];
// uint8_t receive_buf[256];

// Arduino to Jetson R2
uint16_t encoder_angles[] = {10, 20, 30, 40, 50, 60};
uint8_t encoder_anglesB8[12];
int k;
#define NUM_MOTORS 7
#define DATA_SIZE (NUM_MOTORS * 2)
#define MAX_BUFFER_SIZE (R2P_HEADER_SIZE + DATA_SIZE)
uint8_t recv_buffer[MAX_BUFFER_SIZE];
uint8_t send_buffer[MAX_BUFFER_SIZE];
uint16_t checksum;
char type[5];
uint8_t data[DATA_SIZE];
uint32_t data_len;

volatile int counter = 0;
volatile int fill_serial_buffer = false;

// Storing pins and states for each motor
MovingSteppersLib motors[6]{{s0, d0, c0}, {s1, d1, c1}, {s2, d2, c2}, {s3, d3, c3}, {s4, d4, c4}, {s5, d5, c5}}; // Instantiate Motors (StepPin, DirectionPin, EncoderChipSelectPin)
int stepPin[6] = {s0, s1, s2, s3, s4, s5};
int directionPin[6] = {d0, d1, d2, d3, d4, d5};
volatile int move[6];  // volatile because changed in ISR
volatile int state[6]; // volatile because changed in ISR

int reversed[6] = {0, 1, 1, 1, 1, 0}; // motors that have encoders facing the wrong way must pick direction changes slightly differently (opposite of normal)

// Storing encoder values
volatile float encoderDiff[6];   // units of encoder steps
volatile float encoderTarget[6]; // units of encoder steps
volatile float targetAngle[6];   // units of degrees
float encoderPos[6];             // units of encoder steps

volatile int nottolerant; // motor not within expected position

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
    Serial.println("Precise Arm Begin");
    delay(1000);
    reset_input_buffer();

    // Only uncomment when you want to zero the encoders
    Serial.println("setting 0");

    //todo no hard code range
    for(int i = 0; i < 6; i++){
      setZeroSPI(&encoders[i]);
    }
    Serial.println("set 0");

    for (int i = 0 ; i < sizeof(motors1) ; i++){
      init_motor(&motors1[i]);
    }
    Serial.println("begin move");

    // initialize interrupt timer1
    noInterrupts(); // disable all interrupts
    TCCR1A = 0;
    TCCR1B = 0;

    TCNT1 = 65518;          // preload timer 65536-(16MHz/256/4Hz)
    TCCR1B |= (1 << CS12);  // 256 prescaler
    TIMSK1 |= (1 << TOIE1); // enable timer overflow interrupt
    interrupts();           // enable all interrupts

    Serial.println("Exited setup");
}

ISR(TIMER1_OVF_vect) // ISR to pulse pins of moving motors
{
    TCNT1 = 65518;             // preload timer to 300 us
    fill_serial_buffer = true; // check

    //worry about multiple encoders later, mayb put JX_encoder.correctPos in an array
    for (int i = 0; i < 6; i++)
    {
      // getPositionSPI(&encoders[i]);
      if(!encoders[i].correctPos){
        //if target angle greater then move up, otherwise move down 
        //todo later can flip > based on .correctDir
        if(encoders[i].correctDir){
          step_motor(&motors1[i], encoders[i].target_angle < encoders[i].current_angle);
        }
        else{
            step_motor(&motors1[i], encoders[i].target_angle > encoders[i].current_angle);
        }
      } 
    }
}

void update_encoder_angles()
{
    for (k = 0; k < 6; k++)
    {
        encoder_angles[k] = (uint16_t)motors[k].encoder.getPositionSPI(14) / 45.1111; // how to convert to char and how many digits to round to
        // if you get the error message (aka encoder isn't connected), set to zero
        if (encoder_angles[k] == 1452)
        {
            encoder_angles[k] = 0;
        }
    }
}

void send(char type[5], const uint8_t *data, uint32_t data_len, uint8_t *send_buffer)
{
    uint32_t written = r2p_encode(type, data, data_len, send_buffer, 256);
    Serial1.write(send_buffer, written);
    // Serial.println("Bytes written: " + String(written));
    for (int i = 0; i < data_len; i++)
    {
        // Serial.println(data[i]);
    }
}

void loop()
{


  // ---------------------------------------------------------------------------------------------
  // Manual Keyboard Control
  // ---------------------------------------------------------------------------------------------
  if (Serial.available() > 0) {
    String inputStr = Serial.readStringUntil('\n');  // Read until newline character
    
    // Select motor based on input (e.g., "j1", "j2", etc.)
    if ((inputStr.startsWith("j") || inputStr.startsWith("J")) && inputStr.length() == 2) {
      motorIndex = inputStr[1] - '0' - 1;  // Get motor number from string (e.g., '0' to '5')
      //Serial.println(motorIndex);
      //moving[motorIndex] = true;
      Serial.print("Input received from J");
      Serial.println(motorIndex + 1);
    }
    else{
      input = inputStr.toInt();
      Serial.print("Input received from J");
      setTargetAngle(&encoders[motorIndex], input);
      getPositionSPI(&encoders[motorIndex]);
      Serial.print(motorIndex + 1);
      Serial.print(": ");
      Serial.println(encoders[motorIndex].target_angle);
    }
    // switched kill code to 99
    if (inputStr.length() == 0 || inputStr == "99" || input >= 180 || input <= -180) {
      encoders[motorIndex].target_angle = encoders[motorIndex].current_angle;
      moving[motorIndex] = false;
      Serial.println("movement terminated");
      return;  // Avoid repeated processing of 0 if no valid command is given
    } 
  }

  // ---------------------------------------------------------------------------------------------
  // Motor Move/Direction Determination Code
  // ---------------------------------------------------------------------------------------------
  // Check motor moving conditions
  for(int i = 0; i < 6; i ++) {
    if(encoders[i].current_angle >= encoders[i].target_angle - 5  && encoders[i].current_angle <= encoders[i].target_angle + 5){
      encoders[i].correctPos = 1;
      moving[i] = false;
    }
    else{
      encoders[i].correctPos = 0;
      moving[i] = true;
    }
  }

  if (moving[motorIndex]){
    Serial.print("current J");
    encoders[motorIndex].target_angle = input;
    getPositionSPI(&encoders[motorIndex]);
    Serial.print(motorIndex + 1);
    Serial.print(": ");
    Serial.println(encoders[motorIndex].current_angle);
  }


  // ---------------------------------------------------------------------------------------------
  // Jetson to Arduino Communication
  // ---------------------------------------------------------------------------------------------
  if (Serial1.available() > 0) {
    Serial.println("Receiving command");
    Serial1.readBytes(recv_buffer, MAX_BUFFER_SIZE);
    if (r2p_decode(recv_buffer, MAX_BUFFER_SIZE, &checksum, type, data, &data_len))
    {
      Serial.println("message received");
      if (!strcmp(type, "PRMR"))
      {
        Serial.println("current angles requested");
      }
      else if (!strcmp(type, "PRM"))
      {
        Serial.println("angles commanded");
        int ind = 0;
        for (int i = 1; i < data_len; i=i+2)
        {
          Serial.print("J");
          Serial.print(ind+1);
          Serial.print(" Angle: ");
          Serial.println(data[i]);
          data_angles[ind] = data[i];
          ind++;
        }
        // TODO: function to change angles based on input array
        changeAngles(data_angles);
      }
    }
  }
}

void changeAngles(uint8_t data[])
{
    for (int i = 0; i < 3; i++)
    {
      if (encoders[i].target_angle != data[i]) {
        Serial.print("Target ");
        Serial.println(data[i]);
        Serial.print("Current: ");
        Serial.println(encoders[i].current_angle);
        setTargetAngle(&encoders[i], data[i]);
        getPositionSPI(&encoders[i]);
        moving[i] = true;
        encoders[i].correctPos = 0;
      }
        // if (targetAngle[i] != data[i])
        // {
        //     targetAngle[i] = data[i];
        //     encoderTarget[i] = targetAngle[i] * 45.51111 * 360 / 255;
        //     encoderPos[i] = motors[i].encoder.getPositionSPI(14);
        //     encoderDiff[i] = encoderTarget[i] - encoderPos[i];
        //     move[i] = 1;
        // }
    }
}

