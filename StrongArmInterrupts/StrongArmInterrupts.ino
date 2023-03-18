#include <MovingSteppersLib.h>
#include <MotorEncoderLib.h>
#include <Servo.h>
#include "R2Protocol.h"

// R2Protocol Definitions
// Jetson to Arduino r2p decode constants
uint8_t data_buffer[22];
uint32_t data_buffer_len = 22;
uint16_t checksum; 
char type[5];
uint8_t data[6];
uint16_t data_final[3];
uint32_t data_len;

/*
// Arduino to Jetson r2p encode constants
#define MAX_BUFFER_SIZE 2048
uint32_t counter = 0;
char msg[9] = "baseball";
uint8_t msg_data_buffer[1];
uint8_t msg_send_buffer[MAX_BUFFER_SIZE];
*/

// create Servo object to control the wrist 180° movement
Servo reg_servo;
volatile int reg_pos;     
volatile int reg_desired_pos;  
volatile int reg_current_pos;

// create Servo object to control wrist rotation
Servo rot_servo;
volatile int rot_pos;
volatile int rot_desired_pos;
volatile int rot_current_pos;


#define MAX_ENCODER_VAL 16383

int s0 = 8;     // step pin
int d0 = 10;    // direction pin
int c0 = 4;     // chip select pin

int i = 0; 
volatile int counter = 0;
volatile int fill_serial_buffer = false;
volatile int servo_wait = 0;

// storing pins and states for the stepper motor
MovingSteppersLib motors[1] {{s0,d0,c0}};     // instantiate motor (StepPin, DirectionPin, EncoderChipSelectPin)
int stepPin[1] = {s0}; 
int directionPin[1] = {d0};  
volatile int move [1];    //volatile because changed in ISR
volatile int state [1];   //volatile because changed in ISR

int reversed[1] = {0}; // motors that have encoders facing the wrong way must pick direction changes slightly differently (opposite of normal)

// storing encoder values
volatile float encoderDiff[1];    // units of encoder steps
volatile float encoderTarget[1];  // units of encoder steps
volatile float targetAngle[1];    // units of degrees
float encoderPos[1];    // units of encoder steps

volatile int nottolerant; // motor not within expected position

void reset_input_buffer() {
  while (Serial1.available() > 0) {
    Serial1.read();
    delay(100);
  }
}

void redefine_encoder_zero_position(){
  // call this function to reset the encoder's zero position to be the current position of the motor
  motors[0].encoder.setZeroSPI(c0); 
}

void setup()
{
  Serial.begin(9600); // Serial monitor
  Serial1.begin(115200); // TX1/RX1

  // Jetson to Arduino r2p decode setup
  reset_input_buffer();

  // setup for regular Servos
  reg_servo.write(75);  // sets initial position
  reg_servo.attach(7);  // attaches the servo on pin 7 to the Servo object
  reg_desired_pos = 90; // desired position

  rot_servo.write(75);
  rot_servo.attach(6);
  rot_desired_pos = 90;

  // stepper motor setup
  //redefine_encoder_zero_position(); // uncomment this whenever you want to set zero position
  targetAngle[0] = 0; // max is roughly 135 if zeroed correctly 
    
  pinMode(directionPin[0], OUTPUT); //set direction and step pins as outputs
  pinMode(stepPin[0], OUTPUT);
  
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
  TCNT1 = 65518;   // preload timer to 300 us          
  fill_serial_buffer = true; //check
  
  nottolerant = abs(encoderDiff[0]) > 10 && ((abs(encoderDiff[0]) + 10) < (MAX_ENCODER_VAL + encoderTarget[0])); // 2nd condition to check if 359 degrees is close enough to 0
  //nottolerant = abs(encoderDiff[i]) > 10; // we dont need the extra condition above bc we never pass through zero
  if (move[0]) { //if motor should move
    if (nottolerant){ //if not within tolerance
      state[0] = !state[0]; //toggle state
      digitalWrite(stepPin[0], state[0]); //write to step pin
    }
    else {
      move[0] = 0; //stop moving motor if location reached
    }
  }

  servo_wait += 1;

  if (servo_wait == 150) { // used to slow down Servo movement to be more in line with stepper motor
      // regular Servo control
      reg_current_pos = reg_servo.read(); //determine the current position
      if (abs(reg_desired_pos - reg_current_pos) < 1){
        reg_servo.detach();
      }
      else if (abs(reg_desired_pos - reg_current_pos) >= 1){
        if ((reg_desired_pos - reg_current_pos) < 0){
          reg_servo.write(reg_current_pos - 1);
        }  
        else if ((reg_desired_pos - reg_current_pos) > 0){
          reg_servo.write(reg_current_pos + 1);
        }
      }
      
      // rotational wrist Servo control
      rot_current_pos = rot_servo.read(); 
      if (abs(rot_desired_pos - reg_current_pos) < 1){
        rot_servo.detach();
      }
      else if (abs(rot_desired_pos - rot_current_pos) >= 1){
        if ((rot_desired_pos - rot_current_pos) < 0){
          rot_servo.write(rot_current_pos - 1);
        }  
        else if ((rot_desired_pos - rot_current_pos) > 0){
          rot_servo.write(rot_current_pos + 1);
        }
      }
    servo_wait = 0;
  }
}


void loop() {
  checkDirLongWay(0);

  // Jetson to Arduino 
  if (Serial1.available() > 0) {
    Serial1.readBytes(data_buffer, data_buffer_len);
    r2p_decode(data_buffer, data_buffer_len, &checksum, type, data, &data_len);

    Serial.println("Data: ");
    convert_b8_to_b16(data, data_final, 6);
    for(int i = 0; i < 3; i++) {
      Serial.println(data_final[i]);                                  // this was msg_len[i] before; does changeAngles need uint8_t or uint16_t? (Was 8)
    }
    changeAngles(data_final);
  }
}

void changeAngles(uint16_t data[]){
    if (targetAngle[0] != data[0]){
      targetAngle[0] = data[0];
      encoderTarget[0] = targetAngle[0] * 45.51111 * 360/255;
      encoderPos[0] = motors[0].encoder.getPositionSPI(14);
      encoderDiff[0] = encoderTarget[0] - encoderPos[0];
      move[0] = 1;                                                    // should move come before or after?
    }

    Serial.println("New Desired Position for Servo 1: ");
    Serial.println(data[1]);
    reg_servo.attach(6);
    reg_desired_pos = data[1];

    Serial.println("New Desired Position for Servo 2: ");
    Serial.println(data[2]);
    rot_servo.attach(7);
    rot_desired_pos = data[2];
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

void convert_b8_to_b16(uint8_t *databuffer, uint16_t *data, int len) {
  int data_idx;
  for (int i=0; i < len; i++) {
    data_idx = i / 2;
    if ( (i & 1) == 0) {
      // even
      data[data_idx] = databuffer[i] << 8;
    } else {
      // odd
      data[data_idx] |= databuffer[i];
    }
  }
}

void convert_b16_to_b8(int *databuffer, uint8_t *data, int len) {
  int data_idx1;
  int data_idx2;
  for (int i = 0; i < 2*len; i+=2) {
    data[i] = (databuffer[i/2] >> 8) & 255;
    data[i+1] = (databuffer[i/2]) & 255;
  }
}