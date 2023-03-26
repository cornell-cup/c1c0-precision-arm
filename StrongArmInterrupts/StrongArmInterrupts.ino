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

// create Servo object to control the wrist bending
Servo bend_servo;
volatile int bend_pos;     
volatile int bend_desired_pos;  

// create Servo object to control wrist rotation
Servo spin_servo;
volatile int spin_pos;
volatile int spin_desired_pos;

#define MAX_ENCODER_VAL 16383

// Elbow Motor Encoder
int s0 = 8;     // step pin
int d0 = 10;    // direction pin
int c0 = 4;     // chip select pin

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
  Serial1.begin(38400); // TX1/RX1 

  // Jetson to Arduino r2p decode setup
  reset_input_buffer();

  // setup for servo that controls wrist bend
  bend_servo.write(75);  // sets initial position
  bend_servo.attach(7);  // attaches the servo on pin 7 to the Servo object
  bend_desired_pos = 0; // desired position

  spin_servo.write(75);
  spin_servo.attach(6);
  spin_desired_pos = 130+30;

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
      // wrist bend and spin Servo control
      positional_servo_ISR(bend_servo, bend_desired_pos);
      positional_servo_ISR(spin_servo, spin_desired_pos);
      servo_wait = 0; //resets the wait timer
  }
}

// Arduino to Jetson R2
uint16_t encoder_angles[] = {0, 0, 0}; // changed to length 3 instead of 6; set to 0 by default
uint8_t encoder_anglesB8[6];           // changed from 12 to 6
uint8_t send_buffer[256];
int k;

void update_encoder_angles(){
  encoder_angles[0] = (uint16_t) motors[0].encoder.getPositionSPI(14)/ 45.1111;  // how to convert to char and how many digits to round to
  encoder_angles[1] = (uint16_t) bend_servo.read();
  encoder_angles[2] = (uint16_t) spin_servo.read();
}

// Function for updating the position of the Servos in the ISR 
void positional_servo_ISR(Servo servo, int desired_pos) {
    volatile int current_pos = servo.read();
    if (abs(desired_pos - current_pos) < 1) {
      //servo.detach();
    }
    else if (abs(desired_pos - current_pos) >= 1) {
      if ((desired_pos - current_pos) < 0){
        servo.write(current_pos - 1);
      }  
      else if ((desired_pos - current_pos) > 0) {
        servo.write(current_pos + 1);
      }
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
      Serial.println(data_final[i]);                                 
    }
    changeAngles(data_final);
  } else {
    update_encoder_angles();                                   
    convert_b16_to_b8(encoder_angles, encoder_anglesB8, 3);     
    send("prm", encoder_anglesB8, 6, send_buffer);    
    delay(100);         
  }
}

void changeAngles(uint16_t data[]){
    if (targetAngle[0] != data[0]){
      targetAngle[0] = data[0];
      encoderTarget[0] = targetAngle[0] * 45.51111 * 360/255;
      encoderPos[0] = motors[0].encoder.getPositionSPI(14);
      encoderDiff[0] = encoderTarget[0] - encoderPos[0];
      move[0] = 1;                                                    
    }

    Serial.println("New Desired Position for Wrist Bend Servo: ");
    Serial.println(data[1]);
    bend_servo.attach(6);
    bend_desired_pos = data[1];

    Serial.println("New Desired Position for Wrist Spin Servo: ");
    Serial.println(data[2]);
    spin_servo.attach(7);
    spin_desired_pos = data[2];
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

void send(char type[5], const uint8_t* data, uint32_t data_len, uint8_t* send_buffer) {
  uint32_t written = r2p_encode(type, data, data_len, send_buffer, 256);
  Serial1.write(send_buffer, written);
  Serial.println("Bytes written: " + String(written));
  for(int i=0; i < data_len; i++){
    Serial.println(data[i]);
    //Serial.println(send_buffer[i], HEX);
 }
}