#include <MovingSteppersLib.h>
#include <MotorEncoderLib.h>
#include <Servo.h>

Servo reg_servo;  // create servo object to control a servo
volatile int reg_pos;     
volatile int reg_desired_pos = 170;  
volatile int reg_current_pos;

Servo rot_servo; // create servo object to control other servo
volatile int rot_pos;
volatile int rot_desired_pos = 40;
volatile int rot_current_pos;

// This file is used for testing purposes
// You manually set the target angles in the setup() instead of reading values from object detection
#define MAX_ENCODER_VAL 16383*6

// step (pulse) pins 
int s0 = 8;

// direction pins
int d0 = 10;

//chip select pins
int c0 = 9;

int i = 0; 
volatile int counter = 0;
volatile int fill_serial_buffer = false;
volatile int servo_wait = 0;

// storing pins and states for each motor
MovingSteppersLib motors[1] {{s0,d0,c0}};  //Instantiate Motors (StepPin, DirectionPin, EncoderChipSelectPin)
int stepPin[1] = {s0}; 
int directionPin[1] = {d0};  
volatile int move [1]; //volatile because changed in ISR
volatile int state [1]; //volatile because changed in ISR

int reversed[1] = {0}; // motors that have encoders facing the wrong way must pick direction changes slightly differently (opposite of normal)

// storing encoder values
volatile float encoderDiff[1];  // units of encoder steps
volatile float encoderTarget[1];  // units of encoder steps
volatile float targetAngle[1];   // units of degrees
float encoderPos[1];   // units of encoder steps

volatile int nottolerant; // motor not within expected position

void reset_input_buffer() {
  while (Serial1.available() > 0) Serial1.read();
  delay(100);
}

void redefine_encoder_zero_position(){
  // call this function to reset the encoder's zero position to be the current position of the motor
  motors[0].encoder.setZeroSPI(c0); 
}

void setup()
{
  Serial.begin(115200); 

  // regular servo setup
  reg_servo.write(75);  // sets initial position
  reg_servo.attach(7);  // attaches the servo on pin 7 to the servo object

  // other regular servo setup
  rot_servo.write(75);
  rot_servo.attach(6);

  // stepper motor setup
  reset_input_buffer();
  //redefine_encoder_zero_position(); // uncomment this whenever you want to set zero position
  targetAngle[0] = -1;
<<<<<<< HEAD
  targetAngle[0] = 0;
=======
  targetAngle[0] = 15;
>>>>>>> 0f27e385841323e1ca5cdb0cd93b44f67e99f0a4
    
  pinMode(directionPin[0], OUTPUT); //set direction and step pins as outputs
  pinMode(stepPin[0], OUTPUT);
  move[0] = 1; //enable j1 // send move to the jetson and recieve the encoder directions from the jetson
<<<<<<< HEAD
  encoderTarget[0] = targetAngle[0] * 45.51111; //map degree to encoder steps
  encoderTarget[0] = encoderTarget[0] * 6; //multiplier for encoder in the wrong place...
=======
  encoderTarget[0] = targetAngle[0] * 45.51111 * 15; //map degree to encoder steps
>>>>>>> 0f27e385841323e1ca5cdb0cd93b44f67e99f0a4
  encoderPos[0] = motors[0].encoder.getPositionSPI(14); //get starting encoder position
  encoderDiff[0] = encoderTarget[0] - encoderPos[0]; //calculate difference between target and current
  
  
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

  if (servo_wait == 200) { // used to slow down servo movement to be more in line with stepper motor
      // regular servo control
      reg_current_pos = reg_servo.read(); //determine the current position of the regular
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
      
      // rotational wrist servo control
      rot_current_pos = rot_servo.read(); //determine the current position of the regular
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
//  Serial.println("Current:");
//  Serial.println(reg_current_pos);  
//  Serial.println("Desired:");
//  Serial.println(reg_desired_pos);
//  Serial.println("Attached:"); 
//  Serial.println(reg_servo.attached());
}


void loop() {
  checkDirLongWay(0);
  // regServoIncrement();
  // Serial.println(motors[0].encoder.getPositionSPI(14));
  // Serial.println(encoderTarget[0]);
  // Serial.println(move[0]);
}

void regServoIncrement() {
  reg_current_pos = reg_servo.read(); //determine the current position of the regular
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
