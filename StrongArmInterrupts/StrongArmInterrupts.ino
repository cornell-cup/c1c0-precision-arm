// Main file for Strong Arm, to run on Arduino

#include <MovingSteppersLib.h>
#include <MotorEncoderLib.h>
#include <R2Protocol.h>

// use interrupts file for jetson to arduino communicaiton
// ISR: Interrupt Service Routine
// volatile: a directive to the compiler, it signals that the value may change by something beyond the control of the code
// todo: maybe create 2 files, one for communciation and one for just testing Arduino to Strong Arm 

#define MAX_ENCODER_VAL 16383

// step pin
int s0 = 26;

// direction pin
int d0 = 27;

//chip select pin
int c0 = 10;


//SoftwareSerial mySerial(19,18); // RX, TX

uint8_t send_buf[10];
int i = 0; 

// Jetson to Arduino Set up
uint16_t checksum;
char type[5];
uint8_t data[6]; 
uint32_t data_len = 13; // should be 6

uint8_t receive_buf[256];

volatile int counter = 0;
volatile int fill_serial_buffer = false;

// Storing pins and states for each motor
// Instantiate Motors (StepPin, DirectionPin, EncoderChipSelectPin)
MovingSteppersLib motors[1] {{s0,d0,c0}}; 
int stepPin[1] = {s0}; 
int directionPin[1] = {d0};  
volatile int move [1]; //volatile because changed in ISR
volatile int state [1]; //volatile because changed in ISR


// todo: is the encoder considered reversed?
int reversed[1] = {0}; // motors that have encoders facing the wrong way must pick direction changes slightly differently (opposite of normal)

// Storing encoder values
volatile float encoderDiff[1];  // units of encoder steps
volatile float encoderTarget[1];  // units of encoder steps
volatile float targetAngle[1];   // units of degrees
float encoderPos[1];   // units of encoder steps

volatile int nottolerant; // set when motor not within expected position

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

  // todo: figure out what this does
  //send_buf[0] = 255;
  //send_buf[1] = 254;
  //send_buf[8] = 255;
  //send_buf[9] = 253;

  // Only uncomment when you want to zero the encoders
  // motors[0].encoder.setZeroSPI(c0); // zero the encoder at desired position

  // initialize the Target angle to something that isn't valid 
  // this value is used for testing, this will be an input from object detection
  targetAngle[i] = -1;
  
  pinMode(directionPin[1], OUTPUT); //set direction and step pins as outputs
  pinMode(stepPin[1], OUTPUT);
  move[1] = 0; //default is to move none
  //move[1] = 1; //enable joint
   
  encoderTarget[1] = targetAngle[1] * 45.51111; //map degree to encoder steps
  encoderPos[1] = motors[1].encoder.getPositionSPI(14); //get starting encoder position
  encoderDiff[1] = encoderTarget[1] - encoderPos[1]; //calculate difference between target and current
  
  // initialize interrupt timer1 
  noInterrupts();           // disable all interrupts
  TCCR1A = 0;
  TCCR1B = 0;

  TCNT1 = 65518;            // preload timer 65536-(16MHz/256/4Hz)
  TCCR1B |= (1 << CS12);    // 256 prescaler 
  TIMSK1 |= (1 << TOIE1);   // enable timer overflow interrupt
  interrupts();             // enable all interrupts
} // end of setup

ISR(TIMER1_OVF_vect) //ISR to pulse pins of moving motors
{
  TCNT1 = 65518;            // preload timer to 300 us
  fill_serial_buffer = true; //check
  nottolerant = abs(encoderDiff[1]) > 10 && ((abs(encoderDiff[1]) + 10) < (MAX_ENCODER_VAL + encoderTarget[1])); // 2nd condition to check if 359degrees is close enough to 0
  //nottolerant = abs(encoderDiff[i]) > 10; // we dont need the extra condition above bc we never pass through zero
    
  if (move[1]) { //if motor should move
    if (nottolerant){ //if not within tolerance
      state[1] = !state[1]; //toggle state
      digitalWrite(stepPin[1], state[1]); //write to step pin
    }
    else {
//        Serial.println("turn off");
      move[1] = 0; //stop moving motor if location reached
    }
  }
  
    // This is for moving motor to two places
//  if ( !move[0] && !move[1] && !move[2] && !move[3] && !move[4] && !move[5] && (counter==0)) {
//    targetAngle[2] = 100;
//    encoderTarget[2] = targetAngle[2] * 45.51111; //map degree to encoder steps
//    move[2] = 1;
//    encoderDiff[2] = encoderTarget[2] - encoderPos[2];
//    counter++;
//  }
//  if ( !move[0] && !move[1] && !move[2] && !move[3] && !move[4] && !move[5] && (counter==1)) {
//    targetAngle[2] = 80;
//    encoderTarget[2] = targetAngle[2] * 45.51111; //map degree to encoder steps
//    move[2] = 1;
//    encoderDiff[2] = encoderTarget[2] - encoderPos[2];
//    counter++;
//  }
//  if ( !move[0] && !move[1] && !move[2] && !move[3] && !move[4] && !move[5] && (counter==2)) {
//    targetAngle[2] = 100;
//    encoderTarget[2] = targetAngle[2] * 45.51111; //map degree to encoder steps
//    move[2] = 1;
//    encoderDiff[2] = encoderTarget[2] - encoderPos[2];
//    counter++;
//  }
}

// Arduino to Jetson R2
uint16_t encoder_angle[] = {10};
uint8_t encoder_angleB8[12]; 
uint8_t send_buffer[256];
int k;

void update_encoder_angle(){
    encoder_angle[1] = (uint16_t) motors[1].encoder.getPositionSPI(14)/ 45.1111;
}

void send(char type[5], const uint8_t* data, uint32_t data_len, uint8_t* send_buffer) {
  uint32_t written = r2p_encode(type, data, data_len, send_buffer, 256);
  Serial1.write(send_buffer, written);
 // Serial.println("Bytes written: " + String(written));
//  for(int i=0; i <written; i++){
//    Serial.write(send_buffer[i]);
//  }
}

void loop(){
  checkDirLongWay(1); 
  }
  
//  Serial.println(move[1]);
//  Serial.println(move[1]);

  //if(fill_serial_buffer){
    // makeSerBuffers();
  //}
  
   //if (Serial1.available() > 1) {
     // Serial.println(Serial1.read());
    //}
//  Serial.println(motors[0].encoder.getPositionSPI(14)); 
//  Serial.println(motors[1].encoder.getPositionSPI(14));
//  Serial.println(motors[2].encoder.getPositionSPI(14));
//  Serial.println(motors[3].encoder.getPositionSPI(14));
//  Serial.println(motors[4].encoder.getPositionSPI(14));
//  Serial.println(motors[5].encoder.getPositionSPI(14));


  
  // Jetson to Arduino
  
   if (Serial1.available() > 0) {
      Serial.println("Bytes available: " + String(Serial1.available()));
      Serial1.readBytes(receive_buf, 256);
      // todo: check rp2 communciation with Jetson for strong arm
      for (i=0; i<22; i++) {
        Serial.println(receive_buf[i]);
      }
      //Serial.println(r2p_decode(receive_buf, 256, &checksum, type, data, &data_len));
      r2p_decode(receive_buf, 256, &checksum, type, data, &data_len);
      Serial.println(String(type));
      Serial.println("Checksum: " + String(checksum));
      Serial.println(data_len);
      Serial.println("done decode"); 

      Serial.println("Data");
      for (i=0; i<data_len; i++){
        Serial.println(data[i]); 
      }
      //Serial.println(data[1]);
      changeAngle(data);
    } 
 
  // Arduino to Jetson   
  else{
    update_encoder_angle();
    convert_b16_to_b8(encoder_angle, encoder_angleB8, 12);
    send("prm", encoder_angleB8, 12, send_buffer);
    }  
}

// todo: don't round with 360/255, instead just send a longer array and recombine
void changeAngle(uint8_t data[]){
  if (targetAngle[1] != data[1]){
    targetAngle[1] = data[1];
    encoderTarget[1] = targetAngle[1] * 45.51111 * 360/255;
    encoderPos[1] = motors[1].encoder.getPositionSPI(14);
    encoderDiff[1] = encoderTarget[1] - encoderPos[1];
    move[1] = 1;
  }
}

// checks that motor is moving in right direction and switches if not
void checkDirLongWay(int motorNum){ 
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

// todo: update size for strong arm 
void convert_b8_to_b16(uint8_t *databuffer, uint16_t *data) {
  int data_idx;
  for (int i=0; i < 16; i++) {
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

// todo: update size for strong arm 
void convert_b16_to_b8(int *databuffer, uint8_t *data, int len) {
  int data_idx1;
  int data_idx2;
  for (int i = 0; i < 2*len; i+=2) {
    data[i] = (databuffer[i/2] >> 8) & 255;
    data[i+1] = (databuffer[i/2]) & 255;
  }
}

//void makeSerBuffers(){
  
  //fill_serial_buffer = false;
  //for (int i=0; i<6; i++) {
//    send_buf[2*i2+] = (motors[i].encoder.getPositionSPI(14) >> 8) & 255;  
//    send_buf[2*i+3] =  motors[i].encoder.getPositionSPI(14) & 255; 
     // send_buf[2+i] = move[i];
//  }
  //delay(500);
  //Serial.println(send_buf[5]);
  //Serial1.flush();
  //Serial1.write(send_buf, sizeof(send_buf));
//}




//void checkDirThroughZero(int motorNum){ //checks that motor is moving in right direction and switches if not
//
//  encoderPos = motors[motorNum].encoder.getPositionSPI(14);
//  encoderDiff[motorNum] = encoderTarget[motorNum] - encoderPos;
//
//  if (abs(encoderDiff[motorNum]) >= 8192) { //angle > 180 (encoder units)
//        if (encoderDiff[motorNum] > 0) {
//         digitalWrite(directionPin[motorNum], reversed[motorNum]); //J3 Clockwise is LOW
//        }
//        else {
//        digitalWrite(directionPin[motorNum], !reversed[motorNum]);
//        }
//   }
//   else {
//        if(encoderDiff[motorNum] > 0){
//            digitalWrite(directionPin[motorNum], !reversed[motorNum]); 
//        }
//        else {
//            digitalWrite(directionPin[motorNum], reversed[motorNum]);
//        }
//    }
//}
