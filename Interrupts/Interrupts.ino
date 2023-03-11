#include <MovingSteppersLib.h>
#include <MotorEncoderLib.h>
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
//chip select pins
int c0 = 47;
int c1 = 44;
int c2 = 41;
int c3 = 38;
int c4 = 35;
int c5 = 32;

//SoftwareSerial mySerial(19,18); // RX, TX

uint8_t send_buf[10];
int i = 0; 

// Jetson to Arduino Set up
uint16_t checksum;
char type[5];
uint8_t data[6]; 
uint32_t data_len = 6; 

uint8_t receive_buf[256];

volatile int counter = 0;
volatile int fill_serial_buffer = false;

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

void reset_input_buffer() {
  while (Serial1.available() > 0 ) Serial1.read();
  delay(100);
}

void setup()
{
  Serial.begin(9600); //Baud Rate
  Serial1.begin(9600); 

  Serial.println("Hello World");
  delay(1000);
  reset_input_buffer();
   for (i=0; i <256; i++){
    //Serial.println(receive_buf[i]);
   } 
 
  //}

  //send_buf[0] = 255;
  //send_buf[1] = 254;
  //send_buf[8] = 255;
  //send_buf[9] = 253;

  
// Only uncomment when you want to zero the encoders
//  motors[0].encoder.setZeroSPI(c0); // zero the encoder at desired position
//  motors[1].encoder.setZeroSPI(c1);     // when J2 motor juts towards me
//  motors[2].encoder.setZeroSPI(c2);     // zero is at the left
//  motors[3].encoder.setZeroSPI(c3);
//  motors[4].encoder.setZeroSPI(c4);
//  motors[5].encoder.setZeroSPI(c5);
  for (int i=0; i<6; i++){ //for each motor
  // initialized to something that isn't valid
   targetAngle[i] = -1;   // used for testing, this will be an input from object detection
 //  targetAngle[0] = 90; // read serial input

//    targetAngle[1] = 80;
   //targetAngle[2] = 200;
   
//   targetAngle[1] = 100;
//   targetAngle[2] = 90; 
////   
//   targetAngle[1] = 130;
//   targetAngle[2] = 40; 
////   
  // targetAngle[1] = 80;
//   targetAngle[2] = 170;  
   
  //  targetAngle[3] = 300;
   //targetAngle[4] = 100; 
    
    pinMode(directionPin[i], OUTPUT); //set direction and step pins as outputs
    pinMode(stepPin[i], OUTPUT);

   
    move[i] = 0; //default is to move none
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

  Serial.println("Exited setup");
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
//        Serial.println("turn off");
        move[i] = 0; //stop moving motor if location reached
      }
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
uint16_t encoder_angles[] = {10, 20, 30, 40, 50, 60};
uint8_t encoder_anglesB8[12]; 
uint8_t send_buffer[256];
int k;

void update_encoder_angles(){
  for (k =0; k <6; k++){
    encoder_angles[k] = (uint16_t) motors[k].encoder.getPositionSPI(14)/ 45.1111;  // how to convert to char and how many digits to round to
    // if you get the error message (aka encoder isn't connected), set to zero
    if(encoder_angles[k] == 1452){
      encoder_angles[k] = 0;
    }
  }
}


void send(char type[5], const uint8_t* data, uint32_t data_len, uint8_t* send_buffer) {
  uint32_t written = r2p_encode(type, data, data_len, send_buffer, 256);
  Serial1.write(send_buffer, written);
  //Serial.println("Bytes written: " + String(written));
  for(int i=0; i < data_len; i++){
    //Serial.println(data[i]);
 }
}

void loop()
{
   for (i=0; i <256; i++){
    //Serial.println(receive_buf[i]);
   } 
  for (int i=0; i<6; i++){
     checkDirLongWay(i); 
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
  
  if (Serial1.available() > 22) {
      //Serial.println("Bytes available: " + String(Serial1.available()));
      //Serial1.readBytes(receive_buf, 256);
      for (i=0; i<22; i++) {
       // Serial.println(receive_buf[i]);
      }
      //Serial.println(r2p_decode(receive_buf, 256, &checksum, type, data, &data_len));
      r2p_decode(receive_buf, 256, &checksum, type, data, &data_len);
     // Serial.println(String(type));
     // Serial.println("Checksum: " + String(checksum));
     // Serial.println(data_len);
     // Serial.println("done decode"); 

      Serial.println("Data");
      for (i=0; i<data_len; i++){
        Serial.println(data[i]); 
      }
     //Serial.println(data[1]);
     changeAngles(data);
    } 
 
  // Arduino to Jetson   
// else{
    update_encoder_angles();
    convert_b16_to_b8(encoder_angles, encoder_anglesB8, 6);
    send("prm", encoder_anglesB8, 12, send_buffer);
    delay(100);
  //}
                             
  
}

void changeAngles(uint8_t data[]){
  for (i=0; i<6; i++){
    if (targetAngle[i] != data[i]){
      targetAngle[i] = data[i];
      encoderTarget[i] = targetAngle[i] * 45.51111 * 360/255;
      encoderPos[i] = motors[i].encoder.getPositionSPI(14);
      encoderDiff[i] = encoderTarget[i] - encoderPos[i];
      move[i] = 1;
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
