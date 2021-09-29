// Jetson to Arduino Communication 
#include <MovingSteppersLib.h>
#include <MotorEncoderLib.h>
#include <SoftwareSerial.h>

SoftwareSerial mySerial(19,18); // RX, TX

#define MAX_ENCODER_VAL 16383

int i = 0;

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

// uint8_t receive_buf[10];

// read port 8 or 10 using serial 

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600); // set Baud Rate
  Serial1.begin(9600); 
  Serial1.flush();
  Serial.println("Hello World"); // Serial is what is print
  //mySerial.begin(9600);
  //mySerial.println("Bonjour"); 
  Serial1.println("Bonjour"); 
 

}

void loop() {
  // put your main code here, to run repeatedly:
  if (Serial1.available())
    //Serial.println('\n');
    Serial.write(Serial1.read());
  // flush the serial every 100 iterations 
  if ((i % 10) == 0) 
    Serial1.flush();
  
}
