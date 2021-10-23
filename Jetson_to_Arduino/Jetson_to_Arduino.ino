// Jetson to Arduino Communication 
#include <MovingSteppersLib.h>
#include <MotorEncoderLib.h>
#include <R2Protocol.h>

//SoftwareSerial mySerial(19,18); // RX, TX

#define MAX_ENCODER_VAL 16383

int i = 0;
// 16 bit checksum, result of decoding 
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

uint8_t receive_buf[256];

// read port 8 or 10 using serial 

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600); // set Baud Rate
  Serial1.begin(9600); 
  Serial1.flush();
  Serial.println("Hello World");

}
uint16_t checksum;
char type[4];
uint8_t data[1]; // change if you want to send a char or an int, declare globally!
uint32_t data_len = 1; 
void loop() {
    // put your main code here, to run repeatedly:
    // Serial1.flush();
    //receive_buf = Serial1.read();
    //Serial.println('\n');
    if (Serial1.available() > 0) {
      Serial1.readBytes(receive_buf, 256);
      for (i=0;i<17; i++){
        Serial.println(receive_buf[i]); // correctly prints the buffer
      }
      Serial.println(r2p_decode(receive_buf, 256, &checksum, type, data, &data_len));
      Serial.println(data_len);
      Serial.println("done decode"); // prints this line
      for (i=0; i<1; i++){
        Serial.println(data[i]); // doesn't print this line
      }
    }
    // Serial.write("\n");
    //Serial.write(r2p_decode(receive_buf, 1, checksum, type, data, 1));
    //Serial.write("Hello again");
    //Serial.write(data); 
    
    // flush the serial every 10 iterations 
    //if ((i % 10) == 0) 
    //Serial1.flush();

    // decode the message from the Jetson using the R2 Protocol 
    // buffer is what is received including checksum, start, and end message, and encoded target angles
    // data is the output: it is extracted from the buffer 
    // function header: inline int32_t r2p_decode(const uint8_t* buffer, uint32_t buffer_len, uint16_t* checksum, char type[5], uint8_t* data, uint32_t* data_len) {
    
    // data is 4 char, convert to target angles cast using toInt
 
}

// expected output
//255
//254 end of start sequence
//80
//81
//82
//83
//84
//85
//255
//253 end of stop sequence 
