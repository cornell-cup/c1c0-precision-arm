#include "R2Protocol.h"
// This file was used for testing Arduino to Jetson communication
// Currently, all of this code is added to the Interrupts file
// Upload the Interrupts file to the board and run from there
// This file is purely used for debugging with Jetson terminal
uint8_t encoder_angles[] = {10, 20, 30, 40, 50, 60};
char list[] = "parm"; // parm for precise arm
char send_buffer[256];

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial1.begin(9600);
}

void send(const uint8_t* data, uint32_t data_len) {
  uint32_t written = r2p_encode(list, data, data_len, send_buffer, 256);
  Serial.println(written);
  for(int i=0; i <written; i++){
    Serial1.write(send_buffer[i]);
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  send(encoder_angles, 6);
  delay(1000); 
}
