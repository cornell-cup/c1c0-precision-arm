#include "R2Protocol.h"
uint8_t encoderAngles[] = {10, 20, 30, 40, 50, 60};
// expects: dinsxy
char mystr[6] = {'A', 'B', 'C', 'D', 'E', 'F'};
char list[] = "parm"; // parm for precise arm

// 2 writes and then send

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial1.begin(9600);

}
char send_buffer[256];
void send(const uint8_t* data, uint32_t data_len) {
  uint32_t written = r2p_encode(list, data, data_len, send_buffer, 256);
  Serial.println(written);
  for(int i=0; i <written; i++){
    Serial1.write(send_buffer[i]);
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  //convertBytetoChar(); 
  //send(mystr, 6);
  send(encoderAngles, 6);
  delay(1000); 
}

void convertBytetoChar(){
  for (int i=0; i<6; i++){
    mystr[i] = encoderAngles[i]; 
  }
}
