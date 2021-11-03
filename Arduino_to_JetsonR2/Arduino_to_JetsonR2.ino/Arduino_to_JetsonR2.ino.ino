#include "R2Protocol.h"
char mystr[5] = {'A', 'B', 'C', 'D', 'E'};
char list[] = "parm"; // parm for precise arm


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
  send(mystr, 6);
  delay(1000); 

}
