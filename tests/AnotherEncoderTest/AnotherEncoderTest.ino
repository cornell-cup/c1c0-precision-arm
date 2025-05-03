#include "EncoderLib.h"
#define CS_PIN 38
EncoderLib encoder;
void setup() 
{
  
  //Initialize the UART serial connection for debugging
  Serial.begin(115200);
  Serial.println("Beginning test");

  encoder = EncoderLib();
  encoder.setChipSelect(CS_PIN);
  delay(100);
  encoder.setZeroSPI(CS_PIN);
  Serial.println("Finished setup");
}

void loop() 
{
  Serial.println("Grabbing angle setup");
  int encoderSteps = encoder.getPositionSPI(14);
  float encoderAngle = encoderSteps / 45.51111;         // map degree to encoder steps
  Serial.println(encoderAngle);

  //if you want to set the zero position before beggining uncomment the following function call
  //setZeroSPI(ENC_1);

  //once we enter this loop we will run forever
    delay(500);
}
