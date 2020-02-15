#include <MotorEncoderLib.h>
#include <SPI.h>

MotorEncoderLib encoder; 

/* Serial rates for UART */
#define BAUDRATE        115200

/* SPI pins */
#define ENC_0            2
#define ENC_1            3

/* Define special ascii characters */
#define NEWLINE         0x0A
#define TAB             0x09

void setup(){
  pinMode(ENC_0, OUTPUT);
  pinMode(ENC_1, OUTPUT);

  //Initialize the UART serial connection for debugging
  Serial.begin(BAUDRATE);

  //Get the CS line high which is the default inactive state
  digitalWrite(ENC_0, HIGH);
  digitalWrite(ENC_1, HIGH);
  
  SPI.setClockDivider(SPI_CLOCK_DIV32);    // 500 kHz
  SPI.begin(); 
}

void loop(){

  //create a 16 bit variable to hold the encoders position
  uint16_t encoderPosition;
  //let's also create a variable where we can count how many times we've tried to obtain the position in case there are errors
  uint8_t attempts;


  //if you want to set the zero position before beggining uncomment the following function call
  //setZeroSPI(ENC_0);
  //setZeroSPI(ENC_1);

  //once we enter this loop we will run forever
  while(1)
  {
    //set attemps counter at 0 so we can try again if we get bad position    
    attempts = 0;

    //this function gets the encoder position and returns it as a uint16_t
    //send the function either res12 or res14 for your encoders resolution
    encoderPosition = encoder.getPositionSPI(ENC_0, encoder.RES14); 

    //if the position returned was 0xFFFF we know that there was an error calculating the checksum
    //make 3 attempts for position. we will pre-increment attempts because we'll use the number later and want an accurate count
    while (encoderPosition == 0xFFFF && ++attempts < 3)
    {
      encoderPosition = encoder.getPositionSPI(ENC_0, encoder.RES14); //try again
    }

    if (encoderPosition == 0xFFFF) //position is bad, let the user know how many times we tried
    {
      Serial.print("Encoder 0 error. Attempts: ");
      Serial.print(attempts, DEC); //print out the number in decimal format. attempts - 1 is used since we post incremented the loop
      Serial.write(NEWLINE);
    }
    else //position was good, print to serial stream
    {
      
      Serial.print("Encoder 0: ");
      Serial.print(encoderPosition, DEC); //print the position in decimal format
      Serial.write(NEWLINE);
    }

    //For the purpose of this demo we don't need the position returned that quickly so let's wait a half second between reads
    //delay() is in milliseconds
    delay(500);
  }
  
}
