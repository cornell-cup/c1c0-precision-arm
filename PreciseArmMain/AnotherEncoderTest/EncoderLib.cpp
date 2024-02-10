
/*
 * motor_encoder_lib.cpp
 * Date: Feb 15, 2020
 * Converted from the sample code for the CUI amt22 encoder
 *
 */
#include "Arduino.h"
#include "EncoderLib.h"
#include <SPI.h>
/*
 *  Constructor. Defines pins for MISO, MOSI, SCLK
 */
EncoderLib::EncoderLib()
{

  //later j pass in
  /* SPI pins */
  _MISO = 50;
  _MOSI = 51;
  _SCLK = 52;
  CS = 44;
  /* SPI commands */
  AMT22_NOP = 0x00;
  AMT22_RESET = 0x60;
  AMT22_ZERO = 0x70;
  /* We will use these define macros so we can write code once compatible with 12 or 14 bit encoders */
  RES12 = 12;
  RES14 = 14;
  pinMode(CS, OUTPUT);
  pinMode(_MISO, INPUT);
  pinMode(_MOSI, OUTPUT);
  pinMode(_SCLK, OUTPUT);
   //Get the CS line high which is the default inactive state
  digitalWrite(CS, HIGH);
}
/*
void EncoderLib::setChipSelect(int encoderPinIn)
{
  CS = encoderPinIn;
  pinMode(CS, OUTPUT);
 
  
}*/
/*
 * This function gets the absolute position from the AMT22 encoder using the SPI bus. The AMT22 position includes 2 checkbits to use
 * for position verification. Both 12-bit and 14-bit encoders transfer position via two bytes, giving 16-bits regardless of resolution.
 * For 12-bit encoders the position is left-shifted two bits, leaving the right two bits as zeros. This gives the impression that the encoder
 * is actually sending 14-bits, when it is actually sending 12-bit values, where every number is multiplied by 4.
 * This function takes the pin number of the desired device as an input
 * This funciton expects res12 or res14 to properly format position responses.
 * Error values are returned as 0xFFFF
 * Encoder reads from 0 to 16384.
 * 16384/360.0 = 45.5111 per degree

 */
uint16_t EncoderLib::getPositionSPI(uint8_t resolution)
{
  Serial.println(CS);
  uint16_t currentPosition; // 16-bit response from encoder
  bool binaryArray[16];     // after receiving the position we will populate this array and use it for calculating the checksum
  // get first byte which is the high byte, shift it 8 bits. don't release line for the first byte
  currentPosition = spiWriteRead(AMT22_NOP, CS, false) << 8;
  // this is the time required between bytes as specified in the datasheet.
  // We will implement that time delay here, however the arduino is not the fastest device so the delay
  // is likely inherantly there already
  delayMicroseconds(3);
  // OR the low byte with the currentPosition variable. release line after second byte
  currentPosition |= spiWriteRead(AMT22_NOP, CS, true);
  // run through the 16 bits of position and put each bit into a slot in the array so we can do the checksum calculation
  for (int i = 0; i < 16; i++)
    binaryArray[i] = (0x01) & (currentPosition >> (i));
  // using the equation on the datasheet we can calculate the checksums and then make sure they match what the encoder sent
  if ((binaryArray[15] == !(binaryArray[13] ^ binaryArray[11] ^ binaryArray[9] ^ binaryArray[7] ^ binaryArray[5] ^ binaryArray[3] ^ binaryArray[1])) && (binaryArray[14] == !(binaryArray[12] ^ binaryArray[10] ^ binaryArray[8] ^ binaryArray[6] ^ binaryArray[4] ^ binaryArray[2] ^ binaryArray[0])))
  {
    // we got back a good position, so just mask away the checkbits
    currentPosition &= 0x3FFF;
  }
  else
  {
    currentPosition = 0xFFFF; // bad position
  }
  // If the resolution is 12-bits, and wasn't 0xFFFF, then shift position, otherwise do nothing
  if ((resolution == RES12) && (currentPosition != 0xFFFF))
    currentPosition = currentPosition >> 2;
  return currentPosition;
}
/*
 * This function does the SPI transfer. sendByte is the byte to transmit.
 * Use releaseLine to let the spiWriteRead function know if it should release
 * the chip select line after transfer.
 * This function takes the pin number of the desired device as an input
 * The received data is returned.
 */
uint8_t EncoderLib::spiWriteRead(uint8_t sendByte, uint8_t encoder, uint8_t releaseLine)
{
  // holder for the received over SPI
  uint8_t data;
  
  // set cs low, cs may already be low but there's no issue calling it again except for extra time
  setCSLine(encoder, LOW);
  
  // There is a minimum time requirement after CS goes low before data can be clocked out of the encoder.
  // We will implement that time delay here, however the arduino is not the fastest device so the delay
  // is likely inherently there already
  delayMicroseconds(3);
  // send the command
  Serial.println("!p");
  Serial.println(sendByte);
  data = SPI.transfer(sendByte);
  Serial.println("!Q");
  delayMicroseconds(3);            // There is also a minimum time after clocking that CS should remain asserted before we release it
  setCSLine(encoder, releaseLine); // if releaseLine is high set it high else it stays low
  return data;
}
/*
 * This function sets the state of the SPI line. It isn't necessary but makes the code more readable than having digitalWrite everywhere
 * This function takes the pin number of the desired device as an input
 */
void EncoderLib::setCSLine(uint8_t encoder, uint8_t csLine)
{
  digitalWrite(encoder, csLine);
}
/*
 * The AMT22 bus allows for extended commands. The first byte is 0x00 like a normal position transfer, but the
 * second byte is the command.
 * This function takes the pin number of the desired device as an input
 */
void EncoderLib::setZeroSPI(uint8_t encoder)
{
 
  spiWriteRead(AMT22_NOP, encoder, false);
  Serial.print("enter");
  // this is the time required between bytes as specified in the datasheet.
  // We will implement that time delay here, however the arduino is not the fastest device so the delay
  // is likely inherently there already
  delayMicroseconds(3);
  spiWriteRead(AMT22_ZERO, encoder, true);
   
  delay(250); // 250 millisecond delay to allow the encoder to reset
}
/*
 * The AMT22 bus allows for extended commands. The first byte is 0x00 like a normal position transfer, but the
 * second byte is the command.
 * This function takes the pin number of the desired device as an input
 */
void EncoderLib::resetAMT22(uint8_t  encoder)
{
  spiWriteRead(AMT22_NOP, encoder, false);
  // this is the time required between bytes as specified in the datasheet.
  // We will implement that time delay here, however the arduino is not the fastest device so the delay
  // is likely inherently there already
  delayMicroseconds(3);
  spiWriteRead(AMT22_RESET, encoder, true);
  delay(250); // 250 second delay to allow the encoder to start back up
}
