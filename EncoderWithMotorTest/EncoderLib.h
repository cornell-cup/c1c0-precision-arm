
/*
 * motor_encoder_lib.h
 * Date: Feb 15, 2020
 * Converted from the sample code for the CUI amt22 encoder
 *
 */
//==============================================================
// Wiring Reference:
//  blue: sclock
//  blue white: MOSI
//  green: GND
//  green white: 5V
//  orange: MISO
//  orange white: Chip Select
//==============================================================
#ifndef EncoderLib_h
#define EncoderLib_h
#include "Arduino.h"
#include <SPI.h>
class EncoderLib
{
public:
  EncoderLib(); 
  //void setChipSelect(int encoderPinIn);
  uint16_t getPositionSPI(uint8_t resolution);
  uint8_t spiWriteRead(uint8_t sendByte, uint8_t encoder, uint8_t releaseLine);
  void setChipSelect(int encoderPinIn);
  void setCSLine(uint8_t encoder, uint8_t csLine);
  float flipEncoder();
  void setZeroSPI(uint8_t encoder);
  void resetAMT22(uint8_t encoder);
  void stepUsingEncoder();
  int RES12;
  int RES14;
  int CS;
private:
  int _MISO;
  int _MOSI;
  int _SCLK;
  int AMT22_NOP;
  int AMT22_RESET;
  int AMT22_ZERO;
};
#endif
