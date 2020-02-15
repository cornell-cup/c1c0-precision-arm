/*
 * motor_encoder_lib.h
 * Date: Feb 15, 2020
 * Converted from the sample code for the CUI amt22 encoder
 * 
*/

#ifndef MotorEncoderLib_h
#define MotorEncoderLib_h

#include "Arduino.h"

class MotorEncoderLib 
{
	public: 
		MotorEncoder();
		uint16_t getPositionSPI(uint8_t encoder, uint8_t resolution);
		uint8_t spiWriteRead(uint8_t sendByte, uint8_t encoder, uint8_t releaseLine);
		void setCSLine (uint8_t encoder, uint8_t csLine)
		void setZeroSPI(uint8_t encoder)
		void resetAMT22(uint8_t encoder)
		
	private: 
		int _MISO;
		int _MOSI;
		int _SCLK;
		uint8_t _AMT22_NOP;
		uint8_t _AMT22_RESET; 
		uint8_t _AMT22_ZERO;
		uint8_t _RES12;
		uint8_t _RES14;
}

#endif




