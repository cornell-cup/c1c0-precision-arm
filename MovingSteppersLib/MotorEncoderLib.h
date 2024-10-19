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


#ifndef MotorEncoderLib_h
#define MotorEncoderLib_h

#include "Arduino.h"

class MotorEncoderLib
{
	public:
		MotorEncoderLib();
        void setChipSelect(int encoderPinIn);
		uint16_t getPositionSPI(int resolution);
		int spiWriteRead(int sendByte, int encoder, int releaseLine);
		void setCSLine(int encoder, int csLine);
		void setZeroSPI(int encoder);
		void resetAMT22(int encoder);
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
