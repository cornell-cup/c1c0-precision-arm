
#include <SPI.h>
//typedef struct
//{
//    uint16_t max_angle;
//    uint16_t min_angle;
//    uint16_t current_angle;
//    motor_t* motor;
//    encoder_t* encoder;
//} joint_t;
//
//inline void init_motor_encoder_pair(joint_t* joint)
//{
//    init_motor(joint->motor);
//    init_encoder(joint->encoder);
//    joint->current_angle = getPositionSPI(joint->encoder);
//}
//
//motor_t J1_motor = {.pulse_pin = 1, .dir_pin = 2, .positive_dir, .pulse_state = 0};
//encoder_t J1_encoder = {.cs = 1, .resolution = 2};
//joint_t J1 = {.encoder = &J1_encoder, .motor = &J1_motor, .max_angle = 90, .min_angle = 0};
//
typedef struct
{
    uint16_t pulse_pin;
    uint16_t dir_pin;
    bool pulse_state;
} motor_t;

inline void init_motor(motor_t* motor){
    pinMode(motor->pulse_pin, OUTPUT);
    pinMode(motor->dir_pin, OUTPUT);
    motor->pulse_state = 0;
}

inline void step_motor(motor_t* motor, bool dir){
    digitalWrite(motor->dir_pin, dir);
    motor->pulse_state = !motor->pulse_state;
    digitalWrite(motor->pulse_pin, motor->pulse_state);
}

#define AMT22_NOP (0x00)
#define AMT22_RESET (0x60)
#define AMT22_ZERO (0x70)

#define MISO 50
#define MOSI 51
#define SCLK 52

#define RES12 12
#define RES14 14

#define ANGLE_TOLERANCE 100

typedef struct
{
    uint16_t cs;
    uint8_t resolution;
    float current_angle;
} encoder_t;

inline void init_encoder(encoder_t *encoder)
{
    encoder->current_angle = 0;
    pinMode(encoder->cs, OUTPUT);
    pinMode(MISO, INPUT);
    pinMode(MOSI, OUTPUT);
    pinMode(SCLK, OUTPUT);
    // Get the CS line high which is the default inactive state
    digitalWrite(encoder->cs, HIGH);
}

/*
 * This function does the SPI transfer. sendByte is the byte to transmit.
 * Use releaseLine to let the spiWriteRead function know if it should release
 * the chip select line after transfer.
 * This function takes the pin number of the desired device as an input
 * The received data is returned.
 */
inline uint8_t spiWriteRead(encoder_t *encoder, uint8_t sendByte, uint8_t releaseLine)
{
    // holder for the received over SPI
    uint8_t data;
    // set cs low, cs may already be low but there's no issue calling it again except for extra time
    digitalWrite(encoder->cs, LOW);
    // There is a minimum time requirement after CS goes low before data can be clocked out of the encoder.
    // We will implement that time delay here, however the arduino is not the fastest device so the delay
    // is likely inherently there already
    delayMicroseconds(3);
    // send the command
    data = SPI.transfer(sendByte);
    delayMicroseconds(3);                   // There is also a minimum time after clocking that CS should remain asserted before we release it
    digitalWrite(encoder->cs, releaseLine); // if releaseLine is high set it high else it stays low
    return data;
}

/*
 * The AMT22 bus allows for extended commands. The first byte is 0x00 like a normal position transfer, but the
 * second byte is the command.
 * This function takes the pin number of the desired device as an input
 */
inline void setZeroSPI(encoder_t *encoder)
{
    encoder->current_angle = 0;
    spiWriteRead(encoder, AMT22_NOP, false);
    // this is the time required between bytes as specified in the datasheet.
    // We will implement that time delay here, however the arduino is not the fastest device so the delay
    // is likely inherently there already
    delayMicroseconds(3);
    spiWriteRead(encoder, AMT22_ZERO, true);
    delay(250); // 250 millisecond delay to allow the encoder to reset
    Serial.println("zeroed");
}

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
inline void getPositionSPI(encoder_t *encoder)
{
    uint16_t currentPosition; // 16-bit response from encoder
    bool binaryArray[16];     // after receiving the position we will populate this array and use it for calculating the checksum
    // get first byte which is the high byte, shift it 8 bits. don't release line for the first byte
    currentPosition = spiWriteRead(encoder, AMT22_NOP, false) << 8;
    // this is the time required between bytes as specified in the datasheet.
    // We will implement that time delay here, however the arduino is not the fastest device so the delay
    // is likely inherantly there already
    delayMicroseconds(3);
    // OR the low byte with the currentPosition variable. release line after second byte
    currentPosition |= spiWriteRead(encoder, AMT22_NOP, true);
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
    if ((encoder->resolution == RES12) && (currentPosition != 0xFFFF))
        currentPosition = currentPosition >> 2;
    float current_angle = currentPosition / (45.51111);
    //sometimes values spike so this is to only take good values, didn't work tho so commented out
    //if(abs(current_angle - encoder->current_angle) < ANGLE_TOLERANCE)
        encoder->current_angle = current_angle;
}


