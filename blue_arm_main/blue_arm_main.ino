// Interrupt Control for LeArm

// #include <MovingSteppersLib.h>
// #include <MotorEncoderLib.h>
#include "R2Protocol.h"
#include "LobotServoController.h"

typedef enum
{
  INIT,
  DOING_COMMAND
} arm_state;

arm_state state = INIT;

void normalize_angles(uint16_t *angles)
{
  for (int i = 0; i < 6; i++)
  {
    if (angles[i] > 270)
      angles[i] = 90 - (360 - angles[i]);
    else
      angles[i] = angles[i] + 90;
  }
}
// use interrupts file for jetson to arduino communicaiton

#define MAX_ENCODER_VAL 16383

/* PROBLEMS LIST
  1. DO NOT HAVE TWO MOTORS HAVE SAME DIRECTION OR STEP PINS AS ANOTHER MOTOR EVERRRRR IT MESSES UP CODE
*/

// step pins 2
int s0 = 48;
int s1 = 35;
int s2 = 0;
int s3 = 0;
int s4 = 0;
int s5 = 0;
// direction pins
int d0 = 36;
int d1 = 34;
int d2 = 0;
int d3 = 0;
int d4 = 0;
int d5 = 0;
// chip select pins
int c0 = 10;
int c1 = 9;
int c2 = 0;
int c3 = 0;
int c4 = 0;
int c5 = 0;

// Move Individual Motors
int J1_deg_to_pos(float deg)
{
  int lower = 1500;
  int upper = 2500;
  int range = upper - lower;
  int pos = -1;
  if (deg >= 0 && deg <= 180)
  {
    pos = ((deg / 180) * range) + lower;
  }
  return int(pos);
}

int J2_deg_to_pos(float deg)
{
  int lower = 500;
  int upper = 2500;
  int range = upper - lower;
  int pos = -1;
  if (deg >= 0 && deg <= 180)
  {
    pos = ((deg / 180) * range) + lower;
  }
  return pos;
}

int J3_deg_to_pos(float deg)
{
  int lower = 500;
  int upper = 2500;
  int range = upper - lower;
  int pos = -1;
  if (deg >= 0 && deg <= 150)
  {
    pos = ((deg / 180) * range) + lower;
  }
  return pos;
}

int J4_deg_to_pos(float deg)
{
  int lower = 500;
  int upper = 2500;
  int range = upper - lower;
  int pos = -1;
  if (deg >= 0 && deg <= 180)
  {
    pos = ((deg / 180) * range) + lower;
  }
  return pos;
}

int J5_deg_to_pos(float deg)
{
  int lower = 500;
  int upper = 2500;
  int range = upper - lower;
  int pos = -1;
  if (deg >= 0 && deg <= 180)
  {
    pos = ((deg / 180) * range) + lower;
  }
  return pos;
}

int J6_deg_to_pos(float deg)
{
  int lower = 500;
  int upper = 2500;
  int range = upper - lower;
  int pos = -1;
  if (deg >= 0 && deg <= 180)
  {
    pos = ((deg / 180) * range) + lower;
  }
  return pos;
}

// SoftwareSerial mySerial(19,18); // RX, TX

uint8_t send_buf[10];
int i = 0;

// Jetson to Arduino Set up
uint16_t checksum;
char type[5];
uint8_t data[12];
uint32_t data_len = 12;

uint8_t receive_buf[256];
uint8_t buf2[256]; // added, just for test

volatile int counter = 0;
volatile int fill_serial_buffer = false;
void reset_input_buffer()
{
  while (Serial1.available() > 0)
    Serial1.read(); // modified to serial 2
  delay(100);
}

// Arduino to Jetson R2
uint16_t servo_angles[] = {0, 0, 0, 0, 0, 0};
// LobotServo servos[6];   //an array of struct LobotServo
// servos[0].ID = 1;       //No.1 servo
// servos[0].Position = 0;  //1400 position
// servos[1].ID = 4;       //No.4 servo
// servos[1].Position = 0;  //700 position

uint16_t new_servo_angles[] = {0};
uint8_t servo_anglesB8[12];
uint8_t send_buffer[256];

LobotServoController arm(Serial2);

void update_servo_angles(int k)
{
  arm.getPosition(k);
  delay(1000);
  if (Serial2.available() > 0)
  {
    Serial.println("Bytes available: " + String(Serial2.available())); // 2 serials updated to serial 2
    Serial2.readBytes(receive_buf, 8);                                 // serial update to 2
    for (i = 0; i < 8; i++)
    {
      Serial.println(receive_buf[i]);
    }
    //      Serial.println("Data");
    //      for (i=0; i<data_len; i++){
    //        Serial.println(data[i]);
    //      }
    //     //Serial.println(data[1]);
    //     convert_b8_to_b16(data,new_servo_angles);
    //     changeAngles(new_servo_angles);
  }
  // Send read command to LeArm
  // How to read value from the arm???
  // servo_angles[k] = Serial2.read()
}

void changeAngles(uint16_t data[])
{
  // TODO: Call moveServos function for all 6 new joint angles
  // data[] will be 6 elements, call moveservos in each element
  // Position in the array is the joint, value is the angle
  uint16_t normalized_servo_angles[6];

  normalized_servo_angles[0] = data[0];
  normalized_servo_angles[1] = data[5];
  normalized_servo_angles[2] = data[4];
  normalized_servo_angles[3] = data[3];
  normalized_servo_angles[4] = data[2];
  normalized_servo_angles[5] = data[1];

  Serial.println("New Angles:");
  for (i = 0; i < 6; i++)
  {
    Serial.print(String(normalized_servo_angles[i]) + " ");
  }

  normalize_angles(normalized_servo_angles);

  arm.moveServos(6, 1000, 1, J1_deg_to_pos(normalized_servo_angles[0]), 2, J2_deg_to_pos(normalized_servo_angles[5]), 3, J3_deg_to_pos(normalized_servo_angles[4]), 4, J4_deg_to_pos(normalized_servo_angles[3]), 5, J5_deg_to_pos(normalized_servo_angles[2]), 6, J6_deg_to_pos(normalized_servo_angles[1]));
  // delay(1000);
}

void setup()
{
  Serial.begin(115200); // Baud Rate
  Serial1.begin(38400);
  Serial2.begin(9600);

  Serial.println("Hello World");
  //  delay(1000);
  // reset_input_buffer();
}

void send(char type[5], const uint8_t *data, uint32_t data_len, uint8_t *send_buffer)
{
  uint32_t written = r2p_encode(type, data, data_len, send_buffer, 256);
  Serial1.write(send_buffer, written); // modified to serial 2
  Serial1.flush();
  // Serial.println("Bytes written: " + String(written));
  //   for(int i=0; i <written; i++){
  //     Serial.print(send_buffer[i],HEX);
  //   }
  //   Serial.println();
}

void loop()
{
  delay(100);
  changeAngles(servo_angles);

  // Jetson to Arduino
  if (Serial1.available())
  {
    // Serial.println("Bytes available: " + String(Serial1.available())); //2 serials updated to serial 2
    Serial1.readBytes(receive_buf, 28); // serial update to 2
    // Serial.println(r2p_decode(receive_buf, 256, &checksum, type, data, &data_len));
    if (r2p_decode(receive_buf, 256, &checksum, type, data, &data_len))
    {
      Serial.println("message received");
      if (!strcmp(type, "PRMR"))
      {
        Serial.println("current angles requested");
        uint16_t new_data[6] = {};

        convert_b16_to_b8(servo_angles, servo_anglesB8, 6);
        send("prm", servo_anglesB8, 12, send_buffer);
        delay(100);
      }
      else if (!strcmp(type, "PRM"))
      {
        // Receives angles 0-360 degrees
        Serial.println("angles commanded");

        convert_b8_to_b16(data, servo_angles);
        Serial.println("Changed angles");
      }
    }
  }
  // Arduino to Jetson
  else
  {
    // update_servo_angles();
  }
}

void convert_b8_to_b16(uint8_t *databuffer, uint16_t *data)
{
  int data_idx;
  for (int i = 0; i < 12; i++)
  {
    data_idx = i / 2;
    if ((i & 1) == 0)
    {
      // even
      data[data_idx] = databuffer[i] << 8;
    }
    else
    {
      // odd
      data[data_idx] |= databuffer[i];
    }
  }
}
void convert_b16_to_b8(int *databuffer, uint8_t *data, int len)
{
  int data_idx1;
  int data_idx2;
  for (int i = 0; i < 2 * len; i += 2)
  {
    data[i] = (databuffer[i / 2] >> 8) & 255;
    data[i + 1] = (databuffer[i / 2]) & 255;
  }
}
