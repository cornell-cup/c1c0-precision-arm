#include <LobotServoController.h>

// J1: 1500 - 2500
// J2: 500 - 2500
// J3: 500 - 2000 (2500 on App)
// J4: (max range depends on J5) 500 -2500
// J5: 500 - 2500
// J6: 500 - 2500

/*************** Move Individual Motors *****************/
int J1_deg_to_pos (float deg){
  int lower = 1500;
  int upper = 2500; 
  int range = upper - lower; 
  int pos = -1;
  if (deg >= 0 && deg <= 180) {
    pos = ((deg/180) * range) + lower;
  }
  return int(pos);
}

int J2_deg_to_pos (float deg){
  int lower = 500;
  int upper = 2500; 
  int range = upper - lower; 
  int pos = -1;
  if (deg >= 0 && deg <= 180) {
    pos = ((deg/180) * range) + lower;
  }
  return pos;
}

int J3_deg_to_pos (float deg){
  int lower = 500;
  int upper = 2000; 
  int range = upper - lower; 
  int pos = -1;
  if (deg >= 0 && deg <= 180) {
    pos = ((deg/180) * range) + lower;
  }
  return pos;
}

int J4_deg_to_pos (float deg){
  int lower = 500;
  int upper = 2500; 
  int range = upper - lower; 
  int pos = -1;
  if (deg >= 0 && deg <= 180) {
    pos = ((deg/180) * range) + lower;
  }
  return pos;
}

int J5_deg_to_pos (float deg){
  int lower = 500;
  int upper = 2500; 
  int range = upper - lower; 
  int pos = -1;
  if (deg >= 0 && deg <= 180) {
    pos = ((deg/180) * range) + lower;
  }
  return pos;
}

int J6_deg_to_pos (float deg){
  int lower = 500;
  int upper = 2500; 
  int range = upper - lower; 
  int pos = -1;
  if (deg >= 0 && deg <= 180) {
    pos = ((deg/180) * range) + lower;
  }
  return pos;
}
/*************** End Move Individual Motors *****************/

int convert_deg_to_pos (int motor, float deg) {
  switch (motor){
    case 1: J1_deg_to_pos(deg);
    case 2: J2_deg_to_pos(deg);
    case 3: J3_deg_to_pos(deg);
    case 4: J4_deg_to_pos(deg);
    case 5: J5_deg_to_pos(deg);
    case 6: J6_deg_to_pos(deg);
    default: -1; 
  }
}

LobotServoController arm(Serial1);

void setup() {
  Serial1.begin(9600);
  Serial.begin(9600);
  while(!Serial1);
}

void loop() {    
  String pos_str = Serial1.read();
  serial.println(pos_str);
  // arm.moveServos(6, 1000, 1, J1_deg_to_pos(), 2, J2_deg_to_pos(), 3, J3_deg_to_pos(), 4, J4_deg_to_pos(), 5, J5_deg_to_pos(), 6, J6_deg_to_pos());
  delay(1000);
  // arm.moveServos(4, 1000, 1, J1_deg_to_pos(90), 2, 1000, 3, 1000, 4, 1000);
  // delay(1000); 
  // arm.moveServos(4, 1000, 1, J1_deg_to_pos(0), 2, 1000, 3, 1000, 4, 1000);
  // delay(1000);
  //Serial.println(J1_deg_to_pos (90));
  //arm.moveServos(1,2500,2,1000);
  //delay(1000);
  //Serial.println(J1_deg_to_pos (10));
  //arm.moveServos(1,1500,2,1000);
  //delay(1000);

  
}
