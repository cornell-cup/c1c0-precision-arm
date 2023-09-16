#include <MovingSteppersLib.h>
#include <MotorEncoderLib.h>

MovingSteppersLib J3(37, 33, 10);
//MovingSteppersLib J5(47, 22, 11);
//double target1 = 0.0;
//double target2 = 90.0;
int directionPin = 2;
int stepPin = 3;
//
//unsigned long curTime = 0;
//unsigned long curTime2 = 0;

void setup() {
  // put your setup code here, to run once:
  //Serial.begin(115200);
  pinMode(directionPin, OUTPUT);
  pinMode(stepPin, OUTPUT);
}

void loop() {


//   digitalWrite(directionPin, HIGH);
//   
//   digitalWrite(stepPin, HIGH);
//   delayMicroseconds(300);
//   digitalWrite(stepPin, LOW);
//   delayMicroseconds(300);
  // put your main code here, to run repeatedly:
  // --> J3:
//  Serial.println("Before first J3 move");
//  Serial.print("target value: ");
//  Serial.println(target1);
//  Serial.println(45.51111 * target1);
//  Serial.print("encoder value: ");
//  Serial.println(J3.encoder.getPositionSPI(14));
////  Serial.print("CS: ");
////  Serial.println(J3.encoder.CS);
//  Serial.print("direction: ");
////  Serial.println(digitalRead(J3.dirPin));
//  Serial.println(digitalRead(22));
//
//  J3.move(target1, 1);
//
//  Serial.println("After first J3 move");
//  Serial.print("encoder value: ");
//  Serial.println(J3.encoder.getPositionSPI(14));
//
//  Serial.println("before second J3 move");
//  Serial.print("target value: ");
//  Serial.println(target2);
//  Serial.println(45.51111 * target2);
//  Serial.print("encoder value: ");
//  Serial.println(J3.encoder.getPositionSPI(14));
//  Serial.print("CS: ");
//  Serial.println(J3.encoder.CS);
//  Serial.print("direction: ");
//  Serial.println(digitalRead(J3.dirPin));
//  Serial.println(digitalRead(22));
//
//  J3.move(target2, 1);
//
//  Serial.println("after second J3 move");
//  Serial.print("encoder value: ");
  Serial.println(J3.encoder.getPositionSPI(14));

  J3.encoder.setZeroSPI(10); //set current encoder position 

 // J5.move(target1, 1);
  


}
