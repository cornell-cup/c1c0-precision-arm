#include <MovingSteppersLib.h>
#include <MotorEncoderLib.h>

MovingSteppersLib J3(47, 22, 10);
//MovingSteppersLib J5(8, 9, 48);
double target1 = 0;
double target2 = 315;
//int directionPin = 22;
//int stepPin = 47;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
//  pinMode(directionPin, OUTPUT);
//  pinMode(stepPin, OUTPUT);
}

void loop() {

  // put your main code here, to run repeatedly:
  // --> J3:
  Serial.println("Before first J3 move");
  Serial.print("target value: ");
  Serial.println(target1);
  Serial.println(45.51111 * target1);
  Serial.print("encoder value: ");
  Serial.println(J3.encoder.getPositionSPI(14));
//  Serial.print("CS: ");
//  Serial.println(J3.encoder.CS);
  Serial.print("direction: ");
//  Serial.println(digitalRead(J3.dirPin));
  Serial.println(digitalRead(22));

  J3.move(target1, 1);

  Serial.println("After first J3 move");
  Serial.print("encoder value: ");
  Serial.println(J3.encoder.getPositionSPI(14));

  Serial.println("before second J3 move");
  Serial.print("target value: ");
  Serial.println(target2);
  Serial.println(45.51111 * target2);
  Serial.print("encoder value: ");
  Serial.println(J3.encoder.getPositionSPI(14));
  Serial.print("CS: ");
  Serial.println(J3.encoder.CS);
  Serial.print("direction: ");
//  Serial.println(digitalRead(J3.dirPin));
  Serial.println(digitalRead(22));

  J3.move(target2, 1);

  Serial.println("after second J3 move");
  Serial.print("encoder value: ");
  Serial.println(J3.encoder.getPositionSPI(14));

}
