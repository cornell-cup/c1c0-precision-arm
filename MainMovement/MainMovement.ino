#include <MovingSteppersLib.h>
#include <MotorEncoderLib.h>

MovingSteppersLib J3(6, 7, 49);
MovingSteppersLib J5(8, 9, 48);


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println("before first J3 move");
  Serial.print("target value: ");
  Serial.println(45.51111 * 120);
  Serial.print("encoder value: ");
  Serial.println(J3.encoder.getPositionSPI(14));
  Serial.print("CS: ");
  Serial.println(J3.encoder.CS);
  Serial.println(digitalRead(J3.dirPin));
  J3.moveJ3(359.0);
  Serial.println("after first J3 move");
  Serial.print("target value: ");
  Serial.println(45.51111 * 280);
  Serial.println(J3.prevAngle);
  Serial.println(digitalRead(J3.dirPin));
  J3.moveJ3(0.0);
  Serial.println("after second J3 move");


  Serial.println("before first J5 move");
  Serial.print("target value: ");
  Serial.println(45.51111 * 120);
  Serial.print("encoder value: ");
  Serial.println(J5.encoder.getPositionSPI(14));
  Serial.print("CS: ");
  Serial.println(J5.encoder.CS);
  Serial.println(digitalRead(J5.dirPin));
  J5.moveJ5(359.0);
  Serial.println("after first J5 move");
  Serial.print("target value: ");
  Serial.println(45.51111 * 280);
  Serial.println(J5.prevAngle);
  Serial.println(digitalRead(J5.dirPin));
  J5.moveJ5(0.0);
  Serial.println("after second J5 move");
}
