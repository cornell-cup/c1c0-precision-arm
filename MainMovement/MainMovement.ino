#include <MovingSteppersLib.h>
#include <MotorEncoderLib.h>




MovingSteppersLib J2(6, 7, 49);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println("before first move");
  Serial.print("target value: ");
  Serial.println(45.51111 * 120);
  Serial.print("encoder value: ");
  Serial.println(J2.encoder.getPositionSPI(14));
  Serial.println(digitalRead(J2.dirPin));
  J2.moveJ3(359.0);
  Serial.println("after first move");
  Serial.print("target value: ");
  Serial.println(45.51111 * 280);
  Serial.println(J2.prevAngle);
  Serial.println(digitalRead(J2.dirPin));
  J2.moveJ3(0.0);
  Serial.println("after second move");
}
