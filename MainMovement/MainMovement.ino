#include <MovingSteppersLib.h>
#include <MotorEncoderLib.h>

MovingSteppersLib J3(47, 43, 10);
//MovingSteppersLib J5(8, 9, 48);
double target1 = 40.0;
double target2 = 45.0;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
  // --> J3:
  Serial.println("Before first J3 move");
  Serial.print("target value: ");
  Serial.println(45.51111 * target1);
  Serial.print("encoder value: ");
  Serial.println(J3.encoder.getPositionSPI(14));
//  Serial.print("CS: ");
//  Serial.println(J3.encoder.CS);
  Serial.print("direction: ");
  Serial.println(digitalRead(J3.dirPin));

  J3.move(target1, 1);

  Serial.println("After first J3 move");
  Serial.print("encoder value: ");
  Serial.println(J3.encoder.getPositionSPI(14));

  Serial.println("before second J3 move");
  Serial.print("target value: ");
  Serial.println(45.51111 * target2);
  Serial.print("encoder value: ");
  Serial.println(J3.encoder.getPositionSPI(14));
  Serial.print("CS: ");
  Serial.println(J3.encoder.CS);
  Serial.print("direction: ");
  Serial.println(digitalRead(J3.dirPin));

  J3.move(target2, 1);

  Serial.println("after second J3 move");
  Serial.print("encoder value: ");
  Serial.println(J3.encoder.getPositionSPI(14));

}
