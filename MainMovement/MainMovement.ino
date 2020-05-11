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
  // --> J3:
  Serial.println("before first J3 move");
  Serial.print("target value: ");
  Serial.println(45.51111 * 120);
  Serial.print("encoder value: ");
  Serial.println(J3.encoder.getPositionSPI(14));
  Serial.print("CS: ");
  Serial.println(J3.encoder.CS);
  Serial.print("direction: ");
  Serial.println(digitalRead(J3.dirPin));

  J3.move(120.0, 1);

  Serial.println("after first J3 move");
  Serial.print("encoder value: ");
  Serial.println(J3.encoder.getPositionSPI(14));

  Serial.println("before second J3 move");
  Serial.print("target value: ");
  Serial.println(45.51111 * 0);
  Serial.print("encoder value: ");
  Serial.println(J3.encoder.getPositionSPI(14));
  Serial.print("CS: ");
  Serial.println(J3.encoder.CS);
  Serial.print("direction: ");
  Serial.println(digitalRead(J3.dirPin));

  J3.move(0.0, 1);

  Serial.println("after second J3 move");
  Serial.print("encoder value: ");
  Serial.println(J3.encoder.getPositionSPI(14));

  // --> J5
  Serial.println("before first J5 move");
  Serial.print("target value: ");
  Serial.println(45.51111 * 120);
  Serial.print("encoder value: ");
  Serial.println(J5.encoder.getPositionSPI(14));
  Serial.print("CS: ");
  Serial.println(J5.encoder.CS);
  Serial.print("direction: ");
  Serial.println(digitalRead(J5.dirPin));

  J5.move(120.0, 1);

  Serial.println("after first J5 move");
  Serial.print("encoder value: ");
  Serial.println(5.encoder.getPositionSPI(14));

  Serial.println("before second J5 move");
  Serial.print("target value: ");
  Serial.println(45.51111 * 0);
  Serial.print("encoder value: ");
  Serial.println(J5.encoder.getPositionSPI(14));
  Serial.print("CS: ");
  Serial.println(J5.encoder.CS);
  Serial.print("direction: ");
  Serial.println(digitalRead(J5.dirPin));

  J5.move(0.0, 1);

  Serial.println("after second J5 move");
  Serial.print("encoder value: ");
  Serial.println(J5.encoder.getPositionSPI(14));
}
