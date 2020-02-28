#include <MovingSteppersLib.h>
#include <MotorEncoderLib.h>




MovingSteppersLib J3(6, 7, 2);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println("before first move");
  Serial.println(J3.prevAngle);
  Serial.println(digitalRead(J3.dirPin));
  J3.moveJ3(360.0);
  Serial.println("after first move");
  Serial.println(J3.prevAngle);
  Serial.println(digitalRead(J3.dirPin));
  J3.moveJ3(180.0);
  Serial.println("after second move");
}
