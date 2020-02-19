#include <MovingSteppersLib.h>




//MovingSteppersLib J5 = MovingStepperLib(10, 11);
MovingSteppersLib J5(10,11);
void setup() {
  // put your setup code here, to run once:
}

void loop() {
  // put your main code here, to run repeatedly:
  J5.moveJ5(0,360.0);
}
