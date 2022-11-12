#include <Servo.h>

Servo myservo;  // create servo object to control a servo
// twelve servo objects can be created on most boards
int pos = 0;    // variable to store the servo position  
int desired_pos = 0;  
int current_pos;
int input;

void setup() {
  Serial.begin(9600);
  myservo.write(0);
  myservo.attach(9);  // attaches the servo on pin 9 to the servo object
}

void loop() {
  if(Serial.available()) {
      Serial.println("Type desired angle: ");
      input = Serial.parseInt();
      Serial.print("You typed: ");
      Serial.println(input);
      desired_pos = input;
  }

  current_pos = myservo.read();
  if (desired_pos - current_pos > 0) {
    for (pos = current_pos; pos <= desired_pos; pos += 1) { // goes from current position to desired position... should be just <?
      // in steps of 1 degree
      myservo.write(pos);              // tell servo to go to position in variable 'pos'
      Serial.println(pos);
      delay(50);                       // waits 50ms for the servo to reach the position
    }
  }
  else if(desired_pos - current_pos < 0) {
    for (pos = current_pos; pos >= desired_pos; pos -= 1) { // goes from current position to desired position... should be just >?
      // in steps of 1 degree
      myservo.write(pos);              // tell servo to go to position in variable 'pos'
      Serial.println(pos);
      delay(50);                       // waits 50ms for the servo to reach the position
    }
  }
}
