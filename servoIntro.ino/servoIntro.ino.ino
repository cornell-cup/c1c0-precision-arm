
#include <Servo.h>

Servo myservo;  // create servo object to control a servo
// twelve servo objects can be created on most boards

int pos = 0;    // variable to store the servo position

void setup() {
  // servo library only supports pins 9 and 10 
  Serial.begin(9600);
  myservo.attach(9);  // attaches the servo on pin 9 to the servo object
  Serial.println(myservo.attached()); 
   myservo.write(80);
  delay(200);
  myservo.write(90);
}

void loop() {
// for continuous servo, write(n) sets the speed

//  for (pos = 0; pos <= 30; pos += 1) { // goes from 0 degrees to 180 degrees
//    // in steps of 1 degree
//    myservo.write(pos);              // tell servo to go to position in variable 'pos'
//    delay(15);                       // waits 15ms for the servo to reach the position
//  }
//  for (pos = 30; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
//    myservo.write(pos);              // tell servo to go to position in variable 'pos'
//    delay(15);                       // waits 15ms for the servo to reach the position
//  }
}
