#include <Servo.h>

Servo myservo;  // create servo object to control a servo
// twelve servo objects can be created on most boards

int speed;


void setSpeed(int speed) {
  //between 0 and 180 
  myservo.write(speed);
}

void setup() {
  // servo library only supports pins 9 and 10 

  // write(n) controls the speed (forward 0 to 89, backward 91 to 180)
  Serial.begin(9600);
  myservo.attach(7);  // attaches the servo on pin 7 to the servo object
  Serial.println(myservo.attached()); 
  myservo.writeMicroseconds(1500);
  myservo.write(90);
}

void loop() {
  // for continuous servo, write(n) sets the speed
  //setSpeed(180);
  //delay(200);
  myservo.write(140);
  delay(200);
  // delay(2000);
  //myservo.write(50);
  //delay(2000); 
  // myservo.write(90);
  // delay(2000);
}

