#include <Servo.h>

Servo myservo;  // create servo object to control a servo
// twelve servo objects can be created on most boards
int pos = 0;    // variable to store the servo position  
int desired_pos = 150;  
int current_pos;

void setup() {
  Serial.begin(9600);
  myservo.write(0);
  myservo.attach(7);  // attaches the servo on pin 9 to the servo object
}

void step_reg_servo(){
  current_pos = myservo.read(); //determine the current position of the regular
    if (abs(desired_pos - current_pos) < 1){
      myservo.detach();
    }
    else if (abs(desired_pos - current_pos) >= 1){
      if ((desired_pos - current_pos) <0){
        myservo.write(current_pos-1);
      }  

      else if ((desired_pos - current_pos) >0){
        myservo.write(current_pos+1);
      }
    }
}

void loop() {
  myservo.write(45); // rotate the motor counterclockwise
  delay(1000); // keep rotating for 5 seconds (5000 milliseconds)

  myservo.write(90); // stop the motor

  delay(1000); // stay stopped

  myservo.write(135); // rotate the motor clockwise

  delay(1000); // keep rotating :D
    // step_reg_servo();
    // Serial.println("Current Position:");
    // Serial.println(current_pos);
    // Serial.println("Desired Position");
    // Serial.println(desired_pos);
    // Serial.println(myservo.attached());
}

    
//  current_pos = myservo.read();
//  if (desired_pos - current_pos > 0) {
//    for (pos = current_pos; pos <= desired_pos; pos += 1) { // goes from current position to desired position... should be just <?
//      // in steps of 1 degree
//      myservo.write(pos);              // tell servo to go to position in variable 'pos'
//      Serial.println(pos);
//      delay(50);                       // waits 50ms for the servo to reach the position
//    }
//  }
//  else if(desired_pos - current_pos < 0) {
//    for (pos = current_pos; pos >= desired_pos; pos -= 1) { // goes from current position to desired position... should be just >?
//      // in steps of 1 degree
//      myservo.write(pos);              // tell servo to go to position in variable 'pos'
//      Serial.println(pos);
//      delay(50);                       // waits 50ms for the servo to reach the position
//    }
//  }
