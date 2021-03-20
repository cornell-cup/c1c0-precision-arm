#include <MovingSteppersLib.h>
#include <MotorEncoderLib.h>

#define ledPin 13

MovingSteppersLib J5(47, 22, 10); //instantiate the motors
MovingSteppersLib J3(37, 33, 11); 


double target1 = 0.0;
double target2 = 90.0;

int directionPinJ3 = 33;
int stepPinJ3 = 37;
int moveJ3 = 0;
int stateJ3 = LOW;


int directionPinJ5 = 22;
int stepPinJ5 = 47;
int moveJ5 = 0;
int stateJ5 = LOW;

int targetAngle;

int encoderPos[6]; //put values in array
int encoderDiff[6];
int encoderTarget[6];
int targetAngle[6];

void setup()
{
  Serial.begin(9600);
  pinMode(ledPin, OUTPUT);

  pinMode(directionPinJ5, OUTPUT);
  pinMode(stepPinJ5, OUTPUT);

  moveJ5 = 1;

  pinMode(directionPinJ3, OUTPUT);
  pinMode(stepPinJ3, OUTPUT);

  moveJ3 = 1;

  targetAngle = 60;
  
  encoderTarget = targetAngle * 45.51111;
  encoderPosition = J5.encoder.getPositionSPI(14);
//  Serial.println(encoderTarget);
//  Serial.println(encoderPosition);
  encoderDiff = encoderTarget - encoderPosition;

    // --> set moving direction
  if(encoderDiff > 0){
      digitalWrite(directionPinJ5, HIGH);
  }
  else{
      digitalWrite(directionPinJ5, LOW);
  }

  
  // initialize timer1 
  noInterrupts();           // disable all interrupts
  TCCR1A = 0;
  TCCR1B = 0;

  TCNT1 = 65518;            // preload timer 65536-(16MHz/256/4Hz)
  TCCR1B |= (1 << CS12);    // 256 prescaler 
  TIMSK1 |= (1 << TOIE1);   // enable timer overflow interrupt
  interrupts();             // enable all interrupts
}

ISR(TIMER1_OVF_vect)        // interrupt service routine that wraps a user defined function supplied by attachInterrupt
{
  TCNT1 = 65518;            // preload timer

  digitalWrite(ledPin, digitalRead(ledPin) ^ 1);

   if (moveJ3) { // if J3 should move
    //Serial.println(encoderDiff);
    if (encoderDiff > 10 || encoderDiff < 10){
      stateJ3 = !stateJ3;
      digitalWrite(stepPinJ3, stateJ3);
    }
    else {
      moveJ3 = 0;
    }
   }
  
  if (moveJ5) { // if J5 should move
    //Serial.println(encoderDiff);
    if (encoderDiff > 10 || encoderDiff < 10){
      stateJ5 = !stateJ5;
      digitalWrite(stepPinJ5, stateJ5);
    }
    else {
      moveJ5 = 0;
    }
  }


  
}

void loop()
{

  Serial.println(encoderTarget);
  Serial.println(encoderPosition);
  encoderTarget = targetAngle * 45.51111; //encoder step convertion 
  
  encoderPosition = J5.encoder.getPositionSPI(14);
  encoderDiff = encoderTarget - encoderPosition;
  // int prevEncoder = encoderPosition;
  // int attempt = 0;


  //J5
  if (abs(encoderDiff) >= 8192) { //angle > 180 (encoder units)
        if (encoderDiff > 0) {
         digitalWrite(directionPinJ5, LOW); //J3 Clockwise is LOW
        }
        else {
         digitalWrite(directionPinJ5, HIGH);
        
      }
   }
   else {
        if(encoderDiff > 0){
            digitalWrite(directionPinJ5, HIGH);
          
        }
        else {
            digitalWrite(directionPinJ5, LOW);
          
        }

    }

  // J3
  encoderPosition = J3.encoder.getPositionSPI(14);
  encoderDiff = encoderTarget - encoderPosition;
  // int prevEncoder = encoderPosition;
  // int attempt = 0;


  if (abs(encoderDiff) >= 8192) { //angle > 180 (encoder units)
        if (encoderDiff > 0) {
         digitalWrite(directionPinJ3, LOW); //J3 Clockwise is LOW
        }
        else {
         digitalWrite(directionPinJ3, HIGH);
        
      }
   }
   else {
        if(encoderDiff > 0){
            digitalWrite(directionPinJ3, HIGH);
          
        }
        else {
            digitalWrite(directionPinJ3, LOW);
          
        }

    }
  
}

void checkDir(MovingSteppersLib motor,int motorNum,int directionPin){
  encoderPosition[motorNum] = motor.encoder.getPositionSPI(14);
  encoderDiff[motorNum] = encoderTarget[motorNum] - encoderPosition[motorNum];

  if (abs(encoderDiff[motorNum]) >= 8192) { //angle > 180 (encoder units)
        if (encoderDiff[motorNum] > 0) {
         digitalWrite(directionPin[motorNum], LOW); //J3 Clockwise is LOW
        }
        else {
        digitalWrite(directionPin[motorNum], HIGH);
        
      }
   }
   else {
        if(encoderDiff[motorNum] > 0){
            digitalWrite(directionPin[motorNum], HIGH);
          
        }
        else {
            digitalWrite(directionPin[motorNum], LOW);
          
        }

    }
}
