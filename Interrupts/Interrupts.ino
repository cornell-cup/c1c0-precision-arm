#include <MovingSteppersLib.h>
#include <MotorEncoderLib.h>

#define ledPin 13

MovingSteppersLib J5(47, 22, 10);

double target1 = 0.0;
double target2 = 90.0;

int directionPin = 22;
int stepPin = 47;
int moveJ5 = 0;
int stateJ5 = LOW;

int encoderTarget;
int encoderPosition;
int encoderDiff;

int targetAngle;

void setup()
{
  Serial.begin(9600);
  pinMode(ledPin, OUTPUT);

  pinMode(directionPin, OUTPUT);
  pinMode(stepPin, OUTPUT);

  moveJ5 = 1;

  targetAngle = 60;
  
  encoderTarget = targetAngle * 45.51111;
  encoderPosition = J5.encoder.getPositionSPI(14);
//  Serial.println(encoderTarget);
//  Serial.println(encoderPosition);
  encoderDiff = encoderTarget - encoderPosition;

    // --> set moving direction
  if(encoderDiff > 0){
      digitalWrite(directionPin, HIGH);
  }
  else{
      digitalWrite(directionPin, LOW);
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
  
  if (moveJ5) { // if J5 should move
    //Serial.println(encoderDiff);
    if (encoderDiff > 10 || encoderDiff < 10){
      stateJ5 = !stateJ5;
      digitalWrite(stepPin, stateJ5);
    }
  }

  
}

void loop()
{

  Serial.println(encoderTarget);
  Serial.println(encoderPosition);
  encoderTarget = targetAngle * 45.51111;
  
  encoderPosition = J5.encoder.getPositionSPI(14);
  encoderDiff = encoderTarget - encoderPosition;
  // int prevEncoder = encoderPosition;
  // int attempt = 0;


  if (abs(encoderDiff) >= 8192) { //angle > 180 (encoder units)
        if (encoderDiff > 0) {
         digitalWrite(directionPin, LOW); //J3 Clockwise is LOW
        }
        else {
         digitalWrite(directionPin, HIGH);
        
      }
   }
   else {
        if(encoderDiff > 0){
            digitalWrite(directionPin, HIGH);
          
        }
        else {
            digitalWrite(directionPin, LOW);
          
        }

    }
  
}
