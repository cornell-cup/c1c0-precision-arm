int stepPin = 5;
int dirPin = 34;
int enblPin =7;
int Distance = 0;
int STATE = 0;

void setup() {
	pinMode (stepPin, OUTPUT);
	pinMode (dirPin, OUTPUT);
	pinMode (enblPin, OUTPUT);
	digitalWrite(stepPin, LOW);
	digitalWrite(dirPin, LOW);
	digitalWrite(enblPin, HIGH);
}

void loop() {

Serial.println("test");

for(int x=0; x<2000; x++){
	digitalWrite(stepPin, HIGH);
	delayMicroseconds(500);
	digitalWrite(stepPin, LOW);
	delayMicroseconds(500);
	Distance = Distance + 1;
}
	if (Distance == 10000)
	{
	    if (STATE==0)
	    {
	        digitalWrite(dirPin, HIGH);
         STATE = 1;
	    }
	    else
	    {
	       digitalWrite(dirPin, LOW);
        STATE = 0;
	    }
		Distance = 0;
		delay(2000);
	}
}
