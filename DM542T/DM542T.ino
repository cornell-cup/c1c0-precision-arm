int stepPin = 5;
int dirPin = 6;
int enblPin = 7;
int Distance = 0;

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

for(int x=0; x<1600; x++){
	digitalWrite(stepPin, HIGH);
	delayMicroseconds(500);
	digitalWrite(stepPin, LOW);
	delayMicroseconds(500);
	Distance = Distance + 1;
}
	if (Distance == 25600)
	{
	    if (digitalRead(dirPin) == LOW)
	    {
	        digitalWrite(dirPin, HIGH);
	    }
	    else
	    {
	       digitalWrite(dirPin, LOW);
	    }
		Distance = 0;
		delay(2000);
	}
}
