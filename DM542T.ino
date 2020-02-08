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
	delay(5);
	digitalWrite(stepPin, LOW);
	delay(5);
	Distance = Distance + 1;
}
	if (Distance == 6400)
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