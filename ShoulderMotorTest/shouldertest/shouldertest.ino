int driverPUL = 13;
int driverDIR = 12;

int pd = 500;
boolean setdir = LOW;

void revmotor() {
  setdir = !setdir;
}

void setup() {
  // put your setup code here, to run once:
  pinMode(driverPUL, OUTPUT);
  pinMode(driverDIR, OUTPUT);
  //attachInterrupt(digitalPinToInterrupt(reverseSwitch), revmotor, FALLING);
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(driverDIR, setdir);
  digitalWrite(driverPUL, HIGH);
  delayMicroseconds(pd);
  digitalWrite(driverPUL, LOW);
  delayMicroseconds(pd);
}
