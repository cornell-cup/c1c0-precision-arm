#define driverPUL 10
#define driverDIR 9
#define EB 3
#define EA 2
int stepPerRev = 200;
int pd = 1000 * 1000;
int wait = 2;
volatile bool setdir = LOW;
int encoderCnt = 0;
int ea_on = 0;
int eb_on = 0;

void revmotor()
{
  setdir = !setdir;
  Serial.println("reversing");
}
void step()
{
  for (int i = 0; i < 10; i++)
  {
    digitalWrite(driverDIR, setdir);
    digitalWrite(driverPUL, HIGH);
    delayMicroseconds(pd);
    digitalWrite(driverPUL, LOW);
    delayMicroseconds(wait * pd);
  }
}
void ea()
{
  ea_on = 1;

}
void eb()
{
  eb_on = 1;

}
void setup()
{
  Serial.begin(115200);
  // put your setup code here, to run once:
  pinMode(driverPUL, OUTPUT);
  pinMode(driverDIR, OUTPUT);
  Serial.println("setup");
  attachInterrupt(EA,ea,RISING);
  digitalWrite(driverPUL, LOW);
  digitalWrite(driverDIR, HIGH);
}

void loop()
{
  digitalWrite(driverDIR, !setdir);
  digitalWrite(driverPUL, HIGH);
  delay(1);
  digitalWrite(driverPUL, LOW);
  delay(10);
}
