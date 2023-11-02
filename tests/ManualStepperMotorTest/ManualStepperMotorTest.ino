#define driverPUL 46
#define driverDIR 42
#define EB 3
#define EA 2
int stepPerRev = 200;
int pd = 1000 * 1000;
int wait = 2;
volatile bool setdir = LOW;
int encoderCnt = 0;

void setup()
{
  Serial.begin(115200);
  // put your setup code here, to run once:
  pinMode(driverPUL, OUTPUT);
  pinMode(driverDIR, OUTPUT);
  Serial.println("setup");
  digitalWrite(driverPUL, LOW);
  digitalWrite(driverDIR, HIGH);
}

void loop()
{
  digitalWrite(driverDIR, setdir);
  digitalWrite(driverPUL, HIGH);
  delayMicroseconds(500);
  digitalWrite(driverPUL, LOW);
  delayMicroseconds(400);
}
