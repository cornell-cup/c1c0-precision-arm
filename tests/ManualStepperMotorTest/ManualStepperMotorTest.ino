// Pin definitions
#define EB 3
#define EA 2

int stepPerRev = 200;
int pd = 1000 * 1000;
int wait = 2;
volatile bool setdir = LOW;
int encoderCnt = 0;

// Step (pulse) pins - blue
int stepPins[] = {49, 46, 43, 40, 37, 34};

// Direction pins - yellow
int dirPins[] = {48, 45, 42, 39, 36, 33};

// Chip select pins
int chipSelectPins[] = {47, 9, 41, 38, 35, 32};

int motorIndex = 0;

volatile int driverDIR;
volatile int driverPUL;

int input = 0;
bool moving = false;  // To track if the motor is moving

void setup()
{
  Serial.begin(115200);

  // Setup pins for all motors
  for (int i = 0; i < 6; i++) {
    pinMode(stepPins[i], OUTPUT);
    pinMode(dirPins[i], OUTPUT);
  }

  // Initialize motor state
  driverPUL = stepPins[motorIndex];
  driverDIR = dirPins[motorIndex];
  digitalWrite(driverPUL, LOW);
  digitalWrite(driverDIR, LOW);

  Serial.println("Setup complete");
}

void loop()
{
  // Check for new serial input
  if (Serial.available() > 0) {
    String inputStr = Serial.readStringUntil('\n');  // Read until newline character

    // Select motor based on input (e.g., "j1", "j2", etc.)
    if ((inputStr.startsWith("j") | inputStr.startsWith("J")) && inputStr.length() == 2) {
      motorIndex = inputStr[1] - '0' - 1;  // Get motor number from string (e.g., '0' to '5')
      Serial.println(motorIndex);
      if (motorIndex >= 0 && motorIndex < 6) {
        driverDIR = dirPins[motorIndex];
        driverPUL = stepPins[motorIndex];
        Serial.print("\nSwitched to Motor ");
        Serial.println(motorIndex + 1);
      }
    }

    input = inputStr.toInt();  // Convert input to integer
    
    // If the input string was not a valid number, ignore
    if (inputStr.length() == 0 || inputStr == "0" && moving == false) {
      return;  // Avoid repeated processing of 0 if no valid command is given
    }

    Serial.print("Input received: ");
    Serial.println(input);

    // Set direction based on input (positive for forward, negative for reverse)
    if (input > 0) {
      setdir = HIGH;
      moving = true;  // Start moving
    } else if (input < 0) {
      setdir = LOW;
      moving = true;  // Start moving
    } else {
      moving = false;  // Stop motor if input is 0
    }

    // Set motor direction immediately
    digitalWrite(driverDIR, setdir);
  }

  // Move the motor continuously if `moving` is true
  if (moving) {
    motor_move(driverDIR, driverPUL);
  }
  
  delay(3);  // Small delay to avoid overwhelming the loop
}

void motor_move(int driverDIR, int driverPUL) {
  // Pulse to move motor continuously
  digitalWrite(driverPUL, HIGH);
  delayMicroseconds(500);  // Pulse on time
  digitalWrite(driverPUL, LOW);
  delayMicroseconds(500);  // Pulse off time
}

