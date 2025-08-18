#include <PID_v1.h>

// RPM input from sensor
double rpmInput;    
double rpmSetpoint; 
double pwmOutput;   

// PID tuning parameters
double Kp = 1.0, Ki = 0.5, Kd = 0.1;
PID myPID(&rpmInput, &pwmOutput, &rpmSetpoint, Kp, Ki, Kd, DIRECT);

// Current sensor
const int currentPin = A0;
double currentValue;
double currentLimit = 30.0; // Maximum current in Amps

// Throttle / Mode
int throttlePin = A1;
int modePin = A2;
double throttleCurve = 1.0; 
bool ecoMode = true;

void setup() {
  Serial.begin(115200);
  myPID.SetMode(AUTOMATIC);
  pinMode(currentPin, INPUT);
  pinMode(throttlePin, INPUT);
  pinMode(modePin, INPUT);
}

void loop() {
  // Read mode (eco/sport)
  int modeVal = analogRead(modePin);
  ecoMode = (modeVal < 512);
  throttleCurve = ecoMode ? 0.6 : 1.0; // eco = 60% power, sport = 100%

  // Read target RPM from throttle
  int throttleVal = analogRead(throttlePin);
  rpmSetpoint = map(throttleVal, 0, 1023, 0, 5000) * throttleCurve; // max 5000 RPM

  // Read RPM from sensor (hall/encoder)
  rpmInput = readRPM();

  // Compute PID
  myPID.Compute();

  // Read current
  currentValue = readCurrent();
  if(currentValue > currentLimit){
    pwmOutput = 0; // Cut-off overcurrent
  }

  // Send PWM to motor driver
  analogWrite(9, constrain(pwmOutput, 0, 255));

  delay(10);
}

// Function to read RPM (replace with actual sensor code)
double readRPM(){
  return 0; 
}

// Function to read current from ACS712
double readCurrent(){
  int sensorVal = analogRead(currentPin);
  double voltage = sensorVal * (5.0 / 1023.0);
  double current = (voltage - 2.5) / 0.066; // ACS712 30A version
  return current;
}
