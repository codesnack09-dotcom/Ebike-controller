/* ebike_controller.ino
   Simple e-bike controller prototype (Arduino Uno/Nano)
   - Throttle on A0 (0..1023 -> 0..100%).
   - Brake switch on D4 (active LOW if using internal pullup).
   - Hall sensor on D2 (interrupt) to measure wheel speed (pulses).
   - Battery voltage via voltage divider on A1.
   - PWM output on D3 (use proper motor driver/ESC).
   NOTE: Educational/example only. Use proper hardware & safety!
*/

//////////////////////
// CONFIGURATION
//////////////////////
const int PIN_THROTTLE = A0;
const int PIN_BATT = A1;
const int PIN_HALL = 2;        // INT0 on Uno (D2)
const int PIN_BRAKE = 4;       // digital input
const int PIN_PWM = 3;         // PWM output (Timer2) - check compat with driver

// throttle calibration
const int THROTTLE_MIN = 30;   // raw ADC min (to ignore noise)
const int THROTTLE_MAX = 1000; // raw ADC max

// battery divider calibration: Vbatt = reading * scale
// Example: if divider makes 48V -> ~4.9V at ADC, scale = 48 / 4.9
const float BATT_SCALE = 48.0 / 4.9; // adjust according to your divider

// safety limits
const float BATT_CUTOFF_V = 36.0;  // low-voltage cutoff (example)
const float BATT_RECONNECT_V = 38.0;
const int MAX_RPM = 400.0;         // maximum allowed RPM (safety)
const int PULSES_PER_REV = 1;      // pulses per wheel revolution (hall sensor)

// PWM mapping
const int PWM_MIN = 0;     // no power
const int PWM_MAX = 255;   // full power (8-bit PWM)

// speed measurement filter
volatile unsigned long lastPulseTime = 0;
volatile unsigned long pulseIntervalMicros = 0;
volatile unsigned long pulseCount = 0;

unsigned long lastSpeedCalc = 0;
float currentRPM = 0.0;

bool batteryCut = false;

//////////////////////
// SETUP
//////////////////////
void setup() {
  Serial.begin(115200);
  pinMode(PIN_BRAKE, INPUT_PULLUP); // assume brake switch to ground when pressed
  pinMode(PIN_PWM, OUTPUT);

  // attach interrupt for hall sensor
  attachInterrupt(digitalPinToInterrupt(PIN_HALL), hallPulseISR, RISING);

  analogWrite(PIN_PWM, 0); // safety : pwm off
  delay(100);
  Serial.println("eBike controller booting...");
}

//////////////////////
// ISR for hall sensor
//////////////////////
void hallPulseISR() {
  unsigned long now = micros();
  pulseIntervalMicros = now - lastPulseTime;
  lastPulseTime = now;
  pulseCount++;
}

//////////////////////
// HELPER: map throttle to PWM (with deadzone)
//////////////////////
int throttleToPWM(int raw) {
  if (raw < THROTTLE_MIN) return 0;
  if (raw > THROTTLE_MAX) raw = THROTTLE_MAX;
  float norm = (float)(raw - THROTTLE_MIN) / (THROTTLE_MAX - THROTTLE_MIN);
  // optional: apply ramp curve (ease in)
  float curve = norm * norm; // quadratic ramp
  int pwm = (int)(curve * (PWM_MAX - PWM_MIN) + PWM_MIN);
  return constrain(pwm, PWM_MIN, PWM_MAX);
}

//////////////////////
// HELPER: read battery voltage (via ADC and scale)
//////////////////////
float readBatteryVoltage() {
  int adc = analogRead(PIN_BATT);
  // ADC ref = 5V (default)
  float v_adc = (adc / 1023.0) * 5.0;
  float v_batt = v_adc * BATT_SCALE;
  return v_batt;
}

//////////////////////
// HELPER: calculate RPM from pulse interval
//////////////////////
float calculateRPM() {
  noInterrupts();
  unsigned long interval = pulseIntervalMicros;
  unsigned long localCount = pulseCount;
  interrupts();

  if (interval == 0) {
    // no recent pulses: RPM -> 0
    return 0.0;
  }
  // pulses per minute = 60e6 / interval (micros) * (1 / pulses_per_rev)
  float pulsesPerMin = (60000000.0 / (float)interval);
  float rpm = pulsesPerMin / (float)PULSES_PER_REV;
  // apply simple smoothing (basic lowpass)
  static float smoothed = 0.0;
  smoothed = smoothed * 0.7 + rpm * 0.3;
  return smoothed;
}

//////////////////////
// MAIN LOOP
//////////////////////
void loop() {
  unsigned long now = millis();

  // read inputs
  int rawThrottle = analogRead(PIN_THROTTLE);
  bool brakePressed = (digitalRead(PIN_BRAKE) == LOW); // active low
  float battV = readBatteryVoltage();

  // compute RPM occasionally (e.g., 200 ms)
  if (now - lastSpeedCalc > 200) {
    currentRPM = calculateRPM();
    lastSpeedCalc = now;
  }

  // Safety checks
  if (battV < BATT_CUTOFF_V) batteryCut = true;
  if (batteryCut && battV >= BATT_RECONNECT_V) batteryCut = false;

  if (batteryCut) {
    analogWrite(PIN_PWM, 0);
    Serial.print("BATTERY CUTOFF - V=");
    Serial.println(battV);
    delay(200);
    return;
  }

  if (brakePressed) {
    // braking: cut throttle and optionally apply regen if hardware supports
    analogWrite(PIN_PWM, 0);
    Serial.println("BRAKE PRESSED - PWM=0");
    delay(100);
    return;
  }

  // Over-speed protection
  if (currentRPM > MAX_RPM) {
    analogWrite(PIN_PWM, 0);
    Serial.print("OVERSPEED - RPM=");
    Serial.println(currentRPM);
    delay(100);
    return;
  }

  // map throttle to pwm
  int pwm = throttleToPWM(rawThrottle);
  analogWrite(PIN_PWM, pwm);

  // debug output
  if (now % 1000 < 50) { // roughly every 1s
    Serial.print("ThrottleRaw=");
    Serial.print(rawThrottle);
    Serial.print(" PWM=");
    Serial.print(pwm);
    Serial.print(" RPM=");
    Serial.print(currentRPM);
    Serial.print(" Vbatt=");
    Serial.println(battV);
  }

  // small loop delay for stability
  delay(20);
}
