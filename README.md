## This Arduino sketch is designed for a simple e-bike controller and includes the most commonly used features:
- Read throttle (potentiometer / sensor 0–5V)
- Read brake switch (brake cut-off)
- Read wheel hall sensor (for speed measurement)
- Monitor battery voltage (via ADC)
- Output PWM to motor driver / MOSFET (motor power control)
- Soft-start & safety cut-offs (low voltage / brake / overspeed)
- Debug output via Serial'''
# ⚠️ IMPORTANT — Safety & Legal Notice
- This is an educational example. Do not directly connect it to a motor or battery without proper protection. E-bike batteries operate at high voltage and current — mishandling is dangerous.
- Ensure your motor driver/ESC is compatible with the PWM output from Arduino, or use an interface such as a level shifter / limiter.
- Always follow traffic and safety regulations in your country.
# Required Components
- Arduino Uno / Nano
- Throttle: potentiometer 0–5V (or throttle sensor output) → analog pin A0
- Brake switch: pushbutton/switch → digital pin (internal pull-up enabled)
- Wheel hall sensor (pulses per revolution) → digital interrupt pin D2
- Battery voltage divider → analog pin A1 (must scale down <5V input to Arduino)
- PWM output to motor driver / ESC input → PWM pin (D3/D5/D6, etc.)
- Motor driver / ESC capable of handling your motor’s current and compatible with PWM
# Circuit Overview (conceptual)
- Throttle (middle wire) → A0
- Battery voltage divider → A1
- Hall sensor → D2 (interrupt pin)
- Brake switch → D4 (with pull-up or pull-down)
- PWM output → D3 (Timer2 PWM)
# Code Explanation
- Throttle: Read via ADC (A0). Mapped to PWM using a quadratic curve for smoother response.
- Brake: If pressed, PWM is cut off immediately. (Note: regenerative braking requires ESC hardware that supports it.)
- Hall sensor: Interrupt measures the time between pulses → converted to RPM. Adjust PULSES_PER_REV according to your sensor setup (1, 2, or more magnets).
- Battery: ADC reads from voltage divider; BATT_SCALE must match your divider ratio (Vbatt max / Vadc max). Includes low-voltage cut-off (BATT_CUTOFF_V) for battery safety.
- Safety: Immediate cut-off on overspeed, low battery, or brake press.
- PWM: Uses analogWrite() 8-bit (0–255). Ensure your ESC/motor driver can handle this signal.
# Tuning & Setup Tips
- Calibrate BATT_SCALE — measure the ADC output with a multimeter at full battery voltage.
- Set PULSES_PER_REV based on your wheel sensor (e.g., 2 magnets → PULSES_PER_REV = 2).
- Adjust MAX_RPM according to gearing and wheel size.
- Never connect the motor directly to Arduino PWM — always use a driver/ESC rated for your current.
- Add a current sensor (e.g., ACS712 or shunt + amplifier) for overcurrent protection.
- Implement ramping (soft-start) to reduce current spikes; this example uses a quadratic throttle curve, but you can also limit PWM step increase per cycle.
- Always use proper braking hardware (mechanical/electronic) — do not rely solely on software braking.
# 1. Required Components
-	BLDC hub or mid-drive motor
-	Motor controller (VESC, BLDC ESC, or MOSFET gate driver)
-	Arduino/ESP32
-	Current sensor (e.g., ACS712 or INA219)
-	Hall sensor or encoder for RPM feedback
-	Potentiometer or Bluetooth module to adjust mode/throttle
-	Li-ion/LiFePO4 battery according to motor rating
-	Wires, resistors, MOSFETs (if using custom gate driver)
 
# 2. Basic Wiring Diagram (Example with VESC)
[Battery +] ----+----> VESC V+  ----> Motor +
[Battery -] ----+----> VESC GND ----> Motor -
Encoder/Hall --> Arduino/ESP32 (RPM input)
ACS712/I2C   --> Arduino/ESP32 (Current reading)
Pot/Bluetooth --> Arduino/ESP32 (Throttle / Mode)
Arduino PWM   --> VESC PWM input (speed control)
Note: If using a MOSFET gate driver, connect the Arduino PWM output to the gate driver input, with MOSFETs rated for motor current.
 
# 3. Arduino / ESP32 Example Code (PID Speed Control + Overcurrent + Eco/Sport)

# 4. Feature Explanation
1.	PID Speed Control – Automatically adjusts PWM to reach target RPM.
2.	Current Sensor & Cut-Off – Monitors motor current and disables PWM if overcurrent occurs.
3.	Eco/Sport Modes – Selectable via potentiometer or Bluetooth input; modifies throttle response.
4.	Throttle Curve – Adjustable multiplier: 0.6 for Eco, 1.0 for Sport.
 
# 5. MOSFET Driver Notes
-	PWM from Arduino/ESP32 drives the gate driver input.
-	Use MOSFETs rated for motor voltage/current.
-	Include flyback diodes and heatsinks to protect MOSFETs.
-	Current sensor can still feed Arduino for overcurrent cut-off.
 
💡 Tip:
You can expand this system with:
•	Bluetooth app for RPM target and mode switching
•	Data logging of RPM/current/battery voltage
•	Soft start / ramping for smoother acceleration
## Support Me
If you find this project useful, you can support me on [ko-fi.com](https://www.ko-fi.com/codesnack).
