# Arduino Projects

This page describes the Arduino-based home experiment projects included in the `HomeExperiment_Arduino/` directory.

## 1. DimLedEx – LED Dimming Control

A simple introduction to analog input and PWM output.

- **Hardware**: RGB LED, potentiometer
- **Concepts**: Analog-to-Digital Conversion (ADC), Pulse Width Modulation (PWM)
- **Files**: `DimLedEx.ino`, circuit schematics

## 2. MotorControlEx – Basic Motor Control

Introduction to DC motor control with encoder feedback.

- **Hardware**: DC motor with encoder, H-bridge driver
- **Concepts**: Encoder reading, velocity measurement, basic PI control
- **Features**:
  - Real-time RPM measurement
  - Velocity filtering
  - Serial plotting of motor speed

## 3. MotorControlEx_Advanced – PID Motor Control

Advanced motor control implementation with a modular architecture.

- **Hardware**: DC motor with 30:1 gear ratio, quadrature encoder
- **Concepts**: PID control, object-oriented programming, motion profiles
- **Features**:
  - Modular `MotorControl` class library
  - Configurable PID parameters (kp, ki, kd)
  - Multiple velocity measurement methods (dt-based and pulse-based)
  - Motion profile generators (sine, ramp, step)
  - Anti-windup protection
  - 60-second timeout safety feature

## 4. TimeOfFlightSensorEx – Distance Measurement

Distance sensing using VL53L1X Time-of-Flight sensor.

- **Hardware**: SparkFun VL53L1X sensor
- **Concepts**: I2C communication, distance measurement
- **Library**: SparkFun VL53L1X Arduino library

## 📊 Usage Examples

### Running a Basic Motor Control Experiment

1. Wire the DC motor and encoder according to the schematic
2. Upload `MotorControlEx.ino` to your Arduino
3. Open the Serial Plotter (Tools → Serial Plotter)
4. Adjust the potentiometer to set desired speed
5. Observe real-time speed tracking and control performance

### Advanced PID Tuning

1. Upload `MotorControlEx_Advanced.ino`
2. Modify PID gains in the `pidControl()` function:
   ```cpp
   float kp = 0.25;  // Proportional gain
   float ki = 2.5;   // Integral gain
   float kd = 0.001; // Derivative gain
   ```
3. Experiment with different motion profiles (sine, ramp, step)
4. Analyze system response via Serial Plotter

---

← [[Home]] | [[Simulink Projects]] →
