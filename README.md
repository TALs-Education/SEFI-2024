# SEFI-2024: Control Systems Educational Materials

This repository contains educational materials and experimental setups presented at the **SEFI (European Society for Engineering Education) 2024 Conference** by Tel Aviv University. The materials focus on hands-on control systems education through Arduino-based home experiments and MATLAB/Simulink simulations.

## 📋 Overview

This repository provides comprehensive resources for teaching control systems engineering concepts through practical experiments. It includes:

- **Arduino-based home experiment kits** for DC motor control
- **MATLAB/Simulink simulations** for control system modeling and analysis
- **HexaMotor platform** experiments for advanced control applications
- Documentation and example code for educational purposes

## 🗂️ Repository Structure

```
SEFI-2024/
├── HomeExperiment_Arduino/     # Arduino-based home experiments
│   ├── DimLedEx/              # LED dimming with potentiometer
│   ├── MotorControlEx/        # Basic motor control with encoder
│   ├── MotorControlEx_Advanced/ # Advanced motor control with PID
│   └── TimeOfFlightSensorEx/  # Time-of-Flight distance sensor
├── HomeExperiment_Simulink/    # MATLAB/Simulink simulations
│   ├── HomeExperiment.slx
│   ├── HomeExperiment_PositionControl.slx
│   └── dcMotorControlSim.slx
├── HexaMotor/                  # HexaMotor platform experiments
│   ├── HexaMotor_VelocityControl/
│   └── HexaMotor_Force_FF.slx
└── Tel Aviv University_SEFI_2024B.pptx  # Conference presentation
```

## 🏠 Home Experiment - Arduino Projects

### 1. DimLedEx - LED Dimming Control
A simple introduction to analog input and PWM output.
- **Hardware**: RGB LED, potentiometer
- **Concepts**: Analog-to-Digital Conversion (ADC), Pulse Width Modulation (PWM)
- **Files**: `DimLedEx.ino`, circuit schematics

### 2. MotorControlEx - Basic Motor Control
Introduction to DC motor control with encoder feedback.
- **Hardware**: DC motor with encoder, H-bridge driver
- **Concepts**: Encoder reading, velocity measurement, basic PI control
- **Features**:
  - Real-time RPM measurement
  - Velocity filtering
  - Serial plotting of motor speed

### 3. MotorControlEx_Advanced - PID Motor Control
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

### 4. TimeOfFlightSensorEx - Distance Measurement
Distance sensing using VL53L1X Time-of-Flight sensor.
- **Hardware**: SparkFun VL53L1X sensor
- **Concepts**: I2C communication, distance measurement
- **Library**: SparkFun VL53L1X Arduino library

## 🔬 Home Experiment - Simulink Projects

MATLAB/Simulink models for control system simulation and analysis:

### dcMotorControlSim.slx
- DC motor plant modeling
- Control system design and simulation
- Parameter tuning and analysis

### HomeExperiment.slx & HomeExperiment_PositionControl.slx
- Position control implementation
- Integration with Arduino hardware
- Real-time control via Simulink

**Configuration**: Run `dcMotorControlInit.m` to initialize simulation parameters (sample time: 0.01s)

## 🤖 HexaMotor Platform

Advanced control experiments using a multi-axis motor platform:

### HexaMotor_VelocityControl
Educational materials for velocity control implementation:
- **Motor Testing**: System characterization
- **System Identification**: Parameter estimation
- **Velocity Control**: Closed-loop speed regulation

### HexaMotor_Force_FF
Feed-forward force control implementation for the HexaMotor platform.

## 🚀 Getting Started

### Prerequisites

**For Arduino Projects:**
- Arduino IDE (1.8.x or later) or Arduino CLI
- Arduino Uno or compatible board
- Home Experiment Shield (see documentation in `HomeExperiment_Arduino/`)
- Required Arduino libraries:
  - SparkFun VL53L1X (for TimeOfFlightSensorEx)

**For Simulink Projects:**
- MATLAB R2018b or later
- Simulink
- Simulink Support Package for Arduino Hardware (optional, for hardware-in-the-loop)

### Hardware Setup

Refer to the PDF documentation:
- `HomeExperiment_Arduino/Home kit TAU-ARD V01.pdf` - Detailed hardware setup guide
- `HexaMotor/HexaMotor_VelocityControl/HexaMotor Velocity Control.pdf` - HexaMotor documentation

### Installation

1. **Clone the repository:**
   ```bash
   git clone https://github.com/TALs-Education/SEFI-2024.git
   cd SEFI-2024
   ```

2. **For Arduino projects:**
   - Open the desired `.ino` file in Arduino IDE
   - Install required libraries via Library Manager
   - Connect your Arduino board
   - Upload the sketch

3. **For Simulink projects:**
   - Open MATLAB and navigate to the `HomeExperiment_Simulink/` directory
   - Run `dcMotorControlInit.m` to initialize parameters
   - Open the desired `.slx` model file
   - Run the simulation or deploy to hardware

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

## 🎓 Educational Applications

These materials are designed for:
- **Undergraduate control systems courses**
- **Hands-on laboratory sessions**
- **Remote/home learning experiments**
- **Control theory demonstrations**
- **Mechatronics education**

## 📝 Key Learning Objectives

Students working with these materials will learn:
- DC motor modeling and control
- PID controller design and tuning
- Encoder-based velocity measurement
- Real-time embedded systems programming
- MATLAB/Simulink simulation and hardware integration
- System identification techniques

## 📄 Documentation

- **Conference Presentation**: `Tel Aviv University_SEFI_2024B.pptx`
- **Hardware Guide**: `HomeExperiment_Arduino/Home kit TAU-ARD V01.pdf`
- **HexaMotor Manual**: `HexaMotor/HexaMotor_VelocityControl/HexaMotor Velocity Control.pdf`

## 📜 License

This project is licensed under the GNU General Public License v3.0 - see the [LICENSE](LICENSE) file for details.

## 🤝 Contributing

This repository contains educational materials from the SEFI-2024 Conference. For questions, suggestions, or contributions:

- **Institution**: Tel Aviv University
- **Conference**: SEFI 2024 (European Society for Engineering Education)

## 🔗 Related Resources

- [SEFI Conference](https://www.sefi.be/)
- [Arduino Documentation](https://www.arduino.cc/reference/en/)
- [MATLAB/Simulink Documentation](https://www.mathworks.com/help/simulink/)

## ⚙️ Technical Specifications

### Arduino Motor Control System
- **Microcontroller**: Arduino Uno (ATmega328P)
- **Motor**: DC motor with 30:1 gear ratio
- **Encoder**: Quadrature encoder (3 PPR, 12 CPR)
- **Control Loop**: 10ms (100 Hz) default, configurable 5-50ms
- **Supply Voltage**: 5V
- **Control Range**: ±255 PWM (±100% duty cycle)
- **Velocity Units**: rad/s or RPM

### Simulink Parameters
- **Sample Time**: 0.01s (100 Hz)
- **Compatible Versions**: MATLAB R2018b and later

---

**Prepared for SEFI-2024 Conference by Tel Aviv University**
