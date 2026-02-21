# Getting Started

This page covers prerequisites, hardware setup, and installation instructions.

## Prerequisites

### For Arduino Projects

- Arduino IDE (1.8.x or later) or Arduino CLI
- Arduino Uno or compatible board
- Home Experiment Shield (see documentation in `HomeExperiment_Arduino/`)
- Required Arduino libraries:
  - SparkFun VL53L1X (for TimeOfFlightSensorEx)

### For Simulink Projects

- MATLAB R2018b or later
- Simulink
- Simulink Support Package for Arduino Hardware (optional, for hardware-in-the-loop)

## Hardware Setup

Refer to the PDF documentation:

- `HomeExperiment_Arduino/Home kit TAU-ARD V01.pdf` – Detailed hardware setup guide
- `HexaMotor/HexaMotor_VelocityControl/HexaMotor Velocity Control.pdf` – HexaMotor documentation

## Installation

### 1. Clone the repository

```bash
git clone https://github.com/TALs-Education/SEFI-2024.git
cd SEFI-2024
```

### 2. For Arduino projects

- Open the desired `.ino` file in Arduino IDE
- Install required libraries via Library Manager
- Connect your Arduino board
- Upload the sketch

### 3. For Simulink projects

- Open MATLAB and navigate to the `HomeExperiment_Simulink/` directory
- Run `dcMotorControlInit.m` to initialize parameters
- Open the desired `.slx` model file
- Run the simulation or deploy to hardware

---

← [[HexaMotor Platform]] | [[Home]] →
