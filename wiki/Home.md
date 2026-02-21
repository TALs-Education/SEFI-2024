# SEFI-2024: Control Systems Educational Materials

Welcome to the SEFI-2024 Wiki! This repository contains educational materials and experimental setups presented at the **SEFI (European Society for Engineering Education) 2024 Conference** by Tel Aviv University. The materials focus on hands-on control systems education through Arduino-based home experiments and MATLAB/Simulink simulations.

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

## 📚 Wiki Pages

- [[Arduino Projects]] – Arduino-based home experiment details
- [[Simulink Projects]] – MATLAB/Simulink simulation details
- [[HexaMotor Platform]] – HexaMotor advanced control experiments
- [[Getting Started]] – Prerequisites, hardware setup, and installation guide

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

## 📜 License

This project is licensed under the GNU General Public License v3.0 – see the [LICENSE](../LICENSE) file for details.

## 🤝 Contributing

This repository contains educational materials from the SEFI-2024 Conference. For questions, suggestions, or contributions:

- **Institution**: Tel Aviv University
- **Conference**: SEFI 2024 (European Society for Engineering Education)

## 🔗 Related Resources

- [SEFI Conference](https://www.sefi.be/)
- [Arduino Documentation](https://www.arduino.cc/reference/en/)
- [MATLAB/Simulink Documentation](https://www.mathworks.com/help/simulink/)

---

**Prepared for SEFI-2024 Conference by Tel Aviv University**
