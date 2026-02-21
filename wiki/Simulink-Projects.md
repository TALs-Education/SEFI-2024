# Simulink Projects

This page describes the MATLAB/Simulink simulation projects included in the `HomeExperiment_Simulink/` directory.

## dcMotorControlSim.slx

- DC motor plant modeling
- Control system design and simulation
- Parameter tuning and analysis

## HomeExperiment.slx & HomeExperiment_PositionControl.slx

- Position control implementation
- Integration with Arduino hardware
- Real-time control via Simulink

**Configuration**: Run `dcMotorControlInit.m` to initialize simulation parameters (sample time: 0.01s)

## Prerequisites

- MATLAB R2018b or later
- Simulink
- Simulink Support Package for Arduino Hardware (optional, for hardware-in-the-loop)

## Running a Simulation

1. Open MATLAB and navigate to the `HomeExperiment_Simulink/` directory
2. Run `dcMotorControlInit.m` to initialize parameters
3. Open the desired `.slx` model file
4. Run the simulation or deploy to hardware

## Simulink Parameters

| Parameter     | Value           |
|---------------|-----------------|
| Sample Time   | 0.01s (100 Hz)  |
| MATLAB Version | R2018b or later |

---

← [[Arduino Projects]] | [[HexaMotor Platform]] →
