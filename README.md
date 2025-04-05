# Aerospace-Engineering
# Moon Landing Simulation

This project performs a simulation of a spacecraft landing on the Moon, using a physical model of gravitational forces, fuel consumption, and counteracting forces during descent. The goal of the simulation is to land the spacecraft on the Moon with vertical and horizontal velocities less than 2.5 m/s, while conserving as much fuel as possible (aiming to have at least 50 liters of fuel left at the end of the mission).

## Project Goal:
The simulation aims to explore the possibility of autonomous Moon landings, tackling technical challenges such as maintaining low landing velocities, calculating engine thrust, and modeling fuel consumption. The goal is to avoid crashing while maximizing fuel conservation.

## Requirements:

- Python 3.x
- Libraries:
  - `numpy`
  - `matplotlib`

## Code Structure:

### 1. **Simulation Code**:
The code simulates the entire process of landing on the Moon. It includes models for gravitational force, thrust from spacecraft engines, and fuel consumption over time.

### 2. **PID Controller**:
The simulation uses a PID (Proportional-Integral-Derivative) controller to regulate the vertical velocity, horizontal velocity, and attitude of the spacecraft. The PID controller helps to plan the spacecraft’s actions and achieve the target velocities during descent.

### 3. **Data and Graphs**:
The simulation collects data on altitude, vertical and horizontal velocities, fuel remaining, and spacecraft attitude over time. Graphs are generated to visualize the changes in these parameters.

### 4. **Results**:
Based on the simulation, we can evaluate how the PID algorithm influences the landing process and whether the spacecraft successfully avoids crashing while maintaining low velocities and conserving fuel.

## Installation and Execution:

### 1. Install required libraries:
```bash
pip install numpy matplotlib
