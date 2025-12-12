# Project documentation
# Flywheel Inverted Pendulum – Team-1

As part of the COIL project, we are working together as an international team. There are six of us: three from Lisbon, Portugal, and three from Amsterdam, and we are participating in this international program as a single team. The goal of the project is to stabilize a flywheel-driven inverted pendulum. During the five-week program, we work in labs and online meetings, tackling challenging assignments each week. Along the way, we develop various skills, including Arduino programming, PID control, and working with sensors.

---

## 1. Project Motivation

Inverted pendulums are interesting because they are naturally unstable—without constant adjustment, they fall over. Balancing an inverted pendulum requires continuous control, which is why it’s a classic problem in control theory and engineering. By studying and stabilizing inverted pendulums, we can learn how to design systems that maintain balance and stability, which has applications in robotics, transportation, and many automated systems.

---

## 2.  System Overview

The experimental setup is a flywheel-driven inverted pendulum consisting of mechanical and electronic components.

Mechanical Setup
Pendulum arm: 3D-printed arm that freely rotates around its pivot axis.
Angle sensor: a rotary encoder to measure the arm’s tilt.

Reaction wheel: a 3D-printed flywheel attached to the end of the arm, which is accelerated or decelerated by the motor to stabilize the pendulum.

Drive
DC motor: JGA25-370, 12 V DC motor with an integrated encoder.
The encoder signals are calculate the wheel’s rotational speed (RPM).
The gear reduction ratio of the motor should be considered in the measurements.

Electronic Setup
Microcontroller: Arduino UNO
Motor driver: L298N motor driver, which controls the JGA25-370 motor via PWM.

Power supply:
The motor is powered by an external 12 V DC supply
The Arduino receives power through USB from the computer

---

## 3. Control Principle

The inverted pendulum is naturally unstable in the upright position if left alone, it will fall over. To keep it balanced, we use a reaction wheel. By accelerating or braking the wheel, we generate a torque that counteracts the pendulum’s motion and keeps it upright.

A controller continuously reads the pendulum’s angle and angular velocity, and decides how much the motor should accelerate or decelerate the reaction wheel. In this way, the system can maintain balance even if small disturbances occur.

Mathematically, the system can be described by the dynamics of the inverted pendulum with a reaction wheel. Let:

- θ = pendulum angle
- ω = angular velocity of the pendulum
- u = control input

The controller uses feedback from θ and ω to compute the torque command u according to a control law, for example a PID controller:
Kp,Kd,Ki are the proportional, derivative, and integral gains.

---

## 4. Implementation
### 4.1 Arduino Control Software

Main control loop frequency:

- The PID code samples the sensor and computes the motor output every 5 ms (~200 Hz).
- The threshold-based control code reads the sensor every 100 ms.
- The pulse-counter code calculates frequency and RPM every 200 ms.

setup() initializes the sensor, motor outputs, and serial communication.
loop() runs continuously, executing the control logic.

Sensor readings and motor outputs:

- The AS5600 magnetic sensor provides the pendulum angle in degrees.
- The pulse-counter code measures motor RPM from encoder pulses.
- The PID code calculates motor PWM output from angle and angular velocity feedback.
- The threshold-based code sets motor direction and speed based on the angle using simple logic.
- All codes log the key values to the serial monitor (RPM, out, sig_angle_deg).

### 4.2 Python Tools

Reading measurement files:

The Python scripts read data from the Arduino in real time via the serial port and save it to CSV files.
The first script logs rpm, freq, and out values at each sampling step, while the second script logs out and sig_angle_deg.
Each row in the CSV represents one measurement with a timestamp or sequential index.

Generating plots:

The scripts automatically generate plots using matplotlib from the CSV data.
Examples: RPM vs OUT, Frequency vs OUT, RPM vs Time for the first script; OUT vs Angle for the second.
All plots are saved in timestamped folders (graphs_timestamp or visuals_timestamp) for easy organization.

---

## 5. Experiments and Data

Measurement files

- [Step response datas] (.../
                     data_(0,-255).csv
                     data_(0,255).csv

– contains the motor output (out) and measured angle (sig_angle_deg) during a step input to the system.

Stabilization log: codedata_20251209_155822.csv

– records the pendulum angle and motor output while the system is actively balancing.

Plots

Angle over time:


This plot shows how the pendulum angle changes and stabilizes over time under PID control.


Videos
link

---

## 6. Results

Summarize

---

## 7. Project Timeline

aaa

---

## 8. Team and Credits

aa
