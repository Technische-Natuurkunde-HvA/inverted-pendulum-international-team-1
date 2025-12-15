# The project
As part of the COIL project, we are working together as an international team. There are six of us: three from Lisbon, Portugal, and three from Amsterdam, and we are participating in this international program as a single team. The goal of the project is to stabilize a flywheel-driven inverted pendulum. During the five-week program, we work in labs and online meetings, tackling challenging assignments each week. Along the way, we develop various skills, including Arduino programming, PID control, and working with sensors.

---

## 1. Github structure
Our github is split in 5 sections. The first section is [code](https://github.com/Technische-Natuurkunde-HvA/inverted-pendulum-international-team-1/tree/main/code). This is where everything we made in c++, python and csv. In the code folder we have one folder for everything made on the Amsterdam side and one for the Lisbon side. The next folder is [docs](https://github.com/Technische-Natuurkunde-HvA/inverted-pendulum-international-team-1/tree/main/docs) this is where more of the technical information about the project is found. Next up is [feedback](https://github.com/Technische-Natuurkunde-HvA/inverted-pendulum-international-team-1/tree/main/feedback). This is where supervisors gave feedback on the project, which is divided in 3 weeks. Then comes the folder [progress](https://github.com/Technische-Natuurkunde-HvA/inverted-pendulum-international-team-1/tree/main/progress), here is a documentation of everything we accomplished each week. And the last one; [visuals](https://github.com/Technische-Natuurkunde-HvA/inverted-pendulum-international-team-1/tree/main/visuals). This is where our viaual data is found, where you can find figures (pictures). Besides figures, there is also videos. This tab is split into an Amsterdam side and an Lisbon side.

---

## 2.  System Overview

The experimental setup is a flywheel-driven inverted pendulum consisting of mechanical and electronic components.

**Mechanical Setup:**
- Pendulum arm: 3D-printed arm that freely rotates around its pivot axis.
- Angle sensor: a rotary encoder to measure the arm’s tilt.
- Reaction wheel: a 3D-printed flywheel attached to the end of the arm, which is accelerated or decelerated by the motor to stabilize the pendulum.

- [Picture of the reaction wheel](https://github.com/Technische-Natuurkunde-HvA/inverted-pendulum-international-team-1/blob/main/visuals/videos/Lissabon/Wheel_2.JPEG)

**Drive:**
- DC motor: JGA25-370, 12 V DC motor with an integrated encoder.
- The encoder signals are calculate the wheel’s rotational speed (RPM).
- The gear reduction ratio of the motor should be considered in the measurements.

**Electronic Setup:**
- Microcontroller: Arduino UNO
- Motor driver: L298N motor driver, which controls the JGA25-370 motor via PWM.

**Power supply:**
- The motor is powered by an external 12 V DC supply
- The Arduino receives power through USB from the computer

- [Picture of the setup](https://github.com/Technische-Natuurkunde-HvA/inverted-pendulum-international-team-1/blob/main/visuals/videos/Lissabon/Overview_2.JPEG)

---

## 3. Control Principle

The inverted pendulum is naturally unstable in the upright position if left alone, it will fall over. To keep it balanced, we use a reaction wheel. By accelerating or braking the wheel, we generate a torque that counteracts the pendulum’s motion and keeps it upright.

The codes implement the principles of discrete-time, closed-loop PID (Proportional-Integral-Derivative) control systems, designed for the dynamic positioning of motors or physical systems. 

- [See the codes](https://github.com/Technische-Natuurkunde-HvA/inverted-pendulum-international-team-1/tree/main/code/Amsterdam/c%2B%2B)

### 3.1. The Control Basis (Error Calculation)

The foundation of the control loop is the continuous determination of the **error**, which is the difference between the desired target (Setpoint) and the measured real state (AS5600 angle).

* **SP (Setpoint):** The desired angular position.
* **sig_angle_deg (Measured Angle):** The actual angle read from the AS5600

### 3.2 The PID Control Law

The controller (PID) calculates the **Output Signal (`output`)** as a weighted sum of the three components of the error signal. This signal drives the motor:

$$
u(t) = \underbrace{K_p e(t)}_{\text{Proportional (P)}} + \underbrace{K_i \int_{0}^{t} e(\tau) d\tau}_{\text{Integral (I)}} + \underbrace{K_d \frac{d e(t)}{d t}}_{\text{Derivative (D)}}
$$


* **P-term:** Proportional to the current error. This ensures a fast initial reaction.
* **I-term:** Proportional to the sum (integral) of past errors. This component is responsible for eliminating the steady-state error (offset). 
* **D-term:** Proportional to the rate of change of the error (derivative). This dampens overshoot (oscillation) and stabilizes the system.

### 3.3. Output Limitation (Motor Drive)

The calculated output signal must be limited by the physical constraints of the motor driver, which is the maximum PWM value ($\pm 255$):

Example a PID controller:  
  
![PID](https://github.com/Technische-Natuurkunde-HvA/inverted-pendulum-international-team-1/blob/main/visuals/figures/PID.png)

---

## 4. Implementation
### 4.1 Arduino Control Software

The initial step in project setup involves using the [motor_encoder_simple](https://github.com/Technische-Natuurkunde-HvA/inverted-pendulum-international-team-1/blob/main/code/Lissabon/c%2B%2B/Motor_encoder_simple.ino) file to test the motor and quantify its deadzone. The deadzone is the phenomenon where the motor fails to respond to low input PWM values due to internal friction. Knowing this mechanical limitation is critical for successful PID control.

## 9. Directory tree
```text
inverted-pendulum-international-team-1
├─ code/ # Arduino and Python code AND Measurement files (csv, dat, txt)
├─ feedback/ # Internal team/supervisor comments
├─ progress/ # Weekly markdown reports (raw material)
├─ visuals/ # Photos, plots, diagrams, videos
├─ docs/
│ └─ index.md # Public project documentation (for GitHub Pages)
└─ README.md # Technical repo overview for GitHub users
```
