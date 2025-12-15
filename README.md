# The project
As part of the COIL project, we are working together as an international team. There are six of us: three from Lisbon, Portugal, and three from Amsterdam, and we are participating in this international program as a single team. The goal of the project is to stabilize a flywheel-driven inverted pendulum. During the five-week program, we work in labs and online meetings, tackling challenging assignments each week. Along the way, we develop various skills, including Arduino programming, PID control, and working with sensors.

---

## 1. Github structure
Our github is split in 5 sections. The first section is [code] (https://github.com/Technische-Natuurkunde-HvA/inverted-pendulum-international-team-1/tree/main/code)

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

**Main control loop**

- [PID control](https://github.com/Technische-Natuurkunde-HvA/inverted-pendulum-international-team-1/blob/main/code/Amsterdam/c%2B%2B/PID_control_simple.ino)

This code implements a Single-Loop PID Controller, specifically designed to stabilize and maintain pendulum around its unstable equilibrium point.
The code is suitable for the following purposes:

  High-Frequency Stabilization: The primary function is to keep the pendulum balanced at the preset target position (setpoint, representing the vertical angle). It achieves this by operating at a high sampling frequency of 200 Hz (FREE_RUN_PERIOD_MS = 5$ ms), allowing for quick, continuous error correction necessary for unstable systems.

  Angle-Based Control: It uses the AS5600 magnetic encoder to measure the deviation angle (sig_angle_deg). The PID algorithm, tuned with parameters like Kp​=90, processes this angle error to calculate the necessary motor output power (output).

  Proportional Motor Output: The calculated output determines both the direction (positive/negative sign controls motorPin1 and motorPin2) and the magnitude (PWM signal via analogWrite(enablePin, abs(output))) of the corrective force applied to the pendulum.
    
### 4.2 Python Tools

**Reading measurement files**

The Python scripts read data from the Arduino in real time via the serial port and save it to CSV files.
The first [script](https://github.com/Technische-Natuurkunde-HvA/inverted-pendulum-international-team-1/blob/main/code/Lissabon/python/data.py) logs rpm, freq, and out values at each sampling step, while the second [script](https://github.com/Technische-Natuurkunde-HvA/inverted-pendulum-international-team-1/blob/main/code/Lissabon/python/PIDdata.py)  logs out and sig_angle_deg.
Each row in the CSV represents one measurement with a timestamp or sequential index.

**Generating plots**

The scripts automatically generate plots using matplotlib from the CSV data.
Examples: RPM vs OUT, Frequency vs OUT, RPM vs Time for the first script; OUT vs Angle for the second.
All plots are saved in timestamped folders (graphs_timestamp or visuals_timestamp) for easy organization.

### 4.3 Additional challenge

**Second PID:**
This [2nd PID](https://github.com/Technische-Natuurkunde-HvA/inverted-pendulum-international-team-1/blob/main/code/Amsterdam/c%2B%2B/pid_controler_second_HVA_v1.ino) addresses the challenge of maintaining a vertical (balancing) inverted pendulum by utilizing a Cascade (Two-Stage) PID Controller. This method is necessary because it strategically separates the system's control problems: the outer PID corrects the angle error, while the inner PID ensures the motor precisely tracks the speed commanded by the outer loop, compensating for the motor's inherent lag and friction. This approach achieves faster response, greater accuracy, and significantly more stable control compared to using a single PID loop.

**Upside down**

Furthermore, the latest [program](https://github.com/Technische-Natuurkunde-HvA/inverted-pendulum-international-team-1/blob/main/code/Amsterdam/c%2B%2B/upside_down_V1.ino) also solves the swing-up problem: it uses built-in, simple logic (threshold-based control) to forcibly swing the pendulum up from the bottom position into the range where the Cascade PID can take over the balancing task.

---

## 5. Experiments and Data

Measurement files (csv) are located in:
- [Step response data(-255;255)](https://github.com/Technische-Natuurkunde-HvA/inverted-pendulum-international-team-1/blob/main/code/Lissabon/csv/data_(-255%2C255).csv)
- [Step response data(0;-255)](https://github.com/Technische-Natuurkunde-HvA/inverted-pendulum-international-team-1/blob/main/code/Lissabon/csv/data_(0%2C-255).csv)
- [Step response data(0;255)](https://github.com/Technische-Natuurkunde-HvA/inverted-pendulum-international-team-1/blob/main/code/Lissabon/csv/data_(0%2C255).csv)
- [Angle data](https://github.com/Technische-Natuurkunde-HvA/inverted-pendulum-international-team-1/blob/main/code/Lissabon/csv/codedata_20251209_155822.csv)
  
Curves created from those datas:(-255;255)
  
![freq vs out](https://github.com/Technische-Natuurkunde-HvA/inverted-pendulum-international-team-1/blob/main/visuals/figures/GraphsRPM_PWM/graficos_(-255%2C255)/freq_vs_out.png)
![rpm vs out](https://github.com/Technische-Natuurkunde-HvA/inverted-pendulum-international-team-1/blob/main/visuals/figures/GraphsRPM_PWM/graficos_(-255%2C255)/rpm_vs_out.png)
![rpm vs tempo](https://github.com/Technische-Natuurkunde-HvA/inverted-pendulum-international-team-1/blob/main/visuals/figures/GraphsRPM_PWM/graficos_(-255%2C255)/rpm_vs_tempo.png)

Records the pendulum angle and motor output while the system is actively balancing:

![out vs angle](https://github.com/Technische-Natuurkunde-HvA/inverted-pendulum-international-team-1/blob/main/visuals/figures/out_vs_sig_angle_deg.png)

Time-lapse: Pendulum Stability:
- [Watch the pendulum stability](https://youtube.com/shorts/owcCYEMpBoo?si=FQ8dx7qpwToPkynY)

Pendulum returning to equilibrium after disturbances:
- [Watch the pendulum balancing](https://github.com/Technische-Natuurkunde-HvA/inverted-pendulum-international-team-1/blob/main/visuals/videos/Amsterdam/Video%20van%20WhatsApp%20op%202025-12-05%20om%2016.32.28_f4a7a739.mp4)

---

## 6. Results

**Summary of achievements**

The pendulum has shown significant improvement on both sites. In Lisbon, the wheel is now able to move very quickly from the bumper to the setpoint and remain stable for long periods, solving the issue related to stabilizing it in the center. In Amsterdam, the wheel can start from a low position (-180º relative to the equilibrium position), reach the equilibrium position, and maintain stability, making the system almost fully functional. The main remaining challenge on the Lisbon side is the unreliability of the AS5600 sensor, which occasionally outputs significant errors in angle measurements. Overall, wheel stabilization and setpoint achievement have been successfully accomplished on both sites.

---

## 7. Project Timeline

**Week 2** 

We successfully executed the motor control testing cycle, incrementally increasing the PWM value from 0 to 255 while recording five measurements at each 1-second interval. While the primary data collection is complete, the critical next step is to implement the necessary Python code to automatically save these measured values for proper analysis. Furthermore, a key finding from our reflection is that the wheels were printed with different fill percentages; this physical variation in mass and friction necessitates a careful comparison of our results to account for these inconsistencies.
- [Week 2 report](https://github.com/Technische-Natuurkunde-HvA/inverted-pendulum-international-team-1/blob/main/progress/Week%202.md)

**Week 3** 

We implemented the Python code for the automatic processing of measured data and the creation of graphs. As a result, we created three main graphs (PWM-RPM, PWM-FREQ, RPM-Time) and their corresponding CSV files, documenting three different test conditions: PWM increase from 0 to 255, direction change from −255 to 255, and decrease from 0 to −255. As a reflection, for the next week, we will 3D-print a wheel with a larger diameter (200mm) with a completely flat backside to increase stability and eliminate wobbling. Furthermore, planning has begun for the angle-vs-time graphs to visually verify the control system's performance.
- [Week 3 report](https://github.com/Technische-Natuurkunde-HvA/inverted-pendulum-international-team-1/blob/main/progress/Week%203.md)

**Week 4** 

The main focus this week was on fine-tuning the PID controller codes. Regarding results: the Amsterdam team successfully achieved stabilization of the wheel in the middle (upright) position, which was captured on video. On the Lisbon side, the wheel already stabilizes with ease when starting from the bottom, but maintaining the middle position still requires work. As a reflection and next step, the Amsterdam team will focus on making the wheel transition from the bottom to the middle, while the Lisbon team will attempt to achieve stable central positioning by modifying the PID controls. Furthermore, work will begin on creating the graphs and collecting the data for PWM-vs-angle under the influence of the PID algorithm.
- [Week 4 report](https://github.com/Technische-Natuurkunde-HvA/inverted-pendulum-international-team-1/blob/main/progress/Week%204.md)

**Week 5** 

This week, as a result, the Lisbon team successfully solved the issue of stabilizing and maintaining the wheel in the center. The wheel is now able to rapidly move from rest to the setpoint and remain stable for long periods, as documented in the uploaded timelapse video. On the Amsterdam side, the wheel is now capable of starting from the upside-down position and quickly reaching the top setpoint. As a reflection, the main ongoing challenge for the Lisbon side is the lack of reliability of the AS5600 sensor, which occasionally outputs angle measurements with significant errors.
- [Week 5 report](https://github.com/Technische-Natuurkunde-HvA/inverted-pendulum-international-team-1/blob/main/progress/Week%205.md)

---

## 8. Team and Credits

Our project team is composed of two groups: the Portuguese team and the Amsterdam team.

Portuguese Team:
- Artur Matos – Physics Engineering student, ISEL
- Tiago Bernardo – Physics Engineering student, ISEL
- Hollo Milan – Erasmus student, Mechanical Engineering, BME

Amsterdam Team:
- Casper van Schaardenburg – Student, Amsterdam University of Applied Sciences
- Silvan van der Leij – Student, Amsterdam University of Applied Sciences
- Ashley Marsman – Student, Amsterdam University of Applied Sciences

Collaborating Institutions:
- ISEL - Instituto Superior de Engenharia de Lisboa
- Amsterdam University of Applied Sciences

---

## 9. Repository
Project repository:

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
