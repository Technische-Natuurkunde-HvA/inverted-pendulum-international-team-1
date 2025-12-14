#include <Wire.h>
#include <PID_v1.h>
#include <AS5600.h>

AS5600  as5600;  //create sensor object

unsigned long currentMs;  //current time variable
unsigned long lastMs;     // time of last measurement
const unsigned int FREE_RUN_PERIOD_MS = 5; //sampling period in milliseconds
double sig_angle_deg;  // angle measurement
double tot_hoek = (311.61 -260.80 );
// Motor control pins
const int motorPin1 = 10; // IN1
const int motorPin2 = 11; // IN2
const int enablePin = 9; // ENA (PWM pin for speed control)



double output = 0;
double output_pid=0;
int pwm =0;
const int deadzone = 40;  // motor begint pas boven 40


double setpoint_pendulum = 0 ; // Desired angle (vertical position)

double Setpoint_flyweel = 0;    // setpoint afkomstig van hoek-PID (deg/s)
double measuredSpeed = 0;    // gemeten snelheid (deg/s), afgeleid uit hoekverandering
double prevAngle = 0;
double dAngle=0;
unsigned long prevTimeMs = 0;
unsigned long dtMs =0;
// PID parameters (hoek)
double Kp = 40; //40
double Ki = 8; //8
double Kd = 0.03;//0.01
// PID object: buitenlus (hoek) geeft nu speedSetpoint als output
PID pendulumPID(&sig_angle_deg, &Setpoint_flyweel, &setpoint_pendulum, Kp, Ki, Kd, DIRECT);

// PID parameters (snelheid) - tweede PID
double Kp2 = 1;
double Ki2 = 0.9;
double Kd2 = 0.001;
// PID object: binnenlus (snelheid) geeft PWM-output
PID flyweelPID(&measuredSpeed, &output_pid, &Setpoint_flyweel, Kp2, Ki2, Kd2, DIRECT);

void readAndPrintAngle();

void setup() {
  // Set motor control pins as outputs
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  pinMode(enablePin, OUTPUT);

  Wire.begin();     // Initialize I2C
  as5600.begin();   // Initialize sensor
  lastMs= millis();   // Initialize timing
  prevTimeMs = lastMs;
  Serial.begin(9600);  // Initialize Serial Monitor
  delay(2000);
  Serial.print("Test: ");
  Serial.println();

  // Initialize PID controllers
  pendulumPID.SetMode(AUTOMATIC);
  pendulumPID.SetSampleTime(FREE_RUN_PERIOD_MS); // Set sample time in milliseconds
  pendulumPID.SetOutputLimits(-255+deadzone, 255-deadzone);
  flyweelPID.SetMode(AUTOMATIC);
  flyweelPID.SetSampleTime(FREE_RUN_PERIOD_MS);
  flyweelPID.SetOutputLimits(-255+deadzone, 255-deadzone);
  setpoint_pendulum=(0);
  prevAngle = ((float)as5600.readAngle()*0.0879);
}

void loop() {
  // Read and print the angle from AS5600 at the sampling frequency
  currentMs = millis();
  if (currentMs - lastMs >= FREE_RUN_PERIOD_MS) {// periodic sampling
    readAndPrintAngle(); // update sig_angle_deg and lastMs
    dtMs = currentMs - prevTimeMs;
    if (dtMs == 0) dtMs = 1;
    dAngle = sig_angle_deg - prevAngle;
    if (dAngle > 180) dAngle -= 360;
    else if (dAngle < -180) dAngle += 360;
    measuredSpeed = (dAngle) * (1000.0 / dtMs); // deg/s

    prevAngle = sig_angle_deg;
    prevTimeMs = currentMs;


    pendulumPID.Compute();


    flyweelPID.Compute();

     pwm = (int)output_pid;

    if (pwm > 0) {
      output = pwm + deadzone;      // omhoog duwen
      if (pwm > 255) output = 255;
    }
    else if (pwm < 0) {
      output = pwm - deadzone;      // omlaag duwen
      if (output < -255) output = -255;
    }
    // Set motor direction based on PID output
    if (output > 0) {
      digitalWrite(motorPin1, LOW);
      digitalWrite(motorPin2, HIGH);
     
    } else {
      digitalWrite(motorPin1, HIGH);
      digitalWrite(motorPin2, LOW);
     
    }
    if (output < -255) output = -255;
    if (output > 255) output = 255;
    analogWrite(enablePin, abs(output));

    // Print the angle and debug info to the Serial Monitor
    Serial.print(sig_angle_deg);
    Serial.print(" ");
    Serial.print("spdSP:");
    Serial.print(Setpoint_flyweel);
    Serial.print(" spdMeas:");
    Serial.print(measuredSpeed);
    Serial.print(" pwm:");
    Serial.println(output);
 
  }
}

void readAndPrintAngle() {
      lastMs = currentMs;
      sig_angle_deg = ((float)as5600.readAngle()*0.0879)-100-190;
}
