#include <Wire.h>
#include <PID_v1.h>
#include <AS5600.h>

AS5600  as5600;  //create sensor object

unsigned long currentMs;  //current time variable
unsigned long lastMs;     // time of last measurement
const unsigned int FREE_RUN_PERIOD_MS = 5; //sampling period in milliseconds
double sig_angle_deg;  // angle measurement
double tot_hoek = (314.15-263.26);
// Motor control pins
const int motorPin1 = 10; // IN1
const int motorPin2 = 11; // IN2
const int enablePin = 9; // ENA (PWM pin for speed control)


double setpoint = 0 ; // Desired angle (vertical position)
double output = 0;
const int deadzone = 10;  // motor begint pas boven 40

// --- Nieuwe variabelen voor tweede PID (snelheid) ---
double speedSetpoint = 0;    // setpoint afkomstig van hoek-PID (deg/s)
double measuredSpeed = 0;    // gemeten snelheid (deg/s), afgeleid uit hoekverandering
double prevAngle = 0;
unsigned long prevTimeMs = 0;

// PID parameters (hoek)
double Kp = 40; //40
double Ki = 8; //8
double Kd = 0.03;//0.01
// PID object: buitenlus (hoek) geeft nu speedSetpoint als output
PID myPID(&sig_angle_deg, &speedSetpoint, &setpoint, Kp, Ki, Kd, DIRECT);

// PID parameters (snelheid) - tweede PID
double Kp2 = 1;
double Ki2 = 0.5;
double Kd2 = 0.001;
// PID object: binnenlus (snelheid) geeft PWM-output
PID speedPID(&measuredSpeed, &output, &speedSetpoint, Kp2, Ki2, Kd2, DIRECT);

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
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(FREE_RUN_PERIOD_MS); // Set sample time in milliseconds
  // hoek-PID output is snelheid (deg/s) -> beperk naar redelijke snelheid
  myPID.SetOutputLimits(-200, 200);

  speedPID.SetMode(AUTOMATIC);
  speedPID.SetSampleTime(FREE_RUN_PERIOD_MS);
  // snelheid-PID output is PWM
  speedPID.SetOutputLimits(-255, 255);


  setpoint=((float)as5600.readAngle()*0.0879)-(tot_hoek/2);

  // init prevAngle
  prevAngle = ((float)as5600.readAngle()*0.0879);
}

void loop() {
  // Read and print the angle from AS5600 at the sampling frequency
  currentMs = millis();
  if (currentMs - lastMs >= FREE_RUN_PERIOD_MS) {// periodic sampling

    readAndPrintAngle(); // update sig_angle_deg and lastMs

    // Bereken gemeten snelheid (deg/s) op basis van hoekverandering en tijd
    unsigned long dtMs = currentMs - prevTimeMs;
    if (dtMs == 0) dtMs = 1;
    double dAngle = sig_angle_deg - prevAngle;
    // eenvoudige unwrap indien nodig (optioneel, afhankelijk van sensor)
    if (dAngle > 180) dAngle -= 360;
    else if (dAngle < -180) dAngle += 360;
    measuredSpeed = (dAngle) * (1000.0 / dtMs); // deg/s

    prevAngle = sig_angle_deg;
    prevTimeMs = currentMs;

    // Buitenlus: hoek PID berekent gewenste snelheid (speedSetpoint)
    myPID.Compute();

    // Binnenlus: snelheid PID berekent PWM-output
    speedPID.Compute();

    int pwm = (int)output;

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
    Serial.print(speedSetpoint);
    Serial.print(" spdMeas:");
    Serial.print(measuredSpeed);
    Serial.print(" pwm:");
    Serial.println(output);
 
  }
}

void readAndPrintAngle() {
      lastMs = currentMs;
      sig_angle_deg = ((float)as5600.readAngle()*0.0879);
}
