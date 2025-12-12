# Week 5

## 1. Progress description


## 2. Code
Lisbon Code for PID control
```
#include <Wire.h>
#include <PID_v1.h>
#include <AS5600.h>

AS5600  as5600;  //create sensor object

unsigned long currentMs;  //current time variable
unsigned long lastMs;     // time of last measurement
const unsigned int FREE_RUN_PERIOD_MS = 5; //sampling period in milliseconds
double sig_angle_deg = 0;  // angle measurement

// Motor control pins
const int motorPin1 = 10; // IN1
const int motorPin2 = 11; // IN2
const int enablePin = 9; // ENA (PWM pin for speed control)

double setpoint = 235.66; // Desired angle (vertical position)
double out = 0;

// // PID parameters
  double Kp = 70;
  double Ki = 50;
  double Kd = 0.6;
  PID myPID(&sig_angle_deg, &out, &setpoint, Kp, Ki, Kd, DIRECT);
  double error = 5.0;                                                 ;

void readAndPrintAngle();

void setup() {
  // Set motor control pins as outputs
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  pinMode(enablePin, OUTPUT);

  Wire.begin();     // Initialize I2C
  as5600.begin();   // Initialize sensor
  lastMs= millis();   // Initialize timing
  Serial.begin(9600);  // Initialize Serial Monitor
  delay(2000);
  Serial.print("Test: ");
  Serial.println();

  // Initialize PID controller
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(FREE_RUN_PERIOD_MS); // Set sample time in milliseconds
  myPID.SetOutputLimits(-255,255); // YOU CAN ADJUST THESE OUTPUT LIMITS IF YOU WISH
}

void loop() {
  // Read and print the angle from AS5600 at the sampling frequency
  currentMs = millis();
  if (currentMs - lastMs >= FREE_RUN_PERIOD_MS) {// periodic sampling

    readAndPrintAngle();

    myPID.Compute(); // Calculate PID output

    // Set motor direction based on PID output
    if (out < 0) {
      digitalWrite(motorPin1, LOW);
      digitalWrite(motorPin2, HIGH);
      
    } else {
      digitalWrite(motorPin1, HIGH);
      digitalWrite(motorPin2, LOW);
      
    }

    // if(sig_angle_deg>=setpoint-error && sig_angle_deg<=setpoint+error){
    //   //stops the wheel with error degree range from the setpoint
    //   if (currentMs - lastMs >= 200) { 
    //   deathzone();
    //   analogWrite(enablePin, 0.0);
    //   logger();
    //   Serial.print("STABLE zone");
    //   }
    //   else
    //   {
    //   deathzone();
    //   analogWrite(enablePin, abs(out));
    //   logger();
    //   }
    // }

      deathzone();
      analogWrite(enablePin, abs(out));
      logger();
    
 
  }
}

void readAndPrintAngle() {
      lastMs = currentMs;
      sig_angle_deg = (float)as5600.readAngle()*0.0879; //0.0879=360/4096;  // degrees [0..360) 
}

void deathzone(){
  if (out > -50 && out < 50) {
    if (out >= 0){ 
    out = 50;}
    else{
    out = -50;}
  }
}

void logger(){
  
      Serial.print("out:");
      Serial.print(out);
      Serial.print(";");
      Serial.print("sig_angle_deg:");
      Serial.println(sig_angle_deg);

}
```
## 3. Results
This week, on the Lisbon side, we solved the issue related to stabilizing the wheel in the center and maintaining its position. The wheel is now able to move from rest to the setpoint very quickly and remain stable for long periods of time, as shown in the timelapse video.

On the Amsterdam side, the wheel is now capable of starting from an upside-down position and rapidly reaching the top setpoint, which can also be seen in the video uploaded this week.

## 4. Reflection 
On the Lisbon side, the main issue we are currently facing is the lack of reliability of the AS5600 sensor, as it occasionally outputs angle measurements with significant errors.
