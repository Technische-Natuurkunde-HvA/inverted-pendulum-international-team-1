# Week 4

## 1. Progress description
Work on the PID Codes

## 2. Code
PID Arduino CODE
```
//for more information on the PID library: http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/

#include <Wire.h>
#include <PID_v1.h>
#include <AS5600.h>

AS5600  as5600;  //create sensor object

unsigned long currentMs;  //current time variable
unsigned long lastMs;     // time of last measurement
const unsigned int FREE_RUN_PERIOD_MS = 5; //sampling period in milliseconds
double sig_angle_deg;  // angle measurement

// Motor control pins
const int motorPin1 = 10; // IN1
const int motorPin2 = 11; // IN2
const int enablePin = 9; // ENA (PWM pin for speed control)


double setpoint = 0 ; // Desired angle (vertical position)
double output = 0;
const int deadzone = 0;  // motor begint pas boven 40


// PID parameters
double Kp = 85; //45
double Ki = 3; //3
double Kd = 0.5;
PID myPID(&sig_angle_deg, &output, &setpoint, Kp, Ki, Kd, DIRECT);

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
  setpoint =((float)as5600.readAngle()*0.0879)-144.07-(23.29-21.80)+4.74-121.65-2.5-8-5.67+3.5-5.96;
  setpoint= setpoint+(35.66+11.52)/2;
  setpoint= 12.93;
}

void loop() {
  // Read and print the angle from AS5600 at the sampling frequency
  currentMs = millis();
  if (currentMs - lastMs >= FREE_RUN_PERIOD_MS) {// periodic sampling

    readAndPrintAngle();

    myPID.Compute(); // Calculate PID output
        int pwm = output;

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

    // Print the angle to the Serial Monitor
    Serial.print(sig_angle_deg);
    Serial.print(" ");
    // Print PID output for debugging
    Serial.println(output);
 
  }
}

void readAndPrintAngle() {
      lastMs = currentMs;
      sig_angle_deg = ((float)as5600.readAngle()*0.0879)-144.07-(23.29-21.80)+4.74-121.65-2.5-8-5.67+3.5-5.96;
}

```
## 3. Results
On the Amsterdam side we made a video and got it to stabilize in the middle. From the Lissabon side we made it so it goes from start to the middle with this PID controller.


## 4. Reflection 
For Amsterdam: Making the wheel go from start to middle. Lissabon: Make it stabilize in the middle, by changing PID Controls. Also working to make the graphs and take the data for the pwm-angle with the influence of the PID algorithm.

