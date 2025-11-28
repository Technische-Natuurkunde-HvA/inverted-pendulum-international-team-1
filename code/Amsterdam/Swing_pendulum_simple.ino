#include <Wire.h>
#include <AS5600.h>
AS5600  as5600;  //create sensor object
unsigned long currentMs;  //current time variable
unsigned long lastMs;     // time of last measurement
const unsigned int FREE_RUN_PERIOD_MS = 100; //sampling period
float sig_angle_deg;  // angle measurement

// Motor control pins
const int motorPin1 = 10; // IN1
const int motorPin2 = 11; // IN2
const int enablePin = 9; // ENA (PWM pin for speed control)

double output; 

void readAndPrintAngle();

void setup() {
  // Set motor control pins as outputs
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  pinMode(enablePin, OUTPUT);

  digitalWrite(motorPin1, LOW);
  digitalWrite(motorPin2, HIGH);

  output = 0 ; 
  
  Wire.begin();     // Initialize I2C
  as5600.begin();   // Initialize sensor
  lastMs= millis();   // Initialize timing
  Serial.begin(9600);  // Initialize Serial Monitor
  delay(2000);
  Serial.print("Test: ");
  Serial.println();

}

void loop() {
  currentMs = millis();
  if (currentMs - lastMs >= FREE_RUN_PERIOD_MS) {// periodic sampling
    readAndPrintAngle();
  }
  
  if (sig_angle_deg < 6 && sig_angle_deg > -6 ){
    output = 0 ;
    delay(000);
  }
  else if (sig_angle_deg > 6 && output > -255){
    output =-250 ;
    delay(500);
  }
  else if (sig_angle_deg < 6 && sig_angle_deg > -25.84 && output > -255){
    output = 250 ;
    delay(500);
  }

  else if (sig_angle_deg < 0){
    output = 0 ;
  }
    if (output > 0) {
    digitalWrite(motorPin1, LOW);
    digitalWrite(motorPin2, HIGH);
  } else {
    digitalWrite(motorPin1, HIGH);
    digitalWrite(motorPin2, LOW);
  }
  analogWrite(enablePin, abs(output));
}

void readAndPrintAngle() {
      lastMs = currentMs;
      sig_angle_deg = ((float)as5600.readAngle()*0.0879)-144.07+3.08-1.76+0.53-5; //0.0879=360/4096;  // degrees [0..360)
      Serial.print(sig_angle_deg); Serial.print('\t');
      Serial.print(output);
      Serial.println();    
}