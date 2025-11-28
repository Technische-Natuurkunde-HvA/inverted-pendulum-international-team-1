# About this document

This is a team document where you record your weekly progress. It is written in Markdown, which allows you to format content in a simple and readable way. The document is rendered directly in GitHub without the need for a compiler (unlike LaTeX). The syntax is relatively easy. An overview of commonly used Markdown syntax can be found here:  
https://www.markdownguide.org/basic-syntax/

Below you find an example of the sections that must be included in each weekly progress report.

Use the **same document for all weeks**. For each week, use the **same headers and subheaders**.

Use the GitHub repository to store important project files (code, visuals including videos and figures, data, etc.). If necessary refer to those files in this document using a hyperlink. 

---

# Week 3

## 1. Progress description
The code increments the pwm value every 1s taking 5 measuremente per value 0-255.

## 2. Code
```c
// Header file for input/output functions
// Motor control pins
const int motorPin1 = 10; // IN1 
const int motorPin2 = 11; // IN2 
const int enablePin = 9;  // ENA (PWM pin for speed control)

// Encoder pin
const int encoderPin = 2; // use just one encoder pin for simplicity

volatile int pulseCount = 0;  // pulse counter
const int pulsesPerRevolution = 11; // pulses per rotation from each encoder wire

unsigned long lastTime = 0;   // store last time measurement
double frequency = 0; 
double RPM;        // measured frequency

double output = 0;          // motor output value

int PeriodMS = 1000;

void countPulse();

void setup() {
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  pinMode(enablePin, OUTPUT);

  pinMode(encoderPin, INPUT_PULLUP); // 1 encoder input

  attachInterrupt(digitalPinToInterrupt(encoderPin), countPulse, RISING);

  Serial.begin(9600);
  lastTime = millis();
}

void loop() {
  

  // Motor direction + speed
  for(int i=0; i <=255; i++){
  output = i;
  if (output > 0) {
    digitalWrite(motorPin1, LOW);
    digitalWrite(motorPin2, HIGH);
  } else {
    digitalWrite(motorPin1, HIGH);
    digitalWrite(motorPin2, LOW);
  }
  analogWrite(enablePin, abs(output));

  // Frequency calculation every 0.2 seconds
  if (millis() - lastTime >= 200) {
    noInterrupts();
    int count = pulseCount;
    pulseCount = 0;
    interrupts();

    frequency = count / (pulsesPerRevolution*0.2); // frequency in Hz
    RPM = frequency * 60 / 9.6;

    //Serial.print("Output: ");
    Serial.print(output);
    Serial.print(",");
    Serial.println(RPM);
    //Serial.println(" RPM");

    lastTime = millis();
  }
    delay(PeriodMS);
    i++;
  }
}

void countPulse() {
  pulseCount++;  // increment each pulse
}
``` 
## 3. Results
Present your results here. This may include tables, figures, or charts.
Add charts and other visuals to the `visuals` folder in the GitHub repository and reference them in this document if needed.


## 4. Reflection 
We still need to implement the python code to save the measured values.
Our wheels where printed in diffente fill percentages so we are going to need to compare our values.
