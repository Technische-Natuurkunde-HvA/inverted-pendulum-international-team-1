# Week 3

## 1. Progress description
We both made separate output-responses for our motors and combined these. We made this into one graph: 

## 2. Code

Arduino code to produce variable pwm values for testing
```
// Motor control pins
const int motorPin1 = 10; // IN1 
const int motorPin2 = 11; // IN2 
const int enablePin = 9;  // ENA (PWM pin for speed control)

// Encoder pin
const int encoderPin = 2; // use just one encoder pin for simplicity

volatile int pulseCount = 0;  // pulse counter
const int pulsesPerRevolution = 11; // pulses per rotation from each encoder wire

unsigned long lastTime = 0;   // store last time measurement
double freq = 0; 
double rpm = 0;        // measured frequency

double out = 0;          // motor output value

int PeriodMS = 500;

int status = 0;

void countPulse();

void setup() {
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  pinMode(enablePin, OUTPUT);

  pinMode(encoderPin, INPUT_PULLUP); // 1 encoder input

  attachInterrupt(digitalPinToInterrupt(encoderPin), countPulse, RISING);

  Serial.begin(9600);
  lastTime = millis();
  
  delay(5000);
}

void loop() {
  
  if(status == 0){

    // Motor direction + speed
    analogWrite(enablePin, abs(out));

  for(int i=out; i>=-255; i--){
  out = i;
  if (out > 0) {
    digitalWrite(motorPin1, LOW);
    digitalWrite(motorPin2, HIGH);
  } else {
    digitalWrite(motorPin1, HIGH);
    digitalWrite(motorPin2, LOW);
  }
  analogWrite(enablePin, abs(out));

  // Frequency calculation every 0.2 seconds
  if (millis() - lastTime >= 200) {
    noInterrupts();
    int count = pulseCount;
    pulseCount = 0;
    interrupts();

    freq = count / (pulsesPerRevolution*0.5); // frequency in Hz
    rpm = freq * 60.0 / 9.6;

    Serial.print("RPM:");
    Serial.print(rpm);
    Serial.print(";FREQ:");
    Serial.print(freq);
    Serial.print(";OUT:");
    Serial.println(out);

    lastTime = millis();
  }
    delay(PeriodMS);
    i--;
  }
  }
  status = 1;
  analogWrite(enablePin, abs(0));

}

void countPulse() {
  pulseCount++;  // increment each pulse
}
```
Python Code to process values and produce graphs
```
```
## 3. Results
...


## 4. Reflection 
..
