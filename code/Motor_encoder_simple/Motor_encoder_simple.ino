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

int PeriodMS = 200;

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

  // Frequency calculation every 0.5 seconds
  if (millis() - lastTime >= 200) {
    noInterrupts();
    int count = pulseCount;
    pulseCount = 0;
    interrupts();

    frequency = count / (pulsesPerRevolution*0.5); // frequency in Hz
    RPM = frequency * 60 / 9,6;

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