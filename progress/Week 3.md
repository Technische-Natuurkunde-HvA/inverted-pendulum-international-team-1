# Week 3

## 1. Progress description
We both made separate output-responses for our motors and combined these. We made this into one graph: 

## 2. Code

Arduino code to produce variable pwm values for testing
```
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
```
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
import serial
import csv
import time
from datetime import datetime
import matplotlib.pyplot as plt
import pandas as pd
import os
import threading

# -------------------------------
# Configuration
# -------------------------------
PORTA = "COM3"
BAUD = 9600

# --- unique CSV ---
timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
filename_csv = f"dados_{timestamp}.csv"
print("A guardar dados em:", filename_csv)

# --- creates folder for the graphs ---
base_dir = os.path.dirname(os.path.abspath(__file__))
folder_graficos = os.path.join(base_dir, f"graficos_{timestamp}")
os.makedirs(folder_graficos, exist_ok=True)

# --- Opens serial ---
ser = serial.Serial(PORTA, BAUD, timeout=1)
time.sleep(2)  # espera Arduino iniciar

# --- creates CSV with header ---
with open(filename_csv, "w", newline='') as f:
    writer = csv.writer(f)
    writer.writerow(["tempo","rpm","freq","out"])

start_time = time.time()
print("A ler dados do Arduino (pressiona Enter para terminar)...")

# --- Flag to stop loop ---
stop_flag = False

# --- Thread to detect Enter ---
def wait_for_enter():
    global stop_flag
    input("\nPressiona Enter para finalizar o programa...\n")
    stop_flag = True

threading.Thread(target=wait_for_enter, daemon=True).start()

try:
    while not stop_flag:
        linha = ser.readline().decode(errors='ignore').strip()
        if not linha:
            continue

        # --- Parser of line ---
        try:
            partes = linha.split(";")
            rpm  = float(partes[0].split(":")[1])
            freq = float(partes[1].split(":")[1])
            out  = float(partes[2].split(":")[1])
        except:
            print("Linha ignorada:", linha)
            continue

        tempo = time.time() - start_time

        # --- writes immediately CSV ---
        with open(filename_csv, "a", newline='') as f:
            writer = csv.writer(f)
            writer.writerow([tempo, rpm, freq, out])

        # --- Shows on shell ---
        print(f"{tempo:.2f}s  RPM={rpm}  FREQ={freq}  OUT={out}")

finally:
    ser.close()
    print("Serial Port closed.")

    # -------------------------------
    # reads CSV and generates graphs
    # -------------------------------
    df = pd.read_csv(filename_csv)

    # --- 1) RPM vs OUT ---
    plt.figure()
    plt.plot(df["out"], df["rpm"], "o-", color="blue")
    plt.xlabel("OUT")
    plt.ylabel("RPM")
    plt.title("RPM vs OUT")
    plt.grid(True)
    path_fig1 = os.path.join(folder_graficos, "rpm_vs_out.png")
    plt.savefig(path_fig1)
    print("Gráfico salvo:", path_fig1)

    # --- 2) FREQ vs OUT ---
    plt.figure()
    plt.plot(df["out"], df["freq"], "o-", color="green")
    plt.xlabel("OUT")
    plt.ylabel("Frequência (Hz)")
    plt.title("Frequência vs OUT")
    plt.grid(True)
    path_fig2 = os.path.join(folder_graficos, "freq_vs_out.png")
    plt.savefig(path_fig2)
    print("Gráfico salvo:", path_fig2)

    # --- 3) RPM vs TEMPO ---
    plt.figure()
    plt.plot(df["tempo"], df["rpm"], "-", color="red")
    plt.xlabel("Tempo (s)")
    plt.ylabel("RPM")
    plt.title("RPM vs Tempo")
    plt.grid(True)
    path_fig3 = os.path.join(folder_graficos, "rpm_vs_tempo.png")
    plt.savefig(path_fig3)
    print("Gráfico salvo:", path_fig3)

    plt.show()
    print("Fim do programa.")

```
## 3. Results

We've made 3 graphs (PMW-RPM; PMW-FREQ; RPM-Time) and 3 csv files in 3 distinct conditions, first one we had the pmw value go from 0 to 255 with an increment of 2 every 500ms, then go from -255 to 255 and the last one from 0 to -255 as is shown in the code above.

## 4. Reflection 
..
