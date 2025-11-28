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

