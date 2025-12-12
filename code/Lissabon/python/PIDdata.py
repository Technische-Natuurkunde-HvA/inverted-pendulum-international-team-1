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
PORT = "COM3"
BAUD = 9600

# --- Unique CSV filename ---
timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
csv_filename = f"data_{timestamp}.csv"
print("Saving data to:", csv_filename)

# --- Folder for graphs ---
# Thonny FIX: __file__ does not exist → use getcwd()
base_dir = os.getcwd()
graphs_folder = os.path.join(base_dir, f"graphs_{timestamp}")
os.makedirs(graphs_folder, exist_ok=True)

# --- Open Serial port ---
ser = serial.Serial(PORT, BAUD, timeout=1)
time.sleep(2)

# --- Create CSV file with header ---
with open(csv_filename, "w", newline='') as f:
    writer = csv.writer(f)
    writer.writerow(["out", "sig_angle_deg"])

print("Reading data from Arduino (press Enter to stop)...")

# --- Stop flag ---
stop_flag = False

def wait_for_enter():
    global stop_flag
    input("\nPress Enter to stop...\n")
    stop_flag = True

threading.Thread(target=wait_for_enter, daemon=True).start()

try:
    while not stop_flag:
        line = ser.readline().decode(errors='ignore').strip()
        if not line:
            continue

        try:
            parts = line.split(";")
            out = float(parts[0].split(":")[1])
            sig_angle_deg = float(parts[1].split(":")[1])
        except Exception:
            print("Invalid line ignored:", line)
            continue

        with open(csv_filename, "a", newline='') as f:
            csv.writer(f).writerow([out, sig_angle_deg])

        print(f"OUT = {out} | Angle = {sig_angle_deg}")

finally:
    ser.close()
    print("Serial port closed.")

    df = pd.read_csv(csv_filename)

    plt.figure()
    plt.plot(df["out"], df["sig_angle_deg"], "o-", color="blue")
    plt.xlabel("OUT(pwm)")
    plt.ylabel("Angle(°)")
    plt.title("OUT - Angle")
    plt.grid(True)

    graph_path = os.path.join(graphs_folder, "out_vs_sig_angle_deg.png")
    plt.savefig(graph_path)

    print("Graph saved to:", graph_path)
    plt.show()

    print("Program finished.")
