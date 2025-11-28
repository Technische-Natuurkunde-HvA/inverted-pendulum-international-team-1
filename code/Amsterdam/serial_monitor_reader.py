import serial
import time
import csv
import numpy as np
frequency_max=16.452058823529416  #bij output 255
RPM_max=620
gear_ratio=9.6
print(gear_ratio)
ser = serial.Serial('COM6', 9600, timeout=2)
#i=1
filename = f"ard_sm_outputS_{i}.csv"
RPM_list=[]
output_list=[]
j=0
k=0

    

time.sleep(1)
with open(filename, "a") as file:
    try:
        while True:
            if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8', errors='ignore').rstrip()
                print(line)

                parts = line.split()
                output = int(parts[1])
                output_list.append(output)
                RPM = (float(parts[3]) /gear_ratio)
                RPM_list.append(RPM)
                print(RPM,"RPM")
                

                k+=1
                if k==1:
                    file.write(str(output) + "," + str(RPM) + "\n")
                    print("line toevoegen")
                    
                    k=0


    except KeyboardInterrupt:
        print("conection closed!")
    finally:
        ser.close()
        frequency_mean=np.mean(RPM_list)
        print(frequency_mean)
        i+=1
        print(filename)