# -*- coding: utf-8 -*-
"""
Created on Tue Nov 25 14:38:38 2025

@author: caspe
"""

import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import math as math
import pandas as pd
from matplotlib.ticker import MaxNLocator

file = r"C:\Users\caspe\OneDrive\Bureaublad\technise_natuurkunde\project natuurkunde\jaar 2\poject 2.2\python_reader\ard_sm_outputS_10.csv"
RPMs=[]
out=[]


df = pd.read_csv(
    file,
    header=None,
    names=["Output","RPM"],
    decimal=","
)
output=(df["Output"])

print(df)
k=0
for j in output:
    
    print(j)
    RPM=(df["RPM"])
    rpm=(RPM.iloc[k])
    print(rpm)
    k+=1
    RPMs.append(rpm)
    out.append(j)

    
plt.scatter(out,RPMs)

plt.show()