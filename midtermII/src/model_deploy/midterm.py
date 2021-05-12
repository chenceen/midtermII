import matplotlib.pyplot as plt
import numpy as np
import serial
import time

n = np.arange(0,10)
y = np.arange(0,1) 

serdev = '/dev/ttyACM0'
s = serial.Serial(serdev)
for x in range(0, 10):
    line=s.readline() # Read an echo string from B_L4S5I_IOT01A terminated with '\n'
    # print line
    y[x] = float(line)

plt.plot(n,y)

plot.xlabel('sequence number')
plot.ylabel('over:1 under:0')
plt.title("If the detected angle is larger than the threashold angle")
plt.show()
s.close()