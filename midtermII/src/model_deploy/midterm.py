import matplotlib.pyplot as plt
import paho.mqtt.client as paho
import numpy as np
import serial
import time


mqttc = paho.Client()

serdev = '/dev/ttyACM0'
s = serial.Serial(serdev)

host = "192.168.1.150"
topic = "Mbed"

def on_connect(self, mosq, obj, rc):
    print("Connected rc: " + str(rc))

def on_message(mosq, obj, msg):
    print("[Received] Topic: " + msg.topic + ", Message: " + str(msg.payload) + "\n")

def on_subscribe(mosq, obj, mid, granted_qos):
    print("Subscribed OK")

def on_unsubscribe(mosq, obj, mid, granted_qos):
    print("Unsubscribed OK")

# Set callbacks
mqttc.on_message = on_message
mqttc.on_connect = on_connect
mqttc.on_subscribe = on_subscribe
mqttc.on_unsubscribe = on_unsubscribe

# Connect and subscribe
print("Connecting to " + host + "/" + topic)
mqttc.connect(host, port=1883, keepalive=60)
mqttc.subscribe(topic, 0)

# Publish messages from Python
num = 0
while num != 5:
    ret = mqttc.publish(topic, "Message from Python!\n", qos=0)
    if (ret[0] != 0):
            print("Publish failed")
    mqttc.loop()
    time.sleep(1.5)
    num += 1
data = []
data_new = []
while True:
  try:
    line = s.readline().decode()
    if '---start---' in line:
      print("---start---")
      data_new.clear()
    elif '---stop---' in line:
      print("---stop---")
      if len(data_new) > 0:
        print("Data saved:")
        print(data_new)
        data.append(data_new.copy())
        data_new.clear()
      print("Data Num =", len(data))
    else:
      print(line, end="")
      data_new.append(line)

# Loop forever, receiving messages
mqttc.loop_forever()
n = np.arange(0,10)
y = np.arange(0,1) 


for x in range(0, 10):
    y[x] = data[x]

fig,ax = plt.subplots(2, 1)
fig.suptitle("If the detected angle is larger than the threashold angle")
ax[0].plot(n,y)
ax[0].set_xlabel('sequence number')
ax[0].set_ylabel('over:1 under:0')

plt.show()
s.close()