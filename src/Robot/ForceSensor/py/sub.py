import rospy
import matplotlib.pyplot as plt
from std_msgs.msg import Int16MultiArray
import time
import numpy as np

import threading

# using global variable to record data everytime callback function receive data from topic
# and we setting a sampling time in main thread to fetch from these global variable

lfx = []
lfy = []
lfz = []
lmx = []
lmy = []
lmz = []

rfx = []
rfy = []
rfz = []
rmx = []
rmy = []
rmz = []

def left_callback(data):
    lfx.append(data.data[0])
    lfy.append(data.data[1])
    lfz.append(data.data[2])
    lmx.append(data.data[3])
    lmy.append(data.data[4])
    lmz.append(data.data[5])

def right_callback(data):
    rfx.append(data.data[0])
    rfy.append(data.data[1])
    rfz.append(data.data[2])
    rmx.append(data.data[3])
    rmy.append(data.data[4])
    rmz.append(data.data[5])

def mix_callback(data):
    lfx.append(data.data[0])
    lfy.append(data.data[1])
    lfz.append(data.data[2])
    lmx.append(data.data[3])
    lmy.append(data.data[4])
    lmz.append(data.data[5])
    rfx.append(data.data[6])
    rfy.append(data.data[7])
    rfz.append(data.data[8])
    rmx.append(data.data[9])
    rmy.append(data.data[10])
    rmz.append(data.data[11])

def left_listener():    
    rospy.Subscriber("/dev/ttyUSB4", Int16MultiArray, left_callback)
    rospy.spin()

def right_listener():
    rospy.Subscriber("/dev/ttyUSB5", Int16MultiArray, right_callback)
    rospy.spin()

def mix_listener():
    rospy.Subscriber("DataFetch", Int16MultiArray, mix_callback)
    rospy.spin()

rospy.init_node("aiRobots_Python", anonymous=True)
# left_td = threading.Thread(target=left_listener)
# right_td = threading.Thread(target=right_listener)
# left_td.start()
# right_td.start()
mix_td = threading.Thread(target=mix_listener)
mix_td.start()

print("Start Record")

plt.figure(figsize=(15, 8))

while len(lfx) < 5000:
    plt.subplot(3, 4, 1)    
    plt.plot(lfx, "b")
    plt.subplot(3, 4, 5)    
    plt.plot(lfy, "b")
    plt.subplot(3, 4, 9)    
    plt.plot(lfz, "b")
    plt.subplot(3, 4, 2)    
    plt.plot(lmx, "b")
    plt.subplot(3, 4, 6)    
    plt.plot(lmy, "b")
    plt.subplot(3, 4, 10)    
    plt.plot(lmz, "b")

    plt.subplot(3, 4, 3)    
    plt.plot(rfx, "b")
    plt.subplot(3, 4, 7)    
    plt.plot(rfy, "b")
    plt.subplot(3, 4, 11)    
    plt.plot(rfz, "b")
    plt.subplot(3, 4, 4)    
    plt.plot(rmx, "b")
    plt.subplot(3, 4, 8)    
    plt.plot(rmy, "b")
    plt.subplot(3, 4, 12)    
    plt.plot(rmz, "b")
    plt.pause(0.01)

forward_data = np.array([lfx, lfy, lfz, lmx, lmy, lmz, rfx, rfy, rfz, rmx, rmy, rmz])
np.save("shift_data_2", forward_data)

print("Finish Record")