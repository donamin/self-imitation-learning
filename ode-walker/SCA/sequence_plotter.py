import numpy as np
from numpy import genfromtxt
import matplotlib.pyplot as plt
import time

controls = genfromtxt('controls.txt', delimiter=',')

amount_joints = len(controls[0])

for joint in range(0,amount_joints):
    amount = 99

    fig = plt.plot(controls[-amount:-1,joint])

    plt.axis('off')

    #plt.ion()
    plt.show()
    #time.sleep(2.0);


    plt.close('all')
