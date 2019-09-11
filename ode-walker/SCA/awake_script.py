import sys, string, os
import random
import time

from datetime import datetime



dist = "D:/joose/Dropbox/Dropbox/awake.txt"



file = open(dist, 'w+')

time_string = str(datetime.now())
message = "Finished last job at: " + time_string

file.write(message) 

file.close()

