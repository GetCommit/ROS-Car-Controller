#!/usr/bin/env python

import rospy
import math
from sensor_msgs.msg import LaserScan
from race.msg import pid_input

import time
import datetime
from math import sqrt

counter = 0
goal = 1000
time_record = []

desired_trajectory = 1
vel = 30

pub = rospy.Publisher('error', pid_input, queue_size=10)

##	Input: 	data: Lidar scan data
##			theta: The angle at which the distance is requried
##	OUTPUT: distance of scan at angle theta
def getRange(data,theta):
# Find the index of the arary that corresponds to angle theta.
# Return the lidar scan value at that index
# Do some error checking for NaN and ubsurd values
## Your code goes here
  distance = data.ranges[theta]
  if math.isnan(distance) or distance > 1.5:
    distance = 50
  return distance

def callback(data):
	"""
  theta = 360;
	a = getRange(data,theta)
	b = getRange(data,0)
	swing = math.radians(theta)

	## Your code goes here

	error = a-b
	if(getRange(data,540)<.75):
		vel = 0
	else:
		vel = 8
  if getRange(data, 360) > 5: # Wide turn
    error = -100
  if getRange(data,360) > 5:
    error = 100
  else:
    b = 50
    c = 50
    for i in range(170, 450, 10):
      b2 = getRange(i)*math.cos(math.radians((i-180)/4))
      if b2<b:
        b = b2
        c = getRange(i)*math.sin(math.radians((i-180)/4))

    error = (b-.5)/c
	"""
        global counter
        global time_record
        global goal
        start_time = time.time()
	#start_time = time.clock()

	total = 0
#	print(data.intensities[180])
	for i in range(180, 500, 10):
    #if(data.intensities[i]<.9):
		total += (getRange(data,i)*math.cos(math.radians((i-180)/4))-.75)/getRange(data,i)*math.sin(math.radians((i-180)/4))
    #else:
    # total = total + (getRange(data,i)*math.cos(math.radians((i-180)/4))-.5-getRange(180)-getRange(900))/getRange(data,i)*math.sin(math.radians((i-180)/4))
	error = total / 36

	if(getRange(data,540)<.75):
		vel = 0
	else:
		vel = 8

	msg = pid_input()
	msg.pid_error = error
	msg.pid_vel = vel
	pub.publish(msg)

        end_time = time.time()
	#end_time = time.clock()        
	time_record.append(end_time - start_time)
        counter = counter + 1
        if counter == goal:
          mean = sum(time_record)/ goal
          differences = [x - mean for x in time_record]
          sq_differences = [d ** 2 for d in differences]
          sd = sqrt(sum(sq_differences)/goal)
          filepath = "/home/nvidia/catkin_ws/time_test/dist_finder_recording.txt"
          f = open(filepath, "a")
          f.write(datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")+'\n')
          f.write('The mean is {}ms\n'.format(mean * 1000))
          f.write('The standard deviation is {}ms\n\n'.format(sd * 1000))
          f.close()
          counter = 0
          time_record[:] = []


if __name__ == '__main__':
	print("Laser node started")
	rospy.init_node('dist_finder',anonymous = True)
	rospy.Subscriber("scan",LaserScan,callback)
	rospy.spin()
