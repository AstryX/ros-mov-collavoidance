#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Thu Oct 17 17:02:05 2019

@author: s1983630, s1555638
"""

#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from sensor_msgs.msg import LaserScan
from time import time
import numpy as np
import math
import random

location = [0,0]
laserData = np.zeros(360) #for each 360 degree from center of the robot
frontalAngleSpread = 35 #degrees x2
rightAngleRadius = [250, 330] #degrees
leftAngleRadius = [30, 110] #degrees
forwardCollisionDistance = 0.35 #metres
rotationCollisionDistance = 0.35 #metres
rotationErrorTime = 0.5 #seconds
rightRotCollRadius = [125, 145] #degrees
leftRotCollRadius = [215, 235] #degrees
backCollisionDistance = 0.25 #metres

#Callback to read lidar data
def laser_scan_callback(data):
    global laserData
    laserData = data.ranges

def read_laser_scan_data():
    rospy.Subscriber('scan', LaserScan, laser_scan_callback)
    
def location_callback(data):
    global location
    location[0] = data.position[0]
    location[1] = data.position[1]

def read_location_data():
    rospy.Subscriber('joint_state',JointState,location_callback)

def move_motor(fwd,ang):
    pub = rospy.Publisher('cmd_vel',Twist,queue_size = 10)
    mc = Twist()
    mc.linear.x = fwd
    mc.angular.z = ang
    pub.publish(mc)
 
#Main function used to issue tasks to the robot
def perform_action(curTime, duration, fspeed, rspeed, distance, angle, curType):
    global laserData
    if curType == "auto":
        #forward/rotleft/rotright/backwards/rotadjustright/rotadjustleft/init/preposition
        mode = "init"
		leftErrorFlagTime = 0
		rightErrorFlagTime = 0
        endTime = curTime + duration
		#Runs for a duration of time
        while time() < endTime:
            try:
                read_laser_scan_data()
				#Init stage is used to wait for the robot to retrieve sensor data so it does not drive blindly
				if mode == "init":
					for angle in laserData:
						#Sensors got data
						if angle != 0:
							mode = "preposition"
							break
				#Preposition stage makes sure that the robot is positioned correctly before engaging into autopilot mode
				#Focused on it being too close to the wall in front of it
				elif mode == "preposition":
					frontalAngles = np.append(laserData[-frontalAngleSpread:], laserData[:frontalAngleSpread], 0)
                    collisionAngles = []
                    for i in range(len(frontalAngles)):
                        degree = frontalAngles[i]
                        if degree != 0:
                            if degree <= forwardCollisionDistance:
                                collisionAngles.append(i)
								break
                    if len(collisionAngles) > 0:
                        move_motor(-fspeed, 0)
					else:
						mode = "forward"
				#Main mode where robot drives forward and looks for collisions
                elif mode == "forward":
                    frontalAngles = np.append(laserData[-frontalAngleSpread:], laserData[:frontalAngleSpread], 0)
                    collisionAngles = []
                    for i in range(len(frontalAngles)):
                        degree = frontalAngles[i]
                        if degree != 0:
                            if degree <= forwardCollisionDistance:
                                collisionAngles.append(i)
								break
                    if len(collisionAngles) > 0:
                        move_motor(0,0)
                        rightAngles = np.array(laserData[rightAngleRadius[0]:rightAngleRadius[1]])
                        leftAngles = np.array(laserData[leftAngleRadius[0]:leftAngleRadius[1]])
                        leftClear = True
                        rightClear = True
                        for left in leftAngles:
                            if left != 0:
                                if left <= rotationCollisionDistance:
                                    leftClear = False
                        for right in rightAngles:
                            if right != 0:
                                if right <= rotationCollisionDistance:
                                    rightClear = False

                        if leftClear == True and rightClear == True:
                            print("Random rotation allocation!")
                            choice = random.randint(1, 2) 
                            if choice == 1:
                                mode = "rotleft"
                            else:
                                mode = "rotright"
                        elif leftClear == True:
                            mode = "rotleft"
                        elif rightClear == True:
                            mode = "rotright"
                        else:
                            mode = "backwards"
                    else:
                        move_motor(fspeed,0)
				#Robot rotates left until there are no collisions in front of it
                elif mode == "rotleft":
					if leftErrorFlagTime != 0:
						if time() > leftErrorFlagTime + rotationErrorTime:
							mode = "forward"
							leftErrorFlagTime = 0
						else:
							move_motor(0, rspeed)
					else:
						frontalAngles = np.append(laserData[-frontalAngleSpread:], laserData[:frontalAngleSpread], 0)
						collisionAngles = []
						for i in range(len(frontalAngles)):
							degree = frontalAngles[i]
							if degree != 0:
								if degree <= forwardCollisionDistance:
									collisionAngles.append(i)
									break
						if len(collisionAngles) > 0:
							backAngles = np.array(laserData[leftRotCollRadius[0]:leftRotCollRadius[1]])
							backCollisionAngles = []
							for angle in backAngles:
								if angle != 0:
									if angle <= backCollisionDistance:
										backCollisionAngles.append(i)
										break
							if len(backCollisionAngles) > 0:
								mode = "rotadjustleft"
							else:
								move_motor(0, rspeed)
						else:
							leftErrorFlagTime = time()
				#Robot rotates right until there are no collisions in front of it
                elif mode == "rotright":
					if rightErrorFlagTime != 0:
						if time() > rightErrorFlagTime + rotationErrorTime:
							mode = "forward"
							rightErrorFlagTime = 0
						else:
							move_motor(0, -rspeed)
					else:
						frontalAngles = np.append(laserData[-frontalAngleSpread:], laserData[:frontalAngleSpread], 0)
						collisionAngles = []
						for i in range(len(frontalAngles)):
							degree = frontalAngles[i]
							if degree != 0:
								if degree <= forwardCollisionDistance:
									collisionAngles.append(i)
									break
						if len(collisionAngles) > 0:
							backAngles = np.array(laserData[rightRotCollRadius[0]:rightRotCollRadius[1]])
							backCollisionAngles = []
							for angle in backAngles:
								if angle != 0:
									if angle <= backCollisionDistance:
										backCollisionAngles.append(i)
										break
							if len(backCollisionAngles) > 0:
								mode = "rotadjustright"
							else:
								move_motor(0, -rspeed)
						else:
							rightErrorFlagTime = time()
				#Adjustment mode for right rotation so the robot does not crash to the wall with the back during rotation
				elif mode == "rotadjustright":
					backAngles = np.array(laserData[rightRotCollRadius[0]:rightRotCollRadius[1]])
					backCollisionAngles = []
					for angle in backAngles:
						if angle != 0:
							if angle <= backCollisionDistance:
								backCollisionAngles.append(i)
								break
					if len(backCollisionAngles) > 0:
						move_motor(fspeed, 0)
					else:
						mode = "rotright"
				#Adjustment mode for left rotation so the robot does not crash to the wall with the back during rotation
				elif mode == "rotadjustleft":
					backAngles = np.array(laserData[leftRotCollRadius[0]:leftRotCollRadius[1]])
					backCollisionAngles = []
					for angle in backAngles:
						if angle != 0:
							if angle <= backCollisionDistance:
								backCollisionAngles.append(i)
								break
					if len(backCollisionAngles) > 0:
						move_motor(fspeed, 0)
					else:
						mode = "rotleft"
				#This mode makes the robot drive backwards if the robot can't rotate left or right
				#It drives this way until it finds a suitable rotational path
                elif mode == "backwards":
					#Check both left and right side for possible rotations
					rightAngles = np.array(laserData[rightAngleRadius[0]:rightAngleRadius[1]])
					leftAngles = np.array(laserData[leftAngleRadius[0]:leftAngleRadius[1]])
					leftClear = True
					rightClear = True
					for left in leftAngles:
						if left != 0:
							if left <= rotationCollisionDistance:
								leftClear = False
					for right in rightAngles:
						if right != 0:
							if right <= rotationCollisionDistance:
								rightClear = False

					if leftClear == True and rightClear == True:
						print("Random rotation allocation!")
						choice = random.randint(1, 2) 
						if choice == 1:
							mode = "rotleft"
						else:
							mode = "rotright"
					elif leftClear == True:
						mode = "rotleft"
					elif rightClear == True:
						mode = "rotright"
					else:
						move_motor(-fspeed, 0)
            except rospy.ROSInterruptException:
                return False
	#Legacy code, not really used at the moment. Refer to autopilot mode above
    elif distance > 0 :
        read_location_data()
        startLocation = location
        locationDiff = 0
        endTime = curTime + duration
        while time() < endTime:
            try:
                move_motor(fspeed, rspeed)
                read_location_data()
                locationDiff = math.sqrt(pow(startLocation[0]-location[0], 2) + pow(startLocation[1]-location[1], 2))
                if locationDiff > distance:
                    return True
                print(locationDiff)
            except rospy.ROSInterruptException:
                return False
    else:
        endTime = curTime + duration
        while time()<endTime:
            try:
                read_laser_scan_data()
                move_motor(fspeed, rspeed)
            except rospy.ROSInterruptException:
                return False
    return True

if __name__ == '__main__':
    rospy.init_node('example_script',anonymous=True)

    actionArray = [#{"duration": 5, "fspeed": 0.5, "rspeed": 0, "distance": 1, "angle": 45, "type": "fixed"},
            {"duration": 40, "fspeed": 0.3, "rspeed": 0.5, "distance": 0, "angle": 0, "type": "auto"}]
 

    for action in actionArray:
        try:
            if not perform_action(time(), action["duration"], action["fspeed"],
                    action["rspeed"], action["distance"], action["angle"], action["type"]):
                print('Something went wrong during execution!')
                break
            read_location_data()
        except rospy.ROSInterruptException:
            pass
    move_motor(0,0)