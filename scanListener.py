#!/usr/bin/env python




import rospy
import numpy
from sensor_msgs.msg import LaserScan
import math


flag = 0
index90 = 0
indexNeg90 = 0

def scanCallback(msg):

	global flag,index90,indexNeg90
	#rospy.loginfo(rospy.get_caller_id() + 'I heard %s', msg.angle_increment)
	len_ranges = len(msg.ranges)
	angle_inc = msg.angle_increment * (180 / 3.14)
	stepsPerDeg = 1/angle_inc
	zeroDegInd = round(len_ranges/2)
	#total_angle = round(len_ranges/stepsPerDeg)
	index90 = round(zeroDegInd + (90*stepsPerDeg))
	indexNeg90 = round(zeroDegInd + (-90*stepsPerDeg))
	
	#print(len(msg.ranges))
	flag = 1
	

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
		
	
		rospy.init_node('scanListener', anonymous=True)
	    	
		rospy.Subscriber('scan',LaserScan, scanCallback)
	
		#rospy.spin() # spin() simply keeps python from exiting until this node is stopped


if __name__ == '__main__':
        listener()
	main_flag = 1 ## to ensure that the print happens only once
	global flag
	while not rospy.is_shutdown():
		
		if flag == 1 & main_flag == 1:
			print('Index for 90 degrees is at : '+str(index90))
			
			print('Index for -90 degrees is at : '+str(indexNeg90))
						
			flag = 0
			main_flag = 0
			
			## print the -90 and +90
		
		
