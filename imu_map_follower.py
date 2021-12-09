#!/usr/bin/env python
# Ensuring script is executed as a python script

#
# OVERVIEW OF THE FUNCTIONS USED (detailed comments are presented with the code)
# ------------------------------------------------------------------------------
# 1. amclCallback() function updates the value of the current pose (x,y,z,w) of the car as and when the
# /amcl topic receives a update. The function also converts the quaternion to Euler angles that will be used
# further for following the map in the map_follow() function.
#
# 2. imuCallback() - is a callback function of the map_follow node and subscribes to the '/ser_READ' topic. The function primarily performs
# necesary mathematical computations required to compute the real-time position of the car between two amcl-updates. Working at the set imu-sampling
# frequency 50Hz, the function implements a moving-average-filter on the raw-imu data and normalizes the sampled acceleration data. Further, the function computes the
# x and y position by double integrating the normalized acceleration values. Lastly, the function transforms the x-y positions computed from IMU-frame to the MAP-frame
# such that it can be used for correction by the PD algorithm implemented in the map_follow() function.
#
# 3. map_follow() - firstly initializes the Nodes used in thr program and initializes the serial ports for control
# of gas and steering. Futher the function reads the waypoints from the .txt file and performs a simple PD control for
# steering and gas. PD control for gas is based on error in Euclidian distance between the current location and location to be
# heading. PD control for the steering is based on error in current yaw and heading required to reach the waypoint ahead.
#
# NOTE
# ----
# 1. PD control for both the steering and gas is programmed to run at 150Hz
# 2. A custom launch-file "map_follower_pv.launch" needs to be launched prior to execution of this program; The launch file launches
# a bunch of essential nodes like the URG, Rviz etc.
# 3. Remember to run the SerialNode.py program after running the launch file before running this code.
#
# --------------------------------------- HELP BLOCK -----------------------------------------------------------------------------------

#Import for running ROS node
import rospy

#Import for reading laser scan data
from sensor_msgs.msg import LaserScan

#Imports for reading map data
from nav_msgs.msg import OccupancyGrid, MapMetaData

#Imports for reading amcl data
from geometry_msgs.msg import Pose, Point, Quaternion, PoseWithCovarianceStamped

#Import for math computations
import numpy as np
import math

#Imports for reading imu data
from std_msgs.msg import String

#Import for serial communication with Rally car
import serial

#Import for transformation from Quaternion to Euler Angle
import tf

#Import for displaying waypoints on rviz
from visualization_msgs.msg import Marker,MarkerArray

#Class that executes map following
class map_follower:

    #Constructor method for initializing class attributes or values that are required by callback and PD functions
    def __init__(self):

        #Setting the range values for mapping steering command to be sent to the rally car's serial port
        self.min_steer = -2048
        self.max_steer = 2048

        #Setting the range values for mapping speed/gas command to be sent to the rally car's serial port
        self.min_gas = 0
        self.max_gas = 2048

        #Initializing steering and speed/gas command values
        self.steer_val = 0
        self.gas_val = 0

        #Initializing current and previous errors in steering and speed/gas - inputs to PD controller
        self.errS = 0
        self.errG = 0
        self.perrS = 0
        self.perrG = 0

        #Setting the serial port on rally car to communicate with and the baud rate at which communication will happen
        self.port = '/dev/ttyACM0'
        self.baud = 115200

        #Setting the rate at which PD control loop will run - 150 Hz
        self.pidRate = 150

        #Setting the rate at which waypoints are updated - 10 Hz
        self.wayRate = 10

        #Setting Kp and Kd values for PD control of speed/gas error
        self.kpG = 400
        self.kdG = 430

        #Setting the distance threshold from the next waypoint which once crossed will result in waypoint updation
        self.waypoint_error_limit = 1

        #Setting Kp and Kd values for PD control of steering error
        self.kpS = 0.85
        self.kdS = 4.7

        #Initializing waypoint x and y co-ordinates
        self.px = 0
        self.py = 0

        #Initializing rally car's current x and y co-ordinates
	self.ax = 0
        self.ay = 0

        #Initializing rally car's current yaw angle
        self.ayaw = 0

        #Initializing euclidean distance between next waypoint and rally car's current location
        self.distance = 0

        #Initializing timer for storing time since last waypoint updation
        self.waypoint_timer = 0

        #Initializing placeholder array for waypoint x and y co-ordinates
        self.waypoints_locs = np.zeros((230,2))

        #Initializing waypoint index
        self.index = 0

        #Initializing distance to wall - lidar range value at 0 degrees
        self.dist0 = 0

        #Initializing id for marker object to be published to rviz for visualizing waypoints
        self.marker_id = 0

	#Initializing acceleration values in x and y directions - to be computed with data from imu and amcl
        self.imu_ax = 0
        self.imu_ay = 0

        #Initializing position values in x and y directions - to be computed with data from imu and amcl
        self.imu_px = 0
        self.imu_py = 0

        #Initializing velocity values in x and y directions - to be computed with data from imu and amcl
        self.imu_vx = 0
        self.imu_vy = 0

        #Initializing time interval for integration to determine velocity and position
        self.imu_pt = 0
        self.imu_ct = 0

        #Initializing array and index for reading imu data (acceleration values) and storing values to calculate mean for normalizing
        self.imu_index = 0
        self.imu_val = np.zeros((40,2))

        #Initializing rotation matrix and rotation angle for transforming imu values to map frame
        self.rot = []
        self.imuyaw = 0

        #Initializing a pair of values that stores the current x and y position co-ordinates of the rallycar
        self.imu = np.zeros((2,1))

        #Initializing mean for normalizing imu acceleration values
        self.changes = 0

    #Member function - Callback function for 'map_follow' node to subscribe to 'scan' topic
    def scanCallback(self,msg):
        #Computing distance to wall straight ahead - lidar range value at 0 degrees which is at index 540
        self.dist0 = msg.ranges[540]

    #Member function - Callback function for 'map_follow' node to subscribe to 'ser_READ' topic.
    #It reads the topic to get the rally car's current x and y co-ordinates, normalizes, scales and converts them into map frame
    #Our primary source of position data will be imu as frequency of acl updates is very slow.
    def imuCallback(self,data):
        #Calculating time interval for integration to determine velocity and position
        self.imu_pt = self.imu_ct
        self.imu_ct = rospy.get_time()
        delta = self.imu_ct - self.imu_pt

        #Condition to avoid any delays in reading imu values
        if delta > 0.5:
            delta = 0

        #Calculating mean for normalizing imu acceleration values by taking average of 40 acceleration values (x and y directions)
        if self.imu_index <40:
            self.imu_val[self.imu_index,:] = [float(data.data[1:7]),float(data.data[7:13])]
            self.imu_index = self.imu_index+1
            if self.imu_index == 40:
               self.changes = np.mean(self.imu_val,axis=0)

        else:
            #Reading acceleration value in x direction from serial data and normalising and scaling it
            self.imu_ax = (float(data.data[1:7])-self.changes[0])/32768*9.8
            #Reading acceleration value in y direction from serial data and normalising and scaling it
            self.imu_ay = (float(data.data[7:13])-self.changes[1])/32768*9.8

            #Calculating velocity values in x and y directions using formula v = u + a*t
            self.imu_vx = self.imu_vx + self.imu_ax*delta
            self.imu_vy = self.imu_vy + self.imu_ay*delta
            #Calculating position values in x and y directions using formula s = u*t + 0.5*a*t^2
            self.imu_px = self.imu_vx*delta + 0.5*self.imu_ax*delta*delta
            self.imu_py = self.imu_vy*delta + 0.5*self.imu_ay*delta*delta

            #Calculating transform of imu values from imu frame to map frame
            self.imuyaw = math.atan2(self.imu_py,self.imu_px)
            self.rot = [[math.cos(self.imuyaw),-math.sin(self.imuyaw)],[math.sin(self.imuyaw),math.cos(self.imuyaw)]]
            map_vals = self.rot*np.matrix([[self.imu_px],[self.imu_py]])

            #Storing the current x and y position co-ordinates of the rallycar from imu values
            self.imu = self.imu + map_vals

    #Member function - Callback function for 'map_follow' node to subscribe to 'amcl_pose' topic.
    #It reads the topic to get the rally car's current x and y co-ordinates and yaw angle, while logging the same on to screen and /rosout topic.
    #The frequency of amcl updates is much less than imu updates and hence amcl values are used as and when its available.
    #amcl_pose contains orientation data in the form of a quaternion. Using tf library attributes, it is converted to euler angles - roll, pitch and yaw in order.
    def amclCallback(self,data):
	self.ax = data.pose.pose.position.x
        self.ay = data.pose.pose.position.y
        orient = data.pose.pose.orientation
        orientE = tf.transformations.euler_from_quaternion([orient.x,orient.y,orient.z,orient.w])
        self.ayaw = orientE[2]
        rospy.loginfo([self.ax,self.ay,self.ayaw])

        #Whenever amcl data is received, it overwrites imu values for acceleration and resets values for both velocity and position in x and y axes.

        self.imu[0] = self.ax
        self.imu[1] = self.ay
        self.imu_vx = 0
        self.imu_vy = 0
        self.imu_py = 0
        self.imu_px = 0

    #Member function - Callback function for 'map_follow' node to subscribe to 'map' topic
    def mapCallback(self,data):
	pass

    #Member function - Callback function for 'map_follow' node to subscribe to 'map_metadata' topic
    def mapmetaCallback(self,data):
        pass

    #Member function that executes the PD process
    def map_follow(self):

	#Initializes the ROS node 'map_follow' for the whole process that subscribes to the topics 'scan', 'amcl_pose, 'map' and 'map_metadata' using the respective call back functions.
        rospy.init_node('map_follow',anonymous = True)
        #Subsribing to 'scan' topic - specifies the type of message that will be received (LasersScan) and connects the corresponding callback() function (scanCallback)
        rospy.Subscriber("scan",LaserScan,self.scanCallback)
        #Subsribing to 'amcl_pose' topic - specifies the type of message that will be received (PoseWithCovarianceStamped) and connects the corresponding callback() function (amclCallback)
        rospy.Subscriber("amcl_pose",PoseWithCovarianceStamped,self.amclCallback)
        #Subsribing to 'map' topic - specifies the type of message that will be received (OccupancyGrid) and connects the corresponding callback() function (mapCallback)
        rospy.Subscriber("map",OccupancyGrid,self.mapCallback)
        #Subsribing to 'map_metadata' topic - specifies the type of message that will be received (MapMetaData) and connects the corresponding callback() function (mapmetaCallback)
        rospy.Subscriber("map_metadata",MapMetaData,self.mapmetaCallback)
        #Publishing to 'visualization_marker' topic - specifies the type of message that will be published (Marker) so that rviz can subscribe to this for diaplay
        publisher = rospy.Publisher('visualization_marker', Marker, queue_size = 10)
        #Subsribing to 'ser_READ' topic - specifies the type of message that will be received (String) and connects the corresponding callback() function (imuCallback)
        #The topic is user-created and contains acceleration values read from imu over serial port - in raw format
        rospy.Subscriber("ser_READ",String,self.imuCallback)
        #Publishing to 'ser_WRITE' topic of type String to send steering and gas commands to rally car over serial port
        ser_pub = rospy.Publisher('ser_WRITE',String,queue_size = 100)

        #Creating a rate object to use its method sleep() that will help to loop through the following while loop (PID control loop) at 'self.pidRate' times per second.
        ratepid = rospy.Rate(self.pidRate)

        #Initializing serial port to communicate with Rally Car and closing and opening the port for communication
        ser_port = serial.Serial(self.port, self.baud)
        ser_port.close()
        ser_port.open()

        #Loading waypoints from text file (file contains x and y co-ordinates of waypoints clicked using 'waypoint.py' script) and printing them on screen
        a = np.loadtxt("/home/crl3/ros_ws/src/map_follower/src/seconditr_latest.txt",dtype = float,usecols = (0,1))
        self.waypoints_locs = a
        print('The way points are loaded ')
        print(self.waypoints_locs)

        #Code segment for showing waypts on rviz - constructing attributes of marker object
        car_marker = Marker()
        car_marker.header.frame_id = "/map"
        car_marker.type = car_marker.POINTS
        car_marker.action = car_marker.ADD
        car_marker.lifetime = rospy.Duration(0)
        car_marker.id = self.marker_id
        self.marker_id += 1
        car_marker.pose.orientation.x = 0
        car_marker.pose.orientation.y = 0
        car_marker.pose.orientation.z = 0
        car_marker.pose.orientation.w = 1.0
        car_marker.pose.position.x = 0
        car_marker.pose.position.y = 0
        car_marker.pose.position.z = 0
        car_marker.scale.x = 0.1
        car_marker.scale.y = 0.1
        car_marker.scale.z = 0.1
        car_marker.color.a = 1.0
        car_marker.color.r = 0.0
        car_marker.color.g = 0.0
        car_marker.color.b = 1.0
        car_marker.pose.orientation.w = 1.0
        point = Point()

        #PID control loop definition - continues until program is exited
        while not rospy.is_shutdown():

            start_time = rospy.get_time() #Calculating start time for pid loop
            #Calculating euclidean distance between next waypoint and rally car's current location
            self.distance = math.sqrt(math.pow((self.imu[0] - self.px),2)+math.pow((self.imu[1] - self.py),2))

            #Waypoint is updated only if the car is within a distance of 'self.waypoint_error_limit' from the next waypoint and if it has been '1/self.wayRate' seconds since last update
            if (rospy.get_time()-self.waypoint_timer) > (1/self.wayRate) and (self.distance < self.waypoint_error_limit):

                    self.waypoint_timer = rospy.get_time() #Checking if waypoint updation is happening at desired rate and conditions
                    print ('wapoint rate' + str(self.waypoint_timer)) #Checking if waypoint updation is happening at desired rate and conditions
                    print("Waypt updated")
                    self.px = self.waypoints_locs[self.index][0] #Acquiring x co-ordinate of next waypoint from the array that has been loaded with values from text file
                    self.py = self.waypoints_locs[self.index][1] #Acquiring y co-ordinate of next waypoint from the array that has been loaded with values from text file
                    print('Heading to waypt : '+ str(self.px)+str(self.py))
                    #Adding next waypoint to be publsihed to 'visualization_marker' - fixing positions for marker objects
                    point.x = self.px
                    point.y = self.py
                    car_marker.points.append(point)
                    publisher.publish(car_marker)
                    if self.index < (len(self.waypoints_locs) - 1):self.index = self.index+1 #Waypoint updation

            #Computing heading angle from current location to next waypoint that gives the desired steering angle
            alpha = math.atan2((self.py-self.imu[1]), (self.px - self.imu[0]))
            #Computing error between the current yaw angle and heading angle which gives steering error
            self.errS = alpha - self.ayaw

            #Limiting this error to the range -pi to pi so that we get the right steering command values in all cartesian quadrants. This error is the input to the PD process for steering.
            if self.errS>math.pi:
            	self.errS = -2*math.pi+self.errS
            if self.errS<-math.pi:
            	self.errS = 2*math.pi+self.errS

            #Computing PD output for steering = Kp*current error for steering + Kd*(current error for steering - previous error for steering)
            self.steer_val = (self.kpS*self.errS) + (self.kdS*(self.errS - self.perrS))
            #Setting the current steering error as previous steering error for next iteration of PD
            self.perrS = self.errS

            #Input error to PID for gas/speed is the Euclidean distance calculated above
            self.errG = self.distance
            #Computing PD output for gas/speed = Kp*current error for gas/speed + Kd*(current error for gas/speed - previous error for gas/speed)
            self.gas_val = (self.kpG*self.errG) + (self.kdG *(self.errG - self.perrG))
            #Setting the current gas/speed error as previous gas/speed error for next iteration of PD
            self.perrG = self.errG

            #Scale PD steering output values to lie in the range of -60 degrees to 60 degrees, the permissible limits for steering in Ackermann steering mode
            factor_min = -self.kpS*math.pi/3
            factor_max = self.kpS*math.pi/3

            #Mapping steering and gas/speed command values to the right range for Ackermann steering. Limits are initialized above
            self.steer_val = np.interp(self.steer_val,[factor_min,factor_max],[self.max_steer,self.min_steer])
            self.gas_val = np.interp(self.gas_val,[self.min_gas,self.max_gas],[self.min_gas,self.max_gas])

            #Condition to check overspeeding during turnings - turnings are detected by checking if there's a wall right in front of the car
            #Condition is applied only after car has obtained some speed and traversed some distance
            if self.dist0<5 and self.index>4:
            	self.gas_val = 162

            #Constructing the string command to be sent to the serial port consisting of mapped steer and gas values
            ser_val = "A%+05d%+05d" %(self.steer_val, self.gas_val)
            #Sending the Ackermann steering and gas commands to the serial port on rally car
            ser_pub.publish(ser_val)

            #Sleeping just enough to maintain the desired rate through the while loop, i.e. 150 Hz
            ratepid.sleep()
            end_time = rospy.get_time() #Calculating end time for pid loop
            print('Frequency: '+ str(1/(end_time-start_time)))

        #Checking rospy.is_shutdown flag to see if program has exited.
        #In that case, we set the rally car speed and steering to zero before closing the serial port and stopping any further communication with the port.
        if rospy.is_shutdown():
            ser_pub.publish('A+0000+0000')
            ser_port.close()

if __name__ == '__main__': #Standard python 'main' check
    try:
        test = map_follower() #Creating map_follower object and calling its member function 'map_follow' to execute PID process until the ROS node is shutdown
        test.map_follow()
    except rospy.ROSInterruptException: #thrown by manually (Ctrl-C) or otherwise shutting down the ROS node
        pass
