#!/usr/bin/env python
import rospy
import numpy as np
import tf
import math
import geometry_msgs.msg
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker

class waypoint(object):
	def __init__(self):
		self.path = Marker()
		self.marker_id = 1
		self.index=0
		rospy.init_node('echoer')
		# subscribe to "/move_base_simple/goal" to get picked way points using 2D Nav Goal in rviz
		rospy.Subscriber("/move_base_simple/goal", geometry_msgs.msg.PoseStamped, self.get_way_point)
		# display picked way points and path between way points in rviz
		self.publisher = rospy.Publisher('visualization_marker', Marker, queue_size = 10)
	def display_way_point(self,x,y):
		points = Marker()		
		points.header.frame_id = "/map"	# publish path in map frame		
		points.type = points.POINTS
		points.action = points.ADD
		points.lifetime = rospy.Duration(0)
		points.id = self.marker_id
		self.marker_id += 1
		points.scale.x = 0.1
		points.scale.y = 0.1	
		points.color.a = 1.0
		points.color.r = 0.0
		points.color.g = 0.0
		points.color.b = 1.0
		points.pose.orientation.w = 1.0

		point = Point()
		point.x = x
		point.y = y

		points.points.append(point);
		# Publish the MarkerArray
		self.publisher.publish(points)
if __name__ == '__main__':
