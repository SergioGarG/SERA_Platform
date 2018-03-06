# the class is implemented using a template method pattern.
#!/usr/bin/env python

#import numpy
import Queue
import rospy
import sys
import time

from ms1_msgs.msg import PlanMsg, PosePlanArray, MovementPlan, StringStamped

import actionlib
from actionlib import SimpleActionClient
from actionlib_msgs.msg import GoalStatus

from abc import abstractmethod

from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, PolygonStamped, Point32

from std_msgs.msg import Bool, String
from nav_msgs.msg import OccupancyGrid
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionGoal, MoveBaseActionResult

from tf.transformations import euler_from_quaternion, quaternion_from_euler

from ms1_msgs.msg import PlanMsg, ActionSeq, Humans, Human

from sensor_msgs.msg import Image, CameraInfo, PointCloud

class Status(object):
	def __init__(self):
		print 'status init' 
		self.depth = PointCloud()
		self.humans = Humans()
		self.camera = Image()
		self.mission = String()

class GesturesRecognition(object):
# Constructor of the class
	def __init__(self):
		self.status = Status()
		# Publishers
		self.MissionPublisher = rospy.Publisher('mission_update', String, queue_size = 100)
		# code to parse the gestures from an human being

	def update_manager(self):
		#print 'update' 
		self.SendMission(self.status.mission)

	def DepthCallback(self, pointcloud):
		# point cloud of the environment
		self.status.depth = pointcloud
	def HumansCallback(self, humans):
		# metadata regarding tracked humans
		self.status.humans = humans
	def CameraCallback(self, image):
		# stream of images from the camera
		self.status.camera = image


	def run_component(self):
		rospy.init_node('gesturesrecognition')
		# Subscribers
		rospy.Subscriber("tracked_humans", Humans, self.HumansCallback)
		rospy.Subscriber("image_raw", Image, self.CameraCallback)
		rospy.Subscriber("depth_cloud", PointCloud, self.DepthCallback)

		rate = rospy.Rate(10)
		while not rospy.is_shutdown():
			try:
				rate.sleep()
			except rospy.ROSInterruptException:
				pass

	@abstractmethod
	def gestures(self, cloud, images, tracked_humans):
		pass





