#!/usr/bin/env python
# the class is implemented using a template method pattern.
# each object detection algorithm extends the class detection and provides the logic for performing detecting objects, humans and robots

import numpy
import Queue
import rospy
import sys
import time

from ms1_msgs.msg import PlanMsg, PosePlanArray, MovementPlan, StringStamped

import actionlib
from actionlib import SimpleActionClient
from actionlib_msgs.msg import GoalStatus

from abc import abstractmethod

from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, PolygonStamped, Point32, Twist, WrenchStamped, Polygon

from std_msgs.msg import Bool, String
from nav_msgs.msg import OccupancyGrid
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionGoal, MoveBaseActionResult

from tf.transformations import euler_from_quaternion, quaternion_from_euler

from ms1_msgs.msg import PlanMsg, ActionSeq, Humans, Human, Objects, Robots

from sensor_msgs.msg import Image, CameraInfo, PointCloud, JointState

from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory

class Status(object):
	def __init__(self):
		print 'status init' 
		self.depth = PointCloud()
		self.image = Image()
		self.roi = Polygon()
		self.humans = Humans()
		self.objects = Objects()
		self.robots = Robots()

class DetectionExecutor(object):
	def __init__(self):
		self.status=Status()
		# Publishers
		self.HumansPublisher = rospy.Publisher('tracked_humans', Humans, queue_size = 100)
		self.ObjectsPublisher = rospy.Publisher('tracked_objects', Objects, queue_size = 100)
		self.RobotsPublisher = rospy.Publisher('tracked_robots', Robots, queue_size = 100)

	def update_manager(self):
		#print 'update' 
		self.SendHumans(self.status.humans)
		self.SendObjects(self.status.objects)
		self.SendRobots(self.status.robots)

	def DepthCallback(self, cloud):
		# point cloud of the environment
		self.status.depth=cloud
		#self.update_manager()
	def CameraCallback(self, image):
		# stream of images from the camera
		self.status.image=image
		self.update_manager()
	def ROICallback(self, polygon):
		# area to track
		self.status.roi=polygon
		self.update_manager()

	def SendHumans(self, humans):
		# sends data regarding the detected humans
		self.HumansPublisher.publish(humans)
	def SendObjects(self, objects):
		# sends data regarding the detected objects
		self.ObjectsPublisher.publish(objects)
	def SendRobots(self, robots):
		# sends data regarding the detected robots
		self.RobotsPublisher.publish(robots)


	def run_component(self):
		rospy.init_node('objectdetection')
		# Subscribers
		rospy.Subscriber("depth_cloud", PointCloud, self.DepthCallback)
		rospy.Subscriber("image_raw", Image, self.CameraCallback)
		rospy.Subscriber("perception_focus_area", Polygon, self.ROICallback)

		rate = rospy.Rate(10)
		while not rospy.is_shutdown():
			try:
				rate.sleep()
			except rospy.ROSInterruptException:
				pass

	@abstractmethod
	def detectionexecutor_method(self, depth, image, roi, humans, objects, robots):
		pass





