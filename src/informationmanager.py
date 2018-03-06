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

from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, PolygonStamped, Point32, Twist, WrenchStamped

from std_msgs.msg import Bool, String
from nav_msgs.msg import OccupancyGrid
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionGoal, MoveBaseActionResult

from tf.transformations import euler_from_quaternion, quaternion_from_euler

from ms1_msgs.msg import PlanMsg, ActionSeq, Humans, Human, Objects, Robots

from sensor_msgs.msg import Image, CameraInfo, PointCloud, JointState

class Status(object):
	def __init__(self):
		print 'status init' 
		self.cloud = PointCloud()
		self.cloud_ext = PointCloud()
		self.image = Image()
		self.image_ext = Image()


class InformationManager(object):
# Constructor of the class
	def __init__(self):
		
		self.status=Status()
		# Publishers
		self.DepthPublisher = rospy.Publisher('/depth_cloud_robotnumberint', PointCloud, queue_size = 100)
		self.CameraPublisher = rospy.Publisher('/image_raw_robotnumberint', Image, queue_size = 100)
		self.DepthExtPublisher = rospy.Publisher('/depth_cloud_robotnumberext', PointCloud, queue_size = 100)
		self.CameraExtPublisher = rospy.Publisher('/image_raw_robotnumberext', Image, queue_size = 100)

	def update_manager(self):
		#self.status.print_data()
		self.SendDepthExt(self.status.cloud_ext)		
		self.SendCameraExt(self.status.image_ext)	
		self.SendDepth(self.status.cloud_ext)
		self.SendCamera(self.status.image)			


	def DepthCallback(self, pointcloud):
		self.status.cloud=pointcloud
		self.update_manager()
	def CameraCallback(self, image):
		self.status.image=image
		self.update_manager()
	def DepthCallbackExt(self, pointcloud_ext):
		self.status.cloud_ext=pointcloud_ext
		self.update_manager()
	def CameraCallbackExt(self, image_ext):
		self.status.image_ext=image_ext
		self.update_manager()

	def SendDepth(self, cloud):
		self.DepthPublisher.publish(cloud)

	def SendDepthExt(self, cloud):
		self.DepthExtPublisher.publish(cloud)

	def SendCamera(self, camera):
		self.CameraPublisher.publish(camera)
	def SendCameraExt(self, camera):
		self.CameraExtPublisher.publish(camera)

	def run_component(self):
		rospy.init_node('informationmanager')
		# Subscribers
		# Local robot
		rospy.Subscriber("depth_cloud", PointCloud, self.DepthCallback)
		rospy.Subscriber("image_raw", Image, self.CameraCallback)
		# External robot
		rospy.Subscriber("depth_cloud_robotnumberext", PointCloud, self.DepthCallbackExt)
		rospy.Subscriber("image_raw_robotnumberext", Image, self.CameraCallbackExt)


		rate = rospy.Rate(10)
		while not rospy.is_shutdown():
			try:
				rate.sleep()
			except rospy.ROSInterruptException:
				pass

	@abstractmethod
	def informationmanager_method(self, depth_cloud, depth_cloud_robotnumber, image_raw, image_raw_robotnumber):
		pass

