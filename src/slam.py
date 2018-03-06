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

from ms1_msgs.msg import PlanMsg, ActionSeq,Humans, Human

from sensor_msgs.msg import Image, CameraInfo, PointCloud

class Status(object):
	def __init__(self):
		print 'status init' 
		self.depth = PointCloud()
		self.camera = Image()
		self.camera_info = CameraInfo()
		self.map = OccupancyGrid()
		self.pose = PoseWithCovarianceStamped()


class Slam(object):
# Constructor of the class
	def __init__(self):
		self.status=Status()
		# Publishers
		self.MapPublisher = rospy.Publisher('map', OccupancyGrid, queue_size = 100)
		self.PosePublisher = rospy.Publisher('robot_pose', PoseWithCovarianceStamped, queue_size = 100)

	def update_manager(self):
		#print 'update' 
		self.SendMap(self.status.map)
		self.SendPose(self.status.pose)


	def DepthCallback(self, pointcloud):
		# point cloud of the environment
		self.status.depth = pointcloud
		self.update_manager()
	def CameraCallback(self, image):
		# stream of images from the camera
		self.status.camera = image
		self.update_manager()
	def CameraInfoCallback(self, camerainfo):
		# meta data regarding the camera
		self.status.camera_info = camerainfo
		#self.update_manager()

	def SendMap(self, map):
		self.MapPublisher.publish(map)
	def SendPose(self, pose):
		self.PosePublisher.publish(pose)


	def run_component(self):
		rospy.init_node('slam')

		# Subscribers
		rospy.Subscriber("depth_cloud", PointCloud, self.DepthCallback)
		rospy.Subscriber("image_raw", Image, self.CameraCallback)
		rospy.Subscriber('camera_info', CameraInfo, self.CameraInfoCallback)

		rate = rospy.Rate(10)
		while not rospy.is_shutdown():
			try:
				rate.sleep()
			except rospy.ROSInterruptException:
				pass

	@abstractmethod
	def slam_method(self, cloud, images):
		pass

	
	
