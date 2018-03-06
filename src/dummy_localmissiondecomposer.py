#!/usr/bin/env python
import roslib
import numpy
import Queue
import rospy
import sys
import time


import actionlib
from actionlib import SimpleActionClient
from actionlib_msgs.msg import GoalStatus


from abc import abstractmethod

from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, PolygonStamped, Point32

from std_msgs.msg import Bool, String
#from roseus.msg import StringStamped
from nav_msgs.msg import OccupancyGrid
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionGoal, MoveBaseActionResult

from tf.transformations import euler_from_quaternion, quaternion_from_euler
##------------------------------------------------------------------------------------------------
## Local Mission decomper
##------------------------------------------------------------------------------------------------------------------------------------
class DummyLocalDecomposer(object):
	# Constructor of the class
	def __init__(self):
		self.TaskPublisher = rospy.Publisher('task_array', String, queue_size = 100) #Custom message
		rospy.Subscriber("local_mission", String, self.localMissionCallback)

		rate = rospy.Rate(1)
		while not rospy.is_shutdown():
			try:
				task='<> (r1 && <> r2 && <> r3)'
				self.TaskPublisher.publish(task)
				print 'Send the task %s' %(task) 
				rate.sleep()
			except rospy.ROSInterruptException:
			   	pass

	def localMissionCallback(self, localmission):
	# 	# string containing the local mission
	 	pass



if __name__ == '__main__':
	rospy.init_node('dummylocaldecomposer')
	try:
		mg=DummyLocalDecomposer()

	except rospy.ROSInterruptException:
		pass

