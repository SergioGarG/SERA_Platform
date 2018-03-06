#!/usr/bin/env python
#import roslib
import numpy
import Queue
import rospy
import sys
import time

import copy
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

from ms1_msgs.msg import PlanMsg, PosePlanArray, MovementPlan, Humans, ActionSeq

class Status(object):
	def __init__(self, map, robot_pose, robot_status, tracked_humans):
		print 'status init' 
		self.map = map
		self.task_array = []
		self.taskindex = -1
		self.robot_pose = robot_pose
		self.robot_status = robot_status
		self.tracked_humans = tracked_humans

	def print_data(self):
		print 'status= %s %s %d %s %s %s' %(self.map, self.task_array, self.taskindex, self.robot_pose, self.robot_status, self.tracked_humans)

class Plan(object):
	def __init__(self, type, actionPlan, movementPlan):
		print 'RetPlan' 
		self.type = type
		self.actionPlan = actionPlan
		self.movementPlan = movementPlan


##------------------------------------------------------------------------------------------------------------------------------------
## ADAPTATION MANAGER
##------------------------------------------------------------------------------------------------------------------------------------
class AdaptationManager(object):

	# Constructor of the class
	def __init__(self):

		self.status=Status('','','','')
		# Publishers
		self.PlanPublisher = rospy.Publisher('plan', PlanMsg, queue_size = 100) #for simple example
		#FeedbackPublisher = rospy.Publisher('feedback', Feedback, queue_size = 100) #Custom message

	def update_manager(self):
		#print 'update' 
		self.status.print_data()
		task=self.status.task_array[self.status.taskindex]
		retplan=self.plan(self.status.map, task, self.status.robot_pose, self.status.robot_status, self.status.tracked_humans)
		self.SendPlan(retplan)
		

	def TasksCallback(self, tasks_array):
		print 'new task received!'
		print 'task: %s' %(tasks_array.data)
		self.status.task_array.append(tasks_array.data)
		self.status.taskindex=self.status.taskindex+1

		self.update_manager()

	# string containing the array of tasks
	def PoseCallback(self, posedata):
		self.status.robot_pose=posedata

	# it updates the state of the robot
	def StatusCallback(self, robot_status):
		self.status.robot_status=robot_status

	# TODO: update the logic for adding new tasks
	def TaskUpdateCallback(self, task):
		self.status.tasks_array[status.tasks_array__len__()+1]=task	
		if self.status.taskindex == len(self.status.tasks_array):
			self.update_manager()   


	# string containing the new local mission for the robot
	def HumansCallback(self, humans):
		self.status.tracked_humans.humans
		self.update_manager()


	# metadata regarding tracked humans
	def ActionResultCallback(self, action_result):
		if (action_result=='OK'):
			self.status.taskindex=self.status.taskindex+1
			if(self.status.tasks_array__len__()>(self.status.taskindex)):
				self.update_manager()

	# result of the action
	def MoveResultCallback(self, move_result):
		if (move_result=='OK'):
			self.status.taskindex=self.status.taskindex+1
			if(self.status.tasks_array__len__()>(self.status.taskindex)):
			   self.update_manager()

	def SendPlan(self, plan):
		planmsg=PlanMsg()
		planmsg.header.seq = sequence=+1
		planmsg.header.stamp = rospy.Time.now()
		planmsg.header.frame_id = 'map'
		planmsg.type.data=plan.type
		planmsg.actionPlan.data=''
		planmsg.movementPlan.PrePlan=list()

		for i in range(0, len(plan.movementPlan.pre_plan)):
			ps=PosePlanArray()
			ps.posestamped=PoseStamped()
			ps.posestamped.header.seq = i
			ps.posestamped.header.stamp = rospy.Time.now()
			ps.posestamped.header.frame_id = 'map'
			
			pos=plan.movementPlan.pre_plan[i]

			ps.posestamped.pose.position.x = pos[0]
			ps.posestamped.pose.position.y = pos[1]
			quaternion = quaternion_from_euler(0, 0, pos[2]) #roll, pitch, yaw
			ps.posestamped.pose.orientation.x = quaternion[0]
			ps.posestamped.pose.orientation.y = quaternion[1]
			ps.posestamped.pose.orientation.z = quaternion[2]
			ps.posestamped.pose.orientation.w = quaternion[3]

			planmsg.movementPlan.PrePlan.append(ps)

		planmsg.movementPlan.SufPlan=list()
		for i in range(0, len(plan.movementPlan.suf_plan)):
			ps=PosePlanArray()
			ps.posestamped=PoseStamped()
			ps.posestamped.header.seq = i
			ps.posestamped.header.stamp = rospy.Time.now()
			ps.posestamped.header.frame_id = 'map'
			
			pos=plan.movementPlan.suf_plan[i]

			ps.posestamped.pose.position.x = pos[0]
			ps.posestamped.pose.position.y = pos[1]
			quaternion = quaternion_from_euler(0, 0, pos[2]) #roll, pitch, yaw
			ps.posestamped.pose.orientation.x = quaternion[0]
			ps.posestamped.pose.orientation.y = quaternion[1]
			ps.posestamped.pose.orientation.z = quaternion[2]
			ps.posestamped.pose.orientation.w = quaternion[3]

			planmsg.movementPlan.SufPlan.append(ps)

		self.PlanPublisher.publish(planmsg)
		#navigation.send_goal(navi_goal)


	# invocated when a new map is received
	def MapCallback(self, map):
		# updating the current map
		self.status.currentmap=map
		print "New map received!"
		# calling the planner after the new change
		self.update_manager()#self.status)
 

	def run_component(self):
		#global status
		rospy.init_node('adaptationmanager')
		print 'adaptationmanager started!' 


 
		# initializes the adaptation manager node
		rospy.init_node('adaptationmanager')

		# Map: contains an occupancy grid which shows which cells of the environment are occupied
		rospy.Subscriber("map", OccupancyGrid, self.MapCallback)
		
		# Contains an array of tasks the robot should perform
		#rospy.Subscriber("task_array", TasksArray, TasksCallback) #Custom message
		rospy.Subscriber("task_array", String, self.TasksCallback) #String message
		
		# Receives a new position and orientation of the robot
		rospy.Subscriber('robot_pose', PoseWithCovarianceStamped, self.PoseCallback)

		# Reveices information about the robot status, i.e., level of battery etc
		#rospy.Subscriber('robot_status', RobotStatus, StatusCallback) #Custom message
		rospy.Subscriber('robot_status', String, self.StatusCallback) #String message

		# A new task is detected 
		rospy.Subscriber('task_update', String, self.TaskUpdateCallback)

		# It is called when the result of an action is received
		rospy.Subscriber("action/result", ActionSeq, self.ActionResultCallback) 

		# New information about the human is received
		rospy.Subscriber("tracked_humans", Humans, self.HumansCallback) 

		# It is called when the result of a movement is received
		rospy.Subscriber("move_base/result", MoveBaseActionResult, self.MoveResultCallback)

		#rospy.loginfo("wait for the move_base action server to come up")
		#allows up to 5 seconds for the action server to come up
		#navigation.wait_for_server(rospy.Duration(5))

		rate = rospy.Rate(10)
		while not rospy.is_shutdown():
			try:
				#print 'Calling the update method' 
				#self.update_manager()
				rate.sleep()
			except rospy.ROSInterruptException:
			   	pass

 	# logic for computing the new plan.
 	@abstractmethod
 	def plan(self, currentmap, task, robot_pose, robot_status, tracked_humans):
 		pass

