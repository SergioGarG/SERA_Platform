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

from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory

class Status(object):
	def __init__(self):
		print 'status init' 
		self.actionid = StringStamped()
		self.goalid = PoseStamped()
		self.action_result = StringStamped()
		self.goal_result = PoseStamped()
		self.humans = Humans()
		self.robots = Robots()
		self.objects = Objects()
		self.ft = WrenchStamped()
		self.xtion = PointCloud()
		self.joint = JointState()
		self.velocity = Twist()
		self.arm = JointTrajectory()
		self.torso = JointTrajectory()
		self.head = JointTrajectory()
		self.gripper = JointTrajectory()

class MotionControl(object):
# Constructor of the class
	def __init__(self):
		self.status=Status()
		# Publishers
		VelPublisher = rospy.Publisher('/base_controller/cmd_vel', Twist, queue_size = 100)
		ArmPublisher = rospy.Publisher('/arm_controller', Twist, queue_size = 100)
		TorsoPublisher = rospy.Publisher('/torso_controller', Twist, queue_size = 100)
		HeadPublisher = rospy.Publisher('/head_controller', Twist, queue_size = 100)
		GripperPublisher = rospy.Publisher('/gripper_controller', Twist, queue_size = 100)
		ActionPublisher = rospy.Publisher('/action/result', StringStamped, queue_size = 100)
		MovePublisher = rospy.Publisher('/move_base/result', MoveBaseActionResult, queue_size = 100)

	def update_manager(self):
		#print 'update' 
		self.SendAction(self.status.action_result)
		self.SendMove(self.status.move_result)
		self.SendVel(self.status.velocity)
		self.SendArm(self.status.arm)
		self.SendTorso(self.torso)
		self.SendHead(self.head)
		self.SendGripper(self.gripper)


	def ActionIdCallback(self, action_id):
		self.status.actionid=action_id
		self.update_manager()
	def MoveCallback(self, goal):
		self.status.goal=goal
		self.update_manager()
	def HumansCallback(self, humans):
		self.status.humans=humans
		self.update_manager()
	def ObjectsCallback(self, objects):
		self.status.objects=objects
		self.update_manager()
	def RobotsCallback(self, robots):
		self.status.robots=robots
		self.update_manager()
	def XtionCallback(self, cloud):
		self.status.xtion=action_id
		#self.update_manager()
	def FtCallback(self, wrench):
		self.status.ft=wrench
		#self.update_manager()
	def JointCallback(self, joints):
		self.status.joints=joints
		#self.update_manager()

	def SendAction(self, action_result):
		self.ActionPublisher.publish(action_result)
	def SendMove(self, move_result):
		self.MovePublisher.publish(move)
	def SendVel(self, vel):
		self.VelPublisher.publish(vel)
	def SendArm(self, arm):
		self.ArmPublisher.publish(arm)
	def SendTorso(self, torso):
		self.TorsoPublisher.publish(torso)
	def SendHead(self, head):
		self.HeadPublisher.publish(head)
	def SendGripper(self, gripper):
		self.GripperPublisher.publish(gripper)

	def run_component(self):
		rospy.init_node('motioncontrol')
		# Subscribers
		rospy.Subscriber("move_base_simple/goal", PoseStamped, self.MoveCallback)
		rospy.Subscriber("action/id", StringStamped, self.ActionIdCallback)
		rospy.Subscriber("tracked_humans", Humans, self.HumansCallback)
		rospy.Subscriber("tracked_objects", Objects, self.ObjectsCallback)
		rospy.Subscriber("tracked_robots", Robots, self.RobotsCallback)
		rospy.Subscriber("xtion/depth_registered/points", PointCloud, self.XtionCallback)
		rospy.Subscriber("ft_sensor", WrenchStamped, self.FtCallback)
		rospy.Subscriber("joint_states", JointState, self.JointCallback)

		rate = rospy.Rate(10)
		while not rospy.is_shutdown():
			try:
				rate.sleep()
			except rospy.ROSInterruptException:
				pass

	@abstractmethod
	def motioncontrol_method(self, position_goal, action_id, humans, objects, robots, cloud, joints):
		pass

