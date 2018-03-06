# the class is implemented using a template method pattern.
# each plan executor extends the class plan executor and provides the logic for executing plans
#!/usr/bin/env python
#import roslib
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

from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, PolygonStamped, Point32

from std_msgs.msg import Bool, String
from nav_msgs.msg import OccupancyGrid
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionGoal, MoveBaseActionResult

from tf.transformations import euler_from_quaternion, quaternion_from_euler

from ms1_msgs.msg import PlanMsg, ActionSeq


#from nav_msgs import OccupancyGrid 
	
# def FormatGoal(goal, index, time_stamp):
# 	GoalMsg = MoveBaseGoal()
# 	GoalMsg.target_pose.header.seq = index
# 	GoalMsg.target_pose.header.stamp = time_stamp
# 	GoalMsg.target_pose.header.frame_id = 'map'
# 	GoalMsg.target_pose.pose.position.x = goal[0]
# 	GoalMsg.target_pose.pose.position.y = goal[1]
# 	quaternion = quaternion_from_euler(0, 0, goal[2])
# 	GoalMsg.target_pose.pose.orientation.x = quaternion[0]
# 	GoalMsg.target_pose.pose.orientation.y = quaternion[1]
# 	GoalMsg.target_pose.pose.orientation.z = quaternion[2]
# 	GoalMsg.target_pose.pose.orientation.w = quaternion[3]
# 	return GoalMsg

def FormatGoal(goal, index, time_stamp):
	GoalMsg = PoseStamped()
	GoalMsg.header.seq = index
	GoalMsg.header.stamp = time_stamp
	GoalMsg.header.frame_id = "map"
	GoalMsg.pose.position.x = goal[0]
	GoalMsg.pose.position.y = goal[1]
	quaternion = quaternion_from_euler(0, 0, goal[2])
	GoalMsg.pose.orientation.x = quaternion[0]
	GoalMsg.pose.orientation.y = quaternion[1]
	GoalMsg.pose.orientation.z = quaternion[2]
	GoalMsg.pose.orientation.w = quaternion[3]
	return GoalMsg

class PlanExecutor(object):
# Constructor of the class
    def __init__(self):
        rospy.init_node('planexecutor')
        # Publishers
        # MovePublisher = rospy.Publisher('move_base/goal', MoveBaseGoal, queue_size = 100)
        ActionPublisher = rospy.Publisher('action/id', StringStamped, queue_size = 100)
        MoveResultPublisher = rospy.Publisher('move_base/result', MoveBaseActionResult, queue_size = 100)
        ActionResultPublisher = rospy.Publisher('action/result', ActionSeq, queue_size = 100)
        
        # When using the integrated simulator the name of the robot should be added as a suffix of the topic (i.e. robot_0/move_base)
        self.navigation=actionlib.SimpleActionClient("robot_0/move_base", MoveBaseAction)
        self.navigation.wait_for_server(rospy.Duration(5))


    def ActionResultCallback(self, action_result):
        # result of the action
        pass
    def MoveResultCallback(self, move_result):
        # result of the move
        pass
    def PlanCallback(self, plan):
        # plan
        print 'new plan received!'
        # Decompose the plan into different calls
        # Depending the type of call send move or action
        #self.status.task_array.append(tasks_array.data)
        self.plan_executor(plan)

    def sendGoal(self, position, orientation, frame_id, seq):
        goalMsg = MoveBaseGoal()
        goalMsg.target_pose.header.seq = seq
        goalMsg.target_pose.header.stamp = rospy.Time.now()
        goalMsg.target_pose.header.frame_id = frame_id
        goalMsg.target_pose.pose.position = position
        goalMsg.target_pose.pose.orientation = orientation
        #self.MovePublisher.publish(goalMsg)
        self.navigation.send_goal(goalMsg)
        print 'Goal sent! %s' %(str(goalMsg))
        while (not rospy.is_shutdown()) and (self.navigation.get_state() != GoalStatus.SUCCEEDED):
            try:
                ###############  check for model update
                rospy.sleep(1)
            except rospy.ROSInterruptException:
                pass                    

        # format the output msg from PlanMsg to action movement
    def SendAction(self, action_id, time_stamp):
        # format the output msg from PlanMsg to action 
        pass
    def SendMoveResult(self, move_result, time_stamp):
        # sends the result of the move
        pass
    def SendActionResult(self, action_result, time_stamp):
        # sends the result of the action
        pass

    def run_component(self):
        # code to execute the current plan
        # Subscribers
        rospy.Subscriber("plan", PlanMsg, self.PlanCallback)
        #rospy.Subscriber("action/result", StringStamped, self.ActionResultCallback)
        rospy.Subscriber("action/result", ActionSeq, self.ActionResultCallback) #For testing using string instead of stringstamped
        rospy.Subscriber("move_base/result", MoveBaseActionResult, self.MoveResultCallback)

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                pass

    @abstractmethod
    def plan_executor(self, plan):
        pass
