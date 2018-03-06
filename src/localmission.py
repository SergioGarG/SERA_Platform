# the class is implemented using a template method pattern.
#!/usr/bin/env python

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
        self.map = OccupancyGrid()
        self.sharedmap = OccupancyGrid()
        self.mission = StringStamped()
        self.missionupdate = StringStamped()
        self.missionext = StringStamped()
        self.robotpose = PoseWithCovarianceStamped()
        self.robotposeext = PoseWithCovarianceStamped()
        self.robotstatus = StringStamped()
        self.robotstatusext = StringStamped()


class LocalMission(object):
    # Constructor of the class
    def __init__(self):
        self.status=Status()
        # Publishers
        LocalMissionPublisher = rospy.Publisher('local_mission', StringStamped, queue_size = 100)

    def update_manager(self):
        #print 'update' 
        self.SendAction(self.status.action_result)


    def MapCallback(self, map):
        # receive the current map of the environment
        self.status.map = map
    def SharedMapCallback(self, shared_map):
        # receive the current map of the environment with the info from the rest of the team
        self.status.sharedmap = shared_map
    def LocalMissionCallback(self, localmission):
        # string containing the local mission
        self.status.mission = localmission
    def MissionUpdateCallback(self, missionupdate):
        # string containing the new local mission for the robot
        self.status.missionupdate = missionupdate
    def LocalMissionExtCallback(self, localmissionext):
        # string containing the local mission
        self.status.missionext = localmissionext
    def PoseCallback(self, posedata):
        # PoseWithCovarianceStamped data
        self.status.robotpose = posedata
    def PoseExtCallback(self, posedata):
        # PoseWithCovarianceStamped data
        self.status.robotposeext = posedata
    def StatusCallback(self, robotstatus):
        # custom message containing info regarding the robot status (proprioceptive sensors)
        self.status.robotstatus = robotstatus
    def StatusExtCallback(self, robotstatus):
        # custom message containing info regarding the robot status (proprioceptive sensors)
        self.status.robotstatusext = robotstatus


    def SendLocalMission(self, localmission):
    # sends the received local mission after checking its feasability
        self.LocalMissionPublisher.publish(localmission)


###Maybe we should send the position of each robot as well. This information could be included in robot_status_robotnumber#####

    def run_component(self):
        rospy.init_node('localmission')
        # Subscribers
        rospy.Subscriber("map", OccupancyGrid, self.MapCallback)
        rospy.Subscriber("shared_map", OccupancyGrid, self.SharedMapCallback)
        rospy.Subscriber("local_mission", StringStamped, self.LocalMissionCallback)
        rospy.Subscriber("local_mission_robotnumber", StringStamped, self.LocalMissionExtCallback)
        rospy.Subscriber('robot_pose', PoseWithCovarianceStamped, self.PoseCallback)
        rospy.Subscriber('robot_pose_robotnumber', PoseWithCovarianceStamped, self.PoseExtCallback)
        rospy.Subscriber('robot_status', StringStamped, self.StatusCallback) 
        rospy.Subscriber('robot_status_robotnumber', StringStamped, self.StatusExtCallback) 
        rospy.Subscriber('mission_update', StringStamped, self.MissionUpdateCallback)

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                pass

    @abstractmethod
    def localmission_method(self, map, shared_map, robotstatus, robot_status_robotnumber, localmission, local_mission_robotnumber, robot_pose, robot_pose_robotnumber):
        pass
