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
        self.missionext = StringStamped()
        self.robotpose = PoseWithCovarianceStamped()
        self.robotposeext = PoseWithCovarianceStamped()
        self.robotstatus = StringStamped()
        self.robotstatusext = StringStamped()
        self.cloud = PointCloud()
        self.cloud_ext = PointCloud()
        self.image = Image()
        self.image_ext = Image()


class CommunicationManager(object):
    # Constructor of the class
    def __init__(self):
        self.status=Status()
        # Publishers, some commented because will be translated to REST format
        # LocalMissionIntPublisher = rospy.Publisher('local_mission_robotnumber', StringStamped, queue_size = 100) #Mission of the local robot to be sent the external team
        LocalMissionExtPublisher = rospy.Publisher('local_mission_robotnumber', StringStamped, queue_size = 100) #Mission from an external robot to be shared in local
        # SharedMapPublisher = rospy.Publisher('shared_map', OccupancyGrid, queue_size = 100) #Shared map to be sent to the team
        SharedMapExtPublisher = rospy.Publisher('shared_map', OccupancyGrid, queue_size = 100) #Shared map received from the team
        # RobotPosePublisher = rospy.Publisher('robot_pose_robotnumber', PoseWithCovarianceStamped, queue_size = 100) #Pose of the local robot to be shared with the team
        RobotPoseExtPublisher = rospy.Publisher('robot_pose_robotnumber', PoseWithCovarianceStamped, queue_size = 100) #Pose of some external robot yo be shared locally
        # RobotStatusPublisher = rospy.Publisher('robot_status_robotnumber', StringStamped, queue_size = 100) #Status of the local robot to be shared with the team
        RobotStatusExtPublisher = rospy.Publisher('robot_status_robotnumber', StringStamped, queue_size = 100) #Status of some external robot yo be shared locally
        # DepthCloudPublisher = rospy.Publisher('depth_cloud_robotnumber', PointCloud, queue_size = 100) #Depth cloud detected locally by the robot to be shared with the team
        DepthCloudExtPublisher = rospy.Publisher('depth_cloud_robotnumber', PointCloud, queue_size = 100) #Depth cloud detected by an external robot to be shared locally
        # ImagePublisher = rospy.Publisher('image_raw_robotnumber', Image, queue_size = 100) #Image raw feed detected locally by the robot to be shared with the team
        ImageExtPublisher = rospy.Publisher('image_raw_robotnumber', Image, queue_size = 100) #Image raw feed detected by an external robot to be shared locally

    def update_manager(self):
        #print 'update' 
        self.SendMissionExt(self.status.missionext)
        self.SendSharedMap(self.status.sharedmap)
        self.SendRobotPoseExt(self.status.robotposeext)
        self.SendRobotStatusExt(self.status.robotstatusext)
        self.SendDepthCloudExt(self.status.cloud_ext)
        self.SendImageExt(self.status.image_ext)
        # Also send the REST messages concerning the local robot to the team


    def MapCallback(self, map):
        # receive the current map of the environment
        self.status.map = map
    def SharedMapCallback(self, shared_map):
        # receive the current map of the environment with the info from the rest of the team
        self.status.sharedmap = shared_map
    def LocalMissionCallback(self, localmission):
        # string containing the local mission
        self.status.mission = localmission
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
    def StatusExtCallback(self, robotstatus):
        # custom message containing info regarding the robot status (proprioceptive sensors)
        self.status.robotstatusext = robotstatus

    def SendLocalExt(self, missionext):
        # sends the received local mission after checking its feasability
        self.LocalMissionPublisher.publish(missionext)
    def SendSharedMap(self, sharedmap):
        self.SharedMapPublisher.publish(sharedmap)
    def SendRobotPoseExt(self, pose):
        self.RobotPoseExtPublisher.publish(pose)
    def SendRobotStatusExt(self, status):
        self.RobotPoseExtPublisher.publish(status)
    def SenfDepthCloudExt(self, cloud):
        self.DepthCloudExtPublisher.publish(cloud)
    def SendImageExt(self, image):
        self.ImageExtPublisher.publish(image)


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

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                pass

    @abstractmethod
    def communicationmanager_method(self, map, shared_map, robotstatus, robot_status_robotnumber, localmission, local_mission_robotnumber, robot_pose, robot_pose_robotnumber):
        pass
