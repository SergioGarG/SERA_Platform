#!/usr/bin/env python
import rospy
import roslib
import numpy
import Queue
import sys
import time


from communicationmanager import CommunicationManager

##------------------------------------------------------------------------------------------------
## LOCAL MISSION MANAGER
##------------------------------------------------------------------------------------------------------------------------------------
class DummyCommunicationManager(CommunicationManager):
	def __init__(self):
		CommunicationManager.__init__(self)

	def communicationmanager_method(self, map, shared_map, robotstatus, robot_status_robotnumber, localmission, local_mission_robotnumber, robot_pose, robot_pose_robotnumber):
		pass

if __name__ == '__main__':
	try:
		print 'Calling the dummycommunicationmanager method' 
		mg=DummyCommunicationManager()
		mg.run_component()
	except rospy.ROSInterruptException:
		pass


	
	
