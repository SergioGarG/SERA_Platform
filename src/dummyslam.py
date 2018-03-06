#!/usr/bin/env python
import rospy
import roslib
import numpy
import Queue
import sys
import time


from slam import Slam

##------------------------------------------------------------------------------------------------
## ADAPTATION MANAGER
##------------------------------------------------------------------------------------------------------------------------------------
class DummySlam(Slam):
	def __init__(self):
		Slam.__init__(self)

	def slam(self, currentmap, robot_pose, robot_status, tracked_humans, tracked_objects, tracked_robots):
		pass


if __name__ == '__main__':
	try:
		print 'Calling the dummyslam method' 
		mg=DummySlam()
		mg.run_component()
	except rospy.ROSInterruptException:
		pass


	
	
