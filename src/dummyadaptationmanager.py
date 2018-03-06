#!/usr/bin/env python
#import rospy
#import roslib
import numpy
import Queue
import sys
import time


from adaptationmanager import AdaptationManager, Goal, RetTask

##------------------------------------------------------------------------------------------------
## ADAPTATION MANAGER
##------------------------------------------------------------------------------------------------------------------------------------
class DummyAdaptationManager(AdaptationManager):
	def __init__(self):
		AdaptationManager.__init__(self)

	def plan(self, currentmap, robot_pose, robot_status, tracked_humans):
		position=[-1, 0, 1]
		goal=Goal(position, 1, rospy.get_rostime())
		rettask=RetTask('GOAL', '', goal)
		return rettask

mg=DummyAdaptationManager()
mg.run_test()


if __name__ == '__main__':
	try:
		print 'Calling the dummyadaptationmanager method' 
		mg=DummyAdaptationManager()
		mg.run_component()
	except rospy.ROSInterruptException:
		pass


	
	
