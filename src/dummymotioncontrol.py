#!/usr/bin/env python
import rospy
import roslib
import numpy
import Queue
import sys
import time


from motioncontrol import MotionControl

##------------------------------------------------------------------------------------------------
## ADAPTATION MANAGER
##------------------------------------------------------------------------------------------------------------------------------------
class DummyMotionControl(MotionControl):
	def __init__(self):
		MotionControl.__init__(self)

	def motioncontrol(self, position_goal, action_id, humans, objects, robots, cloud, joints):
		pass


if __name__ == '__main__':
	try:
		print 'Calling the dummymotioncontrol method' 
		mg=DummyMotionControl()
		mg.run_component()
	except rospy.ROSInterruptException:
		pass


	
	
