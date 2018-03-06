#!/usr/bin/env python
import rospy
import roslib
import numpy
import Queue
import sys
import time


from gesturesrecognition import GesturesRecognition

##------------------------------------------------------------------------------------------------
## GESTURES RECOGNITION
##------------------------------------------------------------------------------------------------------------------------------------
class DummyGesturesRecognition(GesturesRecognition):
	def __init__(self):
		GesturesRecognition.__init__(self)

	def gesturesrecognition(self, cloud, images, robot_status, tracked_humans):
		pass


if __name__ == '__main__':
	try:
		print 'Calling the dummygesturesrecognition method' 
		mg=DummyGesturesRecognition()
		mg.run_component()
	except rospy.ROSInterruptException:
		pass


	
	
