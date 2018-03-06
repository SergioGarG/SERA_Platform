#!/usr/bin/env python
import rospy
import roslib
import numpy
import Queue
import sys
import time


from detectionexecutor import DetectionExecutor

##------------------------------------------------------------------------------------------------
## Detection executor
##------------------------------------------------------------------------------------------------------------------------------------
class DummyDetectionExecutor(DetectionExecutor):
	def __init__(self):
		DetectionExecutor.__init__(self)

	def detectionexecutor_method(self, depth, image, roi, humans, objects, robots):
		pass

if __name__ == '__main__':
	try:
		print 'Calling the dummydetectionexecutor method' 
		mg=DummyDetectionExecutor()
		mg.run_component()
	except rospy.ROSInterruptException:
		pass


	
	
