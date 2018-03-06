#!/usr/bin/env python
import rospy
import roslib
import numpy
import Queue
import sys
import time


from informationmanager import InformationManager, Status

##------------------------------------------------------------------------------------------------
## INFORMARTION MANAGER
##------------------------------------------------------------------------------------------------------------------------------------
class DummyInformationManager(InformationManager):
	def __init__(self):
		InformationManager.__init__(self)

	def informationmanager_method(self, depth_cloud, depth_cloud_robotnumber, image_raw, image_raw_robotnumber):
		pass


if __name__ == '__main__':
	try:
		print 'Calling the dummyinformationmanager method' 
		mg=DummyInformationManager()
		mg.run_component()
	except rospy.ROSInterruptException:
		pass


	
	
