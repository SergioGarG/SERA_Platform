#!/usr/bin/env python
import rospy
import roslib
import numpy
import Queue
import sys
import time


from planexecutor import PlanExecutor


class DummyPlanExecutor(PlanExecutor):
	def __init__(self):
		Slam.__init__(self)

	def plan_method(self, plan):
		for i in range(0,len(plan.movementPlan.PrePlan)):
			self.sendGoal(plan.movementPlan.PrePlan[i].posestamped.pose.position,plan.movementPlan.PrePlan[i].posestamped.pose.orientation, 'map', i)
		for i in range(0,len(plan.movementPlan.SufPlan)):
			self.sendGoal(plan.movementPlan.SufPlan[i].posestamped.pose.position,plan.movementPlan.SufPlan[i].posestamped.pose.orientation, 'map', i)

if __name__ == '__main__':
	#rospy.init_node('dummyplanexecutor')
	try:
		print 'Creating the DummyPlanExecutor' 
		mg=DummyPlanExecutor()
		mg.run_component()
	except rospy.ROSInterruptException:
		pass


	
	
