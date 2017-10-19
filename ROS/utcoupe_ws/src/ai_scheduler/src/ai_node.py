#!/usr/bin/python
import rospy, time
from ai_communication import AICommunication
from ai_services import AIServices
from AI import RobotAI, TaskStatus

from robot_ai.msg import AICommand

class AINode():
	def __init__(self):
		self.DepartmentName, self.PackageName = "ai", "scheduler"

		self.NODE = rospy.init_node(self.PackageName, log_level = rospy.DEBUG)
		self.communication = AICommunication()

		self.wait_for_departments()

		self.AI = RobotAI("strategy_ftw") #TODO get strategy name from command line param
		self.services = AIServices(self.DepartmentName, self.PackageName)

		self.runAI()

	def runAI(self):
		#Debug: show task tree when starting the system
		rospy.loginfo("[AI] Launching robot with strategy :")
		self.AI.Strategy.PrettyPrint()

		# Run the whole AI until there are no orders left to execute
		while not rospy.is_shutdown():
			if self.AI.Strategy.canContinue():
				self.AI.Strategy.getNext().execute(self.communication)
			else:
				self.AI.Strategy.PrettyPrint()
				rospy.loginfo("[AI] In-Game actions finished!")
				break
			self.AI.Strategy.PrettyPrint()


	def wait_for_departments(self):
		pass # TODO wait for all services to be ready before launching the actions.

'''
PACKAGE STARTING POINT HERE
'''
if __name__ == "__main__":
	node = AINode()
