# -*- coding: utf-8 -*-
import rospy
import robot_ai.srv

class AICommunication():
	def __init__(self):
		pass

	def SendGenericCommand(self, msg_department, msg_dest, msg_command, msg_params):
		# Send a simple command service, with string command and params. Expecting a bool and string for response
		dest = "/{}/{}".format(msg_department, msg_dest)
		rospy.logdebug("[AI] Sending service request to service '{}'...".format(dest))

		rospy.logdebug("[AI]     Waiting for service to be available...")
		rospy.wait_for_service(dest)

		rospy.logdebug("[AI]     Sending request.")
		service = rospy.ServiceProxy(dest, robot_ai.srv.AIGenericCommand)
		rospy.logdebug("[AI]     Got response!")

		return service(msg_department, msg_dest, msg_command, msg_params)

# THREADING THIS : https://stackoverflow.com/questions/6800984/python-how-to-pass-and-run-a-callback-method-in-python ==> FORGET : USE ACTIONS
