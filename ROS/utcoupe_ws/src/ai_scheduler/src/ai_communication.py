# -*- coding: utf-8 -*-
import rospy
import ai_scheduler.srv

class AICommunication():
	def __init__(self):
		pass

	def SendGenericCommand(self, msg_dest, msg_params):
		# Send a simple command service, with string command and params. Expecting a bool and string for response

		rospy.logdebug("[AI] Sending service request to service '{}'...".format(msg_dest))

		rospy.logdebug("[AI]     Waiting for service to be available...")
		rospy.wait_for_service(msg_dest)

		rospy.logdebug("[AI]     Available. Sending request...")
		service = rospy.ServiceProxy(msg_dest, ai_scheduler.srv.AIGenericCommand)
		rospy.logdebug("[AI]     Got response!")

		#TODO: create multiple classes to parse params depending on dest

		return service(msg_dest, msg_params)

# THREADING THIS : https://stackoverflow.com/questions/6800984/python-how-to-pass-and-run-a-callback-method-in-python
# TODO ==> FORGET : IMPLEMENT ACTIONS INSTEAD
