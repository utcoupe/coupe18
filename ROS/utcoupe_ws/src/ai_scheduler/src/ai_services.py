#!/usr/bin/python
import rospy, time
from robot_ai.srv import AIGenericCommand, AIGenericCommandResponse

class AIServices():
	def __init__(self, department, package):
		self.DepartmentName, self.PackageName = department, package
		rospy.Service(self.PackageName, AIGenericCommand, self.on_generic_command)
		
	def on_generic_command(self, req):
		if self.validate_request(req):
			res_code, reason = self.executeService(req)
		else:
			res_code, reason = 403, "Bad request. Wrong department or destination."
		return AIGenericCommandResponse(res_code, reason)

#/*==========================================
#=            Service executions            =
#==========================================*/
	def executeService(self, req):
		if req.command == "ai_delay":
			res_code, reason = self.service_delay(req.params)
		else:
			res_code, reason = 404, "command not recognized"
		return res_code, reason


	def service_delay(self, params):
		rospy.loginfo("[AI] Sleeping for {:.2f} seconds...".format(float(params)))
		time.sleep(float(params))
		return 200, ""

#/*=====  End of Service executions  ======*/


	def tool_manual_response(self):
		return 200 if bool(input("[" + self.__repr__() + "] Send success or not ? ")) else 600, "Response code manually set."
	def validate_request(self, req):
		if not req.department  == self.DepartmentName: 	return False
		if not req.destination == self.PackageName: 	return False
		rospy.logdebug("[{}] NEW SERVICE REQUEST : \nDepartment: {}\nDestination : {}\nCommand : {}\nParams:{}".format(
			self.__repr__(), req.department, req.destination, req.command, req.params))
		return True
	def __repr__(self):
		return "/{}/{}".format(self.DepartmentName, self.PackageName)


if __name__ == '__main__':
	AIServices()