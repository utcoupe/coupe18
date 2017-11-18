#!/usr/bin/python
import rospy, time

from ai_scheduler.srv import AIGenericCommand, AIGenericCommandResponse
from ai_timer.msg import ai_timer

'''
TIME UNIT IN SECONDS
'''
class TimerNode():
	def __init__(self):
		self.DepartmentName, self.PackageName = "ai", "timer"
		self.NODE = rospy.init_node(self.PackageName, log_level = rospy.INFO)
		self.SERV = rospy.Service("/{}/{}".format(self.DepartmentName, self.PackageName), AIGenericCommand, self.on_srv_request)
		self.PUBL = rospy.Publisher("/game_timer", ai_timer, queue_size = 10)

		self.Duration = None # Holds the match duration.
		self.Started = False # Set to true when Timer is triggered.

		self.run() # Publishes the current time, time left and whether the match is over.

	def on_srv_request(self, req):
		res_code, reason = 200, ""
		if req.command == "timer_set_duration":
			self.Duration = int(req.params)
		elif req.command == "timer_start":
			self.start()
		else:
			rospy.logerr("robot_ai_timer got non-recognized command '{}'.".format(req.command))
			res_code, reason = 404, "command not recognized"
		return AIGenericCommandResponse(res_code, reason)

	def run(self):
		rate = rospy.Rate(10) # 10hz
		while not rospy.is_shutdown():
			if self.Started:
				msg = ai_timer()
				msg.elapsed_time = self.elapsed_time()
				msg.time_left = -1 if not self.Duration else self.time_left()
				msg.is_finished = self.is_finished()
				self.PUBL.publish(msg)
				rate.sleep()

	def start(self):
		self.starttime = time.time() * 1000
		self.Started = True

	def elapsed_time(self):
		return (time.time() * 1000 - self.starttime) / 1000.0 # return elapsed time in seconds
	def time_left(self):
		return self.Duration - self.elapsed_time()
	def is_finished(self):
		return True if self.time_left() < 0 else False


if __name__ == "__main__":
	TimerNode()
