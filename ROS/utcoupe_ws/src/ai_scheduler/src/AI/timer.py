import rospy, time

from robot_ai.msg import ai_timer

class GameTimer():
	def __init__(self):
		rospy.Subscriber("/game_timer", ai_timer, self.on_new_time)

		self._finished = False

	def on_new_time(self, msg):
		if msg.is_finished:
			if not self._finished:
				rospy.logwarn("GAME FINISHED")
				self._finished = True
