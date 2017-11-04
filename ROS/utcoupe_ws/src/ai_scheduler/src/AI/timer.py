import rospy, time
from ai_timer.msg import ai_timer

class GameTimer():
    def __init__(self):
        rospy.Subscriber("/game_timer", ai_timer, self.on_new_time)

        self._finished = False

    def on_new_time(self, msg):
        if msg.is_finished:
            if not self._finished:
                rospy.logwarn("[AI] GAME FINISHED EVENT FROM TIMER")
                self._finished = True
