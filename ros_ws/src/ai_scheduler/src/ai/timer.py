import rospy
from ai_game_status.msg import GameTime

class GameTimer():
    def __init__(self):
        rospy.Subscriber("/ai/game_status/timer", GameTime, self.on_new_time)
        self._finished = False

    def on_new_time(self, msg):
        if msg.is_finished:
            if not self._finished:
                rospy.logwarn("[AI] GAME FINISHED EVENT FROM TIMER, STOPPING EXECUTION.")
                self._finished = True
        # TODO stop execution.
