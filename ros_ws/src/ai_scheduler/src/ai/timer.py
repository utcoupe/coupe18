import rospy
from ai_game_status.msg import GameTime

class GameTimer():
    def __init__(self):
        rospy.Subscriber("/ai/timer/time", GameTime, self.on_new_time)
        self.finished = False

    def on_new_time(self, msg):
        if msg.is_active:
            if not self.finished:
                rospy.logwarn("[AI] GAME FINISHED EVENT FROM TIMER, STOPPING EXECUTION.")
                self.finished = True
        # TODO stop execution.
