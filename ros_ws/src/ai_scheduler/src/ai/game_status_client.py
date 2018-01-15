import rospy
from ai_game_status.msg import GameStatus

class GameStatusClient():
    def __init__(self):
        rospy.Subscriber("/ai/game_status/status", GameStatus, self.on_new_status)
        self.game_status = 0
        self.init_status = 0

    def on_new_status(self, msg):
        self.game_status = msg.game_status
        self.init_status = msg.init_status
