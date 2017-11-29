#!/usr/bin/python
import rospy

from timer import GameTimer

from ai_game_status.msg import GameStatus
from ai_game_status.srv import SetStatus, SetStatusResponse


class Status():
    STATUS_INITIALIZING = 0
    STATUS_INITIALIZED  = 1
    STATUS_ARMING       = 2
    STATUS_ARMED        = 3
    STATUS_INGAME       = 4
    STATUS_POSTGAME     = 5
    STATUS_HALT         = 6


class GameStatusNode():
    def __init__(self):
        rospy.init_node("game_status", log_level=rospy.DEBUG)
        self._srv = rospy.Service("/ai/game_status/set", SetStatus, self.on_set_status)
        self._pub = rospy.Publisher("/ai/game_status/status", GameStatus, queue_size=10)
        self._timer = GameTimer()

        self.game_status = Status.STATUS_INITIALIZING
        self.game_duration = -1

        self.run()

    def run(self):
        r = rospy.Rate(5)
        while not rospy.is_shutdown():
            m = GameStatus()
            m.game_status = self.game_status
            m.is_gaming = self._is_gaming()

            if m.is_gaming:
                m.game_time_duration = self.game_duration
                m.game_elapsed_time = self._timer.elapsed_time()
                m.game_time_left = self._timer.time_left()

            self._pub.publish(m)
            r.sleep()

    def on_set_status(self, req):
        # TODO process status change (start/stop timer, error handling...)
        if req.game_status == Status.STATUS_INGAME:
            self._timer.start()
        self.game_status = req.game_status
        return SetStatusResponse()

    def _is_gaming(self):
        return Status.STATUS_INGAME <= self.game_status <= Status.STATUS_POSTGAME


if __name__ == "__main__":
    GameStatusNode()
