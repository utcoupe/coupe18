#!/usr/bin/python
import rospy
from ai_game_status.msg import GameStatus

class GameStatus():
    STATUS_INITIALIZING = 0
    STATUS_INITIALIZED  = 1
    STATUS_ARMING       = 2
    STATUS_ARMED        = 3
    STATUS_INGAME       = 4
    STATUS_POSTGAME     = 5
    STATUS_HALT         = 6


class GameStatusNode():
    def __init__(self):
        rospy.init_node("game_status", log_level=rospy.LEVEL_DEBUG)


class Timer():
    def __init__(self):
        self.PUBL = rospy.Publisher("/ai/timer/time", GameStatus, queue_size = 10)

        self.Duration = None # Holds the match duration.
        self.Started = False # Set to true when Timer is triggered.

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
    lsqdkfjghlsdkfjhg()