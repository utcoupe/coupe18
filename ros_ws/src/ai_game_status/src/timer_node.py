#!/usr/bin/python
import time
import rospy

from ai_game_status.msg import GameTime, GameStatus
from ai_game_status.srv import SetTimer, SetTimerResponse, SetStatus


class Status():
    STATUS_INIT   = 0
    STATUS_INGAME = 1
    STATUS_HALT   = 2


class TimerManager():
    def __init__(self):
        self.game_duration = -1  # Holds the match duration.
        self.started = False     # Set to true when Timer is active.

        self._start_time = -1

    def reset(self):
        self._start_time = time.time() * 1000

    def start(self, duration):
        self.reset()
        self.game_duration = duration
        self.started = True

    def stop(self):
        self._start_time = -1
        self.started = False

    def elapsed_time(self):
        return (time.time() * 1000 - self._start_time) / 1000.0 # return elapsed time in seconds

    def time_left(self):
        return self.game_duration - self.elapsed_time()

    def is_finished(self):
        return True if self.time_left() < 0 else False


class TimerNode():
    def __init__(self):
        rospy.init_node("timer", log_level=rospy.DEBUG)
        self._set_timer_srv = rospy.Service("/ai/game_status/set_timer",  SetTimer,  self.on_set_timer)
        self._timer_pub     = rospy.Publisher("/ai/game_status/timer",    GameTime,  queue_size = 10)

        rospy.Subscriber("/ai/game_status/status", GameStatus, self.on_status)
        self._game_status = Status.STATUS_INIT

        self.timer = TimerManager()

        r = rospy.Rate(5)
        while not rospy.is_shutdown():
            if self.timer.started:
                if self.timer.time_left() <= 0:
                    self.set_game_status(GameStatus.STATUS_HALT)
            self.publish_timer()
            r.sleep()

    def publish_timer(self):
        m = GameTime()
        m.is_active          = self.timer.started
        m.game_time_duration = self.timer.game_duration
        m.game_elapsed_time  = self.timer.elapsed_time() if self.timer.started else -1
        m.game_time_left     = self.timer.time_left()    if self.timer.started else -1
        self._timer_pub.publish(m)

    def on_set_timer(self, req):
        if req.action == req.ACTION_START:
            if self._game_status == Status.STATUS_INGAME:
                self.timer.start(req.duration)
            else:
                rospy.logerr("ERROR Not allowed to start a timer outside game_status STATUS_INGAME.")
                return SetTimerResponse(False)
        elif req.action == req.ACTION_RESET:
            if self.timer.started:
                self.timer.reset()
            else:
                rospy.logerr("ERROR Tried to reset timer but it is not running.")
                return SetTimerResponse(False)
        elif req.action == req.ACTION_STOP:
            if self.timer.started:
                self.timer.stop()
            else:
                rospy.logerr("ERROR Tried to stop timer but it is not running.")
                return SetTimerResponse(False)
        return SetTimerResponse(True)

    def on_status(self, req):
        self._game_status = req.game_status

    def set_game_status(self, status):
        rospy.wait_for_service("/ai/game_status/set_status", timeout = 2)
        service = rospy.ServiceProxy("/ai/game_status/set_status", SetStatus)
        return service(status)


if __name__ == "__main__":
    TimerNode()
