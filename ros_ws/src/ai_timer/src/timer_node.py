#!/usr/bin/python
import time
import rospy

from ai_game_status.msg import GameStatus
from ai_game_status.srv import SetStatus
from ai_game_status import StatusServices

from ai_timer.srv import SetTimer, SetTimerResponse, Delay, DelayResponse
from ai_timer.msg import GameTime


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
        rospy.loginfo("Starting {} seconds timer.".format(duration))
        self.reset()
        self.game_duration = duration
        self.started = True

    def stop(self):
        self.__init__()

    def elapsed_time(self):
        return (time.time() * 1000 - self._start_time) / 1000.0 # return elapsed time in seconds

    def time_left(self):
        return self.game_duration - self.elapsed_time()

    def is_finished(self):
        return True if self.time_left() < 0 else False


class TimerNode():
    def __init__(self):
        rospy.init_node("timer", log_level=rospy.INFO)
        self._set_timer_srv = rospy.Service("/ai/timer/set_timer", SetTimer,  self.on_set_timer)
        self._delay_srv     = rospy.Service("/ai/timer/delay",     Delay,  self.on_delay)
        self._timer_pub     = rospy.Publisher("/ai/timer/time",    GameTime,  queue_size = 10)

        rospy.Subscriber("/ai/game_status/status", GameStatus, self.on_status)
        self._game_status = Status.STATUS_INIT

        self.timer = TimerManager()

        # Tell ai/game_status the node initialized successfuly.
        StatusServices("ai", "timer").ready(True)

        r = rospy.Rate(5)
        while not rospy.is_shutdown():
            if self.timer.started:
                if self.timer.time_left() <= 0:
                    rospy.logwarn_throttle(20, "Timer ended! setting HALT to ai/game_status, stopping timer.")
                    self.set_game_status(GameStatus.STATUS_HALT)
                    self.timer.stop()
                else:
                    rospy.loginfo_throttle(20, "Timer seconds left : {}".format(int(round(self.timer.time_left()))) + " s")
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
            if not self.timer.started:
                self.timer.start(req.duration)
            else:
                rospy.logerr("ERROR Timer already started.")
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

    def on_delay(self, req):
        rospy.loginfo("[timer] Sleeping for {}s (service callback)...".format(req.duration))
        time.sleep(req.duration)
        return DelayResponse(True)

    def on_status(self, req):
        self._game_status = req.game_status

    def set_game_status(self, status):
        rospy.wait_for_service("/ai/game_status/set_status", timeout = 2)
        service = rospy.ServiceProxy("/ai/game_status/set_status", SetStatus)
        return service(status)


if __name__ == "__main__":
    TimerNode()
