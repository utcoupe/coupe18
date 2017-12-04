#!/usr/bin/python
import rospy

from timer import GameTimer

from ai_game_status.msg import GameStatus, GameTime
from ai_game_status.srv import SetStatus, SetTimer, NodeReady


class Status():
    STATUS_INITIALIZING     = 0 # All nodes initializing, didn't respond yet.
    STATUS_INITIALIZED      = 1 # All nodes initialized, ready to start.
    # STATUS_NOT_INITIALIZED  = 1 # One node or more responded false init, go at your own risk.
    STATUS_INGAME           = 2 # Scheduler started, doing its job.
    STATUS_HALT             = 3 # Robot stopped (game end, critical HALT requested by a node...)

    INIT_CHECKLIST = {  # Please comment the lines instead of deleting them.
        "/ai/scheduler": None,
        "/ai/scripts": None,
        #"/ai/game_status": None,

        "/memory/map": None,
        "/memory/definitions": None,

        "/navigation/navigator": None,
        "/navigation/pathfinder": None,
        "/navigation/collisions": None,

        "/movement/actuators": None,

        "/recognition/localizer": None,
        "/recognition/enemy_tracker": None,
        "/recognition/cube_finder": None,
        "/recognition/cp_recognizer": None,

        "/processing/belt_interpreter": None,
        "/processing/lidar_objects": None,

        "/feedback/webclient": None
    }


class GameStatusNode():
    def __init__(self):
        rospy.init_node("game_status", log_level=rospy.DEBUG)
        self._node_ready_notif = rospy.Service("/ai/game_status/node_ready", NodeReady, self.on_node_ready)
        self._set_status_srv   = rospy.Service("/ai/game_status/set_status", SetStatus, self.on_set_status)
        self._set_timer_srv    = rospy.Service("/ai/game_status/set_timer",  SetTimer,  self.on_set_timer)

        self._status_pub = rospy.Publisher("/ai/game_status/status", GameStatus, queue_size = 10)
        self._timer_pub  = rospy.Publisher("/ai/game_status/timer",  GameTime,   queue_size = 10)

        duration = 100 # TODO get from def file
        self._timer = GameTimer(duration)

        self.game_status = Status.STATUS_INITIALIZING

        r = rospy.Rate(5)
        while not rospy.is_shutdown():
            if self.game_status == Status.STATUS_INITIALIZING:
                self.check_init_checklist()
            self._status_pub.publish(self.game_status) # publish game status at 5Hz.
            self.publish_timer() # publish timer times at 5Hz.

            r.sleep()

    def publish_timer(self):
        m = GameTime()
        m.game_duration     = self._timer.game_duration
        m.game_elapsed_time = self._timer.elapsed_time()
        m.game_time_left    = self._timer.time_left()
        m.is_active         = self._timer.started
        self._timer_pub.publish(m)

    def check_init_checklist(self):
        for node_status in Status.INIT_CHECKLIST:
            if node_status in [None, False]:
                return
        if self.game_status == Status.STATUS_INITIALIZING:
            self.game_status = Status.STATUS_INITIALIZED
        else:
            rospy.logerr("Unexpected behaviour : game_status checklist got full but status is not INITIALIZING.")

    def on_node_ready(self, msg):
        if msg.node_name in Status.INIT_CHECKLIST:
            Status.INIT_CHECKLIST[msg.node_name] = msg.success
        else:
            rospy.logwarn("Node name '{}' not in ai/game_status init checklist, passing.".format(msg.node_name))

    def on_set_status(self, req):
        self.game_status = req.new_game_status

    def on_set_timer(self, req):
        if req.action == req.ACTION_START:
            self._timer.start()
        # elif req.action == req.ACTION_PAUSE: # TODO Implement.
        #     self._timer.pause()
        # elif req.action == req.ACTION_RESUME:
        #     self._timer.resume()
        elif req.action == req.ACTION_RESET:
            self._timer.reset()
        elif req.action == req.ACTION_STOP:
            self._timer.stop()


if __name__ == "__main__":
    GameStatusNode()
