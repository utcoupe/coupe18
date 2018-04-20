#!/usr/bin/python
import time
import rospy

from ai_game_status.msg import GameStatus, NodesStatus
from ai_game_status.srv import SetStatus, SetStatusResponse, NodeReady, NodeReadyResponse


class Status():
    STATUS_INIT   = 0 # All nodes initializing, didn't respond yet.
    STATUS_INGAME = 1 # Scheduler started, doing its job.
    STATUS_HALT   = 2 # Robot stopped (game end, critical HALT requested by a node...)

    INIT_INITIALIZING = 0 # All nodes didn't respond and we didn't reach the init timeout yet.
    INIT_INITIALIZED  = 1 # All nodes responded successfully and are initialized.
    INIT_FAILED       = 2 # Nodes responded false or didn't respond after before the init timeout.

    INIT_CHECKLIST = {  # Please comment the lines instead of deleting them.
        "/ai/scheduler": None,
        "/ai/timer": None,

        "/memory/map": None,
        "/memory/definitions": None,

        "/navigation/navigator": None,
        "/navigation/pathfinder": None,
        "/navigation/collisions": None,

        "/movement/actuators": None,

        "/recognition/localizer": None,
        "/recognition/enemy_tracker": None,
        # "/recognition/cube_finder": None,
        # "/recognition/cp_recognizer": None,
        "/recognition/objects_classifier": None,

        "/processing/belt_interpreter": None,
        # "/processing/lidar_objects": None,

        "/drivers/ard_asserv": None,
        #"/drivers/ard_hmi": None,
        #"/drivers/ard_others": None,
        "/drivers/port_finder": None,
        "/drivers/ax12": None,
        "/drivers/teraranger": None,
    }


class GameStatusNode():
    INIT_TIMEOUT = 40 # seconds to wait for the nodes to send their init response before timeout.

    def __init__(self):
        rospy.init_node("game_status", log_level=rospy.INFO)
        self._node_ready_notif = rospy.Service("/ai/game_status/node_ready", NodeReady, self.on_node_ready)
        self._set_status_srv   = rospy.Service("/ai/game_status/set_status", SetStatus, self.on_set_status)
        self._game_status_pub  = rospy.Publisher("/ai/game_status/status", GameStatus,        queue_size = 10)
        self._nodes_status_pub = rospy.Publisher("/ai/game_status/nodes_status", NodesStatus, queue_size = 10)

        self.game_status = Status.STATUS_INIT
        self.init_status = Status.INIT_INITIALIZING

        self._init_start_time = time.time()
        r = rospy.Rate(5)
        while not rospy.is_shutdown():
            if self.init_status == Status.INIT_INITIALIZING:
                self.check_init_checklist()
                if time.time() - self._init_start_time > GameStatusNode.INIT_TIMEOUT:
                    if len([n for n in Status.INIT_CHECKLIST if Status.INIT_CHECKLIST[n] in [None, False]]) > 0:
                        self.set_init_status(Status.INIT_FAILED)
                    else:
                        self.set_init_status(Status.INIT_INITIALIZED)
                        rospy.loginfo("All nodes initialized successfully, ready to start !")
            self.publish_statuses() # publish game status at 5Hz.

            r.sleep()

    def set_init_status(self, new_status):
        if new_status == Status.INIT_INITIALIZING:
            rospy.loginfo("game_status set to INITIALIZING.")
        elif new_status == Status.INIT_INITIALIZED:
            rospy.loginfo("All nodes initialized successfully, ready to start !")
        elif new_status == Status.INIT_FAILED:
            rospy.logerr("Init timeout reached, certain nodes failed ({}) or didn't respond ({}) ! "
                         "System will continue, but be careful..".format(
                         str([n for n in Status.INIT_CHECKLIST if Status.INIT_CHECKLIST[n] == False]),
                         str([n for n in Status.INIT_CHECKLIST if Status.INIT_CHECKLIST[n] == None])))
        self.init_status = new_status

    def publish_statuses(self):
        m = GameStatus()
        m.game_status = self.game_status
        m.init_status = self.init_status
        self._game_status_pub.publish(m)

        m = NodesStatus()
        for node in Status.INIT_CHECKLIST:
            if Status.INIT_CHECKLIST[node] is None:
                m.pending_nodes.append(node)
            elif Status.INIT_CHECKLIST[node] is True:
                m.ready_nodes.append(node)
            elif Status.INIT_CHECKLIST[node] is False:
                m.failed_nodes.append(node)
        self._nodes_status_pub.publish(m)

    def check_init_checklist(self):
        for node in Status.INIT_CHECKLIST:
            if Status.INIT_CHECKLIST[node] in [None, False]:
                return
        if self.init_status == Status.INIT_INITIALIZING:
            self.set_init_status(Status.INIT_INITIALIZED)
        else:
            rospy.logerr("Unexpected behaviour : init_status checklist got full but status is not INITIALIZING.")

    def on_node_ready(self, msg):
        if msg.node_name in Status.INIT_CHECKLIST:
            Status.INIT_CHECKLIST[msg.node_name] = msg.success
            return NodeReadyResponse()
        else:
            rospy.logwarn("Node name '{}' not in ai/game_status init checklist, passing.".format(msg.node_name))
            return NodeReadyResponse()

    def on_set_status(self, req):
        self.game_status = req.new_game_status
        return SetStatusResponse(True)

if __name__ == "__main__":
    GameStatusNode()
