# -*- coding: utf-8 -*-
import time
import rospy
import actionlib

from ai_scheduler.msg import TaskResult
import ai_scheduler.srv

import navigation_navigator.msg
import movement_actuators.msg
import movement_actuators.srv
import memory_map.srv
import ai_game_status.srv
import ai_timer.srv
import drivers_ard_hmi.msg
import drivers_ard_asserv.srv
import drivers_ard_asserv.msg
import drivers_ax12.msg

class RequestTypes(object):
    PUB_MSG = 0
    SUB_MSG = 1
    SERVICE = 2
    ACTION  = 3

    SERVERS = None

    @staticmethod
    def init():
        RequestTypes.SERVERS = {
            "/ai/scheduler/score":               (RequestTypes.PUB_MSG, ai_scheduler.msg.AIScore),
            "/ai/game_status/set_status":        (RequestTypes.SERVICE, ai_game_status.srv.SetStatus),
            "/ai/timer/set_timer":               (RequestTypes.SERVICE, ai_timer.srv.SetTimer),
            "/ai/timer/delay":                   (RequestTypes.SERVICE, ai_timer.srv.Delay),

            "/memory/map/get":                   (RequestTypes.SERVICE, memory_map.srv.MapGet),
            "/memory/map/set":                   (RequestTypes.SERVICE, memory_map.srv.MapSet),
            "/memory/map/transfer":              (RequestTypes.SERVICE, memory_map.srv.MapTransfer),

            "/navigation/navigator/goto_action": (RequestTypes.ACTION,  navigation_navigator.msg.DoGotoAction, navigation_navigator.msg.DoGotoGoal),
            "/navigation/navigator/gotowaypoint_action": (RequestTypes.ACTION,  navigation_navigator.msg.DoGotoWaypointAction, navigation_navigator.msg.DoGotoWaypointGoal),
            "/movement/actuators/dispatch":      (RequestTypes.ACTION,  movement_actuators.msg.DispatchAction, movement_actuators.msg.DispatchGoal),
            "/movement/actuators/barrel":        (RequestTypes.ACTION, movement_actuators.msg.BarrelAction, movement_actuators.msg.BarrelGoal),
            "/movement/actuators/arm":           (RequestTypes.ACTION, movement_actuators.msg.ArmAction, movement_actuators.msg.ArmGoal),
            "/movement/actuators/activate_canon":        (RequestTypes.SERVICE, movement_actuators.srv.ActivateCanon),

            "/drivers/ard_asserv/set_pos":       (RequestTypes.SERVICE, drivers_ard_asserv.srv.SetPos),
            "/drivers/ard_asserv/pwm": (RequestTypes.SERVICE, drivers_ard_asserv.srv.Pwm),
            "/drivers/ard_asserv/goto_action": (RequestTypes.ACTION, drivers_ard_asserv.msg.DoGotoAction, drivers_ard_asserv.msg.DoGotoGoal),
            "/drivers/ax12":                (RequestTypes.ACTION, drivers_ax12.msg.Ax12CommandAction, drivers_ax12.msg.Ax12CommandGoal),

            "/feedback/ard_hmi/ros_event":       (RequestTypes.PUB_MSG, drivers_ard_hmi.msg.ROSEvent),
            "/feedback/ard_hmi/hmi_event":       (RequestTypes.SUB_MSG, drivers_ard_hmi.msg.HMIEvent),

            "/test":                             (RequestTypes.PUB_MSG, TaskResult),
            "/test2":                            (RequestTypes.SUB_MSG, TaskResult) }

    @staticmethod
    def getRequestType(dest):
        return RequestTypes.SERVERS[dest][0]
    @staticmethod
    def getRequestClass(dest):
        return RequestTypes.SERVERS[dest][1]
    @staticmethod
    def getActionGoalClass(dest):
        return RequestTypes.SERVERS[dest][2]

class AICommunication():
    DEFAULT_SUB_MSG_TIMEOUT = 5
    DEFAULT_ACTION_TIMEOUT  = 20

    _sub_msg_success = False

    def __init__(self):
        RequestTypes.init()
        self._cached_publishers = {}

    def SendRequest(self, dest, params, timeout=None, callback=None):
        start_time = time.time()
        if dest in RequestTypes.SERVERS:
            if RequestTypes.getRequestType(dest) == RequestTypes.PUB_MSG:
                response = self._pub_msg(dest, RequestTypes.getRequestClass(dest), params)
            elif RequestTypes.getRequestType(dest) == RequestTypes.SUB_MSG:
                response = self._sub_msg(dest, RequestTypes.getRequestClass(dest), timeout)
            elif RequestTypes.getRequestType(dest) == RequestTypes.SERVICE:
                response = self._send_service(dest, RequestTypes.getRequestClass(dest), params)
            elif RequestTypes.getRequestType(dest) == RequestTypes.ACTION:
                response = self._send_blocking_action(dest, RequestTypes.getRequestClass(dest),
                                                      RequestTypes.getActionGoalClass(dest), params, timeout)
            if callback is not None:
                callback(response, time.time() - start_time)
            else:
                return response
        else:
            raise ValueError, "Message destination '{}' was not recognized. Has it been added to scheduler_communication.py definition dict, or mispelled ?".format(dest)

    def _pub_msg(self, dest, msg_class, params):
        try:
            if dest not in self._cached_publishers:
                rospy.loginfo("Creating and caching new publisher to '{}'.".format(dest))
                self._cached_publishers[dest] = rospy.Publisher(dest, msg_class, queue_size=10)
                time.sleep(0.05)

            pub = self._cached_publishers[dest]
            pub.publish(**params)
            rospy.loginfo("Published to topic '{}'.".format(dest))
            return TaskResult(0, "")
        except Exception as e:
            rospy.logerr("Publishing to topic '{}' failed.".format(dest))
            return TaskResult(2, "scheduler_communication.py could not send message to topic '{}': {}".format(dest, e))

    def _sub_msg(self, dest, msg_class, timeout=None):
        self._sub_msg_success = False
        rospy.Subscriber(dest, msg_class, self._sub_msg_callback)

        if timeout is None:
            timeout = AICommunication.DEFAULT_SUB_MSG_TIMEOUT
        rospy.loginfo("Waiting for message on topic '{}' for {}s...".format(dest, timeout))

        s = time.time()
        while not self._sub_msg_success and (time.time() - s < timeout):
            time.sleep(0.02)
        if self._sub_msg_success:
            rospy.loginfo("Got message from topic '{}'.".format(dest))
        else:
            rospy.logerr("Didn't receive any message from '{}' in {} seconds.".format(dest, timeout))
        return TaskResult(0, "") if self._sub_msg_success else TaskResult(1, "Didn't receive any message in {} seconds.".format(timeout))
    def _sub_msg_callback(self, msg):
        self._sub_msg_success = True

    def _send_service(self, dest, srv_class, params):
        try: # Handle a timeout in case one node doesn't respond
            server_wait_timeout = 2
            rospy.logdebug("Waiting for service %s for %d seconds" % (dest, server_wait_timeout))
            rospy.wait_for_service(dest, timeout=server_wait_timeout)
        except rospy.ROSException:
            res = TaskResult()
            res.result = res.RESULT_FAIL
            res.verbose_reason = "wait_for_service request timeout exceeded."
            return res

        rospy.loginfo("Sending service request to '{}'...".format(dest))
        service = rospy.ServiceProxy(dest, srv_class)
        response = service(srv_class._request_class(**params)) #TODO handle timeout
        if response is not None:
            rospy.loginfo("Got service response from '{}'.".format(dest))
        else:
            rospy.logerr("Service call response from '{}' is null.".format(dest))
        return response

    def _send_blocking_action(self, dest, action_class, goal_class, params, timeout=None):
        client = actionlib.SimpleActionClient(dest, action_class)
        SERVER_WAIT_TIMEOUT = 2
        rospy.loginfo("Waiting for action server on '{}'...".format(dest))
        if client.wait_for_server(timeout = rospy.Duration(SERVER_WAIT_TIMEOUT)):
            rospy.loginfo("Action server available, sending action goal...")
            client.send_goal(goal_class(**params))

            if timeout is None:
                timeout = AICommunication.DEFAULT_ACTION_TIMEOUT

            rospy.loginfo("Waiting for action result with {}s timeout...".format(timeout))
            client.wait_for_result(timeout = rospy.Duration(timeout))
            response =  client.get_result()
            if response is not None:
                rospy.loginfo("Got action result.")
            else: rospy.logerr("Action response timeout reached!")
            return response
        else:
            res = TaskResult()
            res.result = res.RESULT_FAIL
            rospy.logerr("Action failed: wait_for_server action timeout reached.")
            return res
