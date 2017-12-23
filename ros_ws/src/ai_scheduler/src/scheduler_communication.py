# -*- coding: utf-8 -*-
import time
import rospy
import actionlib

from ai_scheduler.msg import TaskResult
import ai_scheduler.srv

import navigation_navigator.msg
import memory_map.srv

class RequestTypes():
    PUB_MSG = 0
    SUB_MSH = 1
    SERVICE = 2
    ACTION  = 3

class AICommunication():
    def SendRequest(self, dest, params, callback):
        servers = {
            "/ai/timer":                        (RequestTypes.SERVICE, ai_scheduler.srv.AIGenericCommand),
            "/ai/scheduler":                    (RequestTypes.SERVICE, ai_scheduler.srv.AIGenericCommand),
            "/navigation/navigator/dogoto":     (RequestTypes.ACTION,  navigation_navigator.msg.DoGotoAction, navigation_navigator.msg.DoGotoGoal),
            "/test":                            (RequestTypes.PUB_MSG, TaskResult),
            "/memory/map/transfer":             (RequestTypes.SERVICE, memory_map.srv.MapTransfer)
        }
        def getRequestType(dest):
            return servers[dest][0]
        def getRequestClass(dest):
            return servers[dest][1]
        def getActionGoalClass(dest):
            return servers[dest][2]

        start_time = time.time()
        if dest in servers:
            if getRequestType(dest) == RequestTypes.PUB_MSG:
                response = self._pub_msg(dest, getRequestClass(dest), params)
            elif getRequestType(dest) == RequestTypes.SERVICE:
                response = self._send_service(dest, getRequestClass(dest), params)
            elif getRequestType(dest) == RequestTypes.ACTION:
                response = self._send_blocking_action(dest, getRequestClass(dest), getActionGoalClass(dest), params)
            # response = TaskResult() # DEBUG Force success response
            # response.result = response.RESULT_SUCCESS if bool(input("success ?")) else response.RESULT_FAIL
            callback(response, time.time() - start_time)
        else:
            raise ValueError, "Message destination '{}' was not recognized. Has it been added to ai_communication.py definition dict, or mispelled ?".format(dest)

    def _pub_msg(self, dest, msg_class, params):
        try:
            pub = rospy.Publisher(dest, msg_class, queue_size=10)
            pub.publish(**params)
            return TaskResult(0, "")
        except Exception as e:
            return TaskResult(2, "ai_communication.py could not send message to topic '{}': {}".format(dest, e))

    def _send_service(self, dest, srv_class, params):
        try: # Handle a timeout in case one node doesn't respond
            rospy.wait_for_service(dest, timeout = 2)
        except rospy.ROSException:
            res = TaskResult()
            res.result = res.RESULT_FAIL
            res.verbose_reason = "wait_for_service request timeout exceeded."
            return res
        service = rospy.ServiceProxy(dest, srv_class)
        return service(srv_class._request_class(**params))

    def _send_blocking_action(self, dest, action_class, goal_class, params):
        client = actionlib.SimpleActionClient(dest, action_class)
        try: # Handle a timeout in case one node doesn't respond
            client.wait_for_server(timeout = rospy.Duration(2.0))
            client.send_goal(goal_class(**params))
            client.wait_for_result()
            return client.get_result()
        except:
            res = TaskResult()
            res.result = res.RESULT_FAIL
            res.verbose_reason = "wait_for_service request timeout exceeded."
            return res
