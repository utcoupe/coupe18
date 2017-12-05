# -*- coding: utf-8 -*-
import time
import rospy

import ai_scheduler.srv
from ai_scheduler.msg import TaskResult

class RequestTypes():
    SERVICE = 0
    ACTION = 1

class AICommunication():
    def SendRequest(self, dest, params, callback):
        servers = {
            "/ai/timer":          (RequestTypes.SERVICE, ai_scheduler.srv.AIGenericCommand),
            "/ai/scheduler":      (RequestTypes.SERVICE, ai_scheduler.srv.AIGenericCommand),
            "/movement/navigator/goto":         (RequestTypes.SERVICE, ai_scheduler.srv.test),
        }
        def getRequestType(dest):
            return servers[dest][0]
        def getRequestClass(dest):
            return servers[dest][1]

        start_time = time.time()
        if dest in servers:
            if getRequestType(dest) == RequestTypes.SERVICE:
                response = self._send_service(dest, getRequestClass(dest), params)
                callback(response, time.time() - start_time)
            elif getRequestType(dest) == RequestTypes.ACTION:
                pass
            else:
                raise NotImplementedError, "Can't send a message of this type (yet ? NotImplemented ?)."
        else:
            raise ValueError, "Message destination '{}' was not recognized. Has it been added to ai_communication.py definition dict, or misspelled ?".format(dest)

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
