# -*- coding: utf-8 -*-
import rospy
import ai_scheduler.srv

class RequestTypes():
    SERVICE = 0
    ACTION = 1

class AICommunication():
    def SendRequest(self, dest, params, callback):
        servers = {
            "/ai/timer":          (RequestTypes.SERVICE, ai_scheduler.srv.AIGenericCommand),
            "/ai/scheduler":      (RequestTypes.SERVICE, ai_scheduler.srv.AIGenericCommand),
            "/test":         (RequestTypes.SERVICE, ai_scheduler.srv.test)
        }
        def getRequestType(dest):
            return servers[dest][0]
        def getRequestClass(dest):
            return servers[dest][1]

        if dest in servers:
            if getRequestType(dest) == RequestTypes.SERVICE:
                response = self.sendService(dest, getRequestClass(dest), params)
                callback(response)
                return response
            elif getRequestType(dest) == RequestTypes.ACTION:
                pass
            else:
                raise NotImplementedError, "Can't send a message of this type (yet ? NotImplemented ?)."
        else:
            raise ValueError, "Message destination '{}' was not recognized. Has it been added to ai_conditions.py definition dict, or misspelled ?".format(dest)

    def sendService(self, dest, srv_class, params):
        rospy.wait_for_service(dest)
        service = rospy.ServiceProxy(dest, srv_class)
        return service(srv_class._request_class(**params))
