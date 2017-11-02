#!/usr/bin/python
import json
import time

import rospy
import memory_map.srv
from MapManager import Map, map_filters


class Servers():
    CONDITIONS_CONTAINERS_SERV = "/memory/map/conditions/containers"
    GET_SERV = "/memory/map/get"
    SET_SERV = "/memory/map/set"


class GetServiceHandler():
    def __init__(self):
        self.GetSERV = rospy.Service(Servers.GET_SERV, memory_map.srv.MapGetFromPath, self.on_get)

    def on_get(self, req):
        s = time.time() * 1000
        rospy.loginfo("GET:" + str(req.request_path))

        parsed_path = map_filters.MapPath(req.request_path)
        response = Map.get(parsed_path)

        rospy.logdebug("    Responding: " + str(response))
        rospy.logdebug("    Process took {0:.2f}ms".format(time.time() * 1000 - s))
        return memory_map.srv.MapGetFromPathResponse(json.dumps(response))


class SetServiceHandler():
    def __init__(self):
        self.SetSERV = rospy.Service(Servers.SET_SERV, memory_map.srv.MapSet, self.on_set)

    def on_set(self, req):
        s = time.time() * 1000
        rospy.loginfo("SET:" + str(req.request_path))

        path = map_filters.MapPath(req.request_path)
        success = Map.set(path, json.loads(req.new_value_dict))

        rospy.logdebug("    Responding: " + str(success))
        rospy.logdebug("    Process took {0:.2f}ms".format(time.time() * 1000 - s))
        return memory_map.srv.MapSetResponse(success)


class ConditionsHandler():
    def __init__(self):
        self.ContainersSERV = rospy.Service(Servers.CONDITIONS_CONTAINERS_SERV,
                                            memory_map.srv.MapConditionContainer,
                                            self.on_condition)

    def on_condition(self, req):
        result = None
        return memory_map.srv.MapConditionContainerResponse(result)
