#!/usr/bin/python
import rospy
from MapManager import Map, MapPath
import memory_map.srv


class Servers():
    CONDITIONS_CONTAINERS_SERV = "/memory/map/conditions/containers"
    GET_SERV = "/memory/map/get"
    SET_SERV = "/memory/map/set"


class GetServiceHandler():
    def __init__(self):
        self.GetSERV = rospy.Service(Servers.GET_SERV, memory_map.srv.MapGetFromPath, self.on_get)

    def on_get(self, req):
        rospy.loginfo("GET:" + str(req.request_path))
        parsed_path = MapPath(req.request_path)
        response = Map.get(parsed_path)
        print "GET Response : " + str(response)
        return None



class SetServiceHandler():
    def __init__(self):
        self.SetSERV = rospy.Service(Servers.SET_SERV, memory_map.srv.MapSet, self.on_set)

    def on_set(self, req):
        pass


class ConditionsHandler():
    def __init__(self):
        self.ContainersSERV = rospy.Service(Servers.CONDITIONS_CONTAINERS_SERV,
                                            memory_map.srv.MapConditionContainer,
                                            self.serv_handler_on_container_condition)

    def serv_handler_on_container_condition(self, req):
        result = None
        return memory_map.srv.MapConditionContainerResponse(result)