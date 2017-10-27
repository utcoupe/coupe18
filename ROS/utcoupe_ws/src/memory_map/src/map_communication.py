#!/usr/bin/python
import rospy
from MapManager import Map
import memory_map.srv


class Servers():
    CONDITIONS_CONTAINERS_SERV = "/memory/map/conditions/containers"
    GET_SERV = "/memory/map/get"
    SET_SERV = "/memory/map/set"


class ConditionsHandler():
    def __init__(self):
        self.ContainersSERV = rospy.Service(Servers.CONDITIONS_CONTAINERS_SERV,
                                            memory_map.srv.MapConditionContainer,
                                            self.serv_handler_on_container_condition)
        rospy.loginfo("created conditions server")

    def serv_handler_on_container_condition(self, req):
        if req.condition_type == memory_map.srv.MapConditionContainer.CONDITION_LT_NUMBER:
            result = True
        elif req.condition_type == memory_map.srv.MapConditionContainer.CONDITION_MT_NUMBER:
            result = True
        elif req.condition_type == memory_map.srv.MapConditionContainer.CONDITION_LT_CONTAINER:
            result = True
        elif req.condition_type == memory_map.srv.MapConditionContainer.CONDITION_MT_CONTAINER:
            result = True
        return memory_map.srv.MapConditionContainerResponse(result)


class GetServiceHandler():
    def __init__(self):
        self.GetSERV = rospy.Service(Servers.GET_SERV, memory_map.srv.MapGet2, self.on_get)

    def on_get(self, req):
        print "Got get request ! responding : " + str(Map.get(req.request_path))



class SetServiceHandler():
    def __init__(self):
        self.SetSERV = rospy.Service(Servers.SET_SERV, memory_map.srv.MapSet, self.on_set)

    def on_set(self, req):
        pass
