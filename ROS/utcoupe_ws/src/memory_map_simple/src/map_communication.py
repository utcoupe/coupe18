#!/usr/bin/python
import json
import rospy
from map import Map
import memory_map_simple.srv


class Servers():
    CONDITIONS_CONTAINERS_SERV = "/memory/map/conditions/containers"
    GET_SERV = "/memory/map/get"
    SET_SERV = "/memory/map/set"


class ConditionsHandler():
    def __init__(self):
        return
        self.ContainersSERV = rospy.Service(Servers.CONDITIONS_CONTAINERS_SERV,
                                            memory_map_simple.srv.MapConditionContainer,
                                            self.serv_handler_on_container_condition)

    def serv_handler_on_container_condition(self, req):
        if req.condition_type == memory_map_simple.srv.MapConditionContainer.CONDITION_LT_NUMBER:
            result = True
        elif req.condition_type == memory_map_simple.srv.MapConditionContainer.CONDITION_MT_NUMBER:
            result = True
        elif req.condition_type == memory_map_simple.srv.MapConditionContainer.CONDITION_LT_CONTAINER:
            result = True
        elif req.condition_type == memory_map_simple.srv.MapConditionContainer.CONDITION_MT_CONTAINER:
            result = True
        return memory_map_simple.srv.MapConditionContainerResponse(result)


class GetServiceHandler():
    def __init__(self):
        self.GetSERV = rospy.Service(Servers.GET_SERV, memory_map_simple.srv.MapGet, self.on_get)

    def on_get(self, req):
        result = Map.get(req.request_path)
        print "Got get request ! responding : " + str(result)
        return memory_map_simple.srv.MapGetResponse(res_type = memory_map_simple.srv.MapGetResponse.RESPONSE_TYPE_JSON,
                                                    res_json = json.dumps(result))



class SetServiceHandler():
    def __init__(self):
        return
        self.SetSERV = rospy.Service(Servers.SET_SERV, memory_map_simple.srv.MapSet, self.on_set)

    def on_set(self, req):
        pass
