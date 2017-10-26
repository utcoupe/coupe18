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
        return  # TODO: adapt
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

        success, reason = 200, ""
        if not result: success, reason = 500, "Invalid path on get request."
        rospy.loginfo("GET: responding '{}'.".format(result))
        
        return memory_map_simple.srv.MapGetResponse(res_code = success, reason = reason,
                                                    res_json = json.dumps(result))



class SetServiceHandler():
    def __init__(self):
        self.SetSERV = rospy.Service(Servers.SET_SERV, memory_map_simple.srv.MapSet, self.on_set)

    def on_set(self, req):
        success, reason = Map.set(req.path, req.json_new_value)
        rospy.loginfo("SET: responding code {} (reason : '{}').".format(success, reason))
        return memory_map_simple.srv.MapSetResponse(success, reason)
