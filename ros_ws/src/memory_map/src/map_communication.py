#!/usr/bin/python
import json
import time

import rospy
import memory_map.srv
from map_manager import SetMode, Map, DictManager
from occupancy import OccupancyGenerator


class Servers():
    GET_SERV = "/memory/map/get"
    SET_SERV = "/memory/map/set"
    TRANSFER_SERV = "/memory/map/transfer"
    OCCUPANCY_SERV = "/memory/map/get_occupancy"

class MapServices():
    def __init__(self):
        self.GetSERV       = rospy.Service(Servers.GET_SERV, memory_map.srv.MapGet,                self.on_get)
        self.SetSERV       = rospy.Service(Servers.SET_SERV, memory_map.srv.MapSet,                self.on_set)
        self.TransferSERV  = rospy.Service(Servers.TRANSFER_SERV, memory_map.srv.MapTransfer,      self.on_transfer)
        self.OccupancySERV = rospy.Service(Servers.OCCUPANCY_SERV, memory_map.srv.MapGetOccupancy, self.on_get_occupancy)

    def on_get(self, req):
        s = time.time() * 1000
        rospy.loginfo("GET:" + str(req.request_path))

        success = False
        response = Map.get(req.request_path)
        if isinstance(response, DictManager):
            rospy.logerr("    GET Request failed : '^' dict operator not allowed in services.")
            response = None

        if response != None:
            success = True

        rospy.logdebug("    Responding: " + str(response))
        rospy.logdebug("    Process took {0:.2f}ms".format(time.time() * 1000 - s))
        return memory_map.srv.MapGetResponse(success, json.dumps(response))

    def on_set(self, req):
        s = time.time() * 1000
        rospy.loginfo("SET:" + str(req.request_path))

        success = False
        success = Map.set(req.request_path, req.mode)
        try:
            success = Map.set(req.request_path, req.mode)
        except Exception as e:
            rospy.logerr("    SET Request failed (python reason) : " + str(e))

        rospy.logdebug("    Responding: " + str(success))
        rospy.logdebug("    Process took {0:.2f}ms".format(time.time() * 1000 - s))
        return memory_map.srv.MapSetResponse(success)

    def on_transfer(self, req):
        s = time.time() * 1000
        rospy.loginfo("TRANSFER:{} to {}".format(req.old_path, req.new_path))
        elem, elem_name = Map.get(req.old_path + "/^"), req.old_path.split('/')[-1]
        if elem:
            success = Map.set(req.old_path, SetMode.MODE_DELETE) and \
                      Map.set(req.new_path + "/{}".format(elem_name), SetMode.MODE_ADD, instance = elem)
        else:
            rospy.logerr("    TRANSFER Request failed : could not find the object at old_path '{}'.".format(req.old_path))
            success = False

        rospy.logdebug("    Responding: " + str(success))
        rospy.logdebug("    Process took {0:.2f}ms".format(time.time() * 1000 - s))
        return memory_map.srv.MapTransferResponse(success)

    def on_get_occupancy(self, req):
        s = time.time() * 1000
        rospy.loginfo("GET_OCCUPANCY:" + str(req.layer_name))

        try:
            path = OccupancyGenerator.getImagePath(req.layer_name)
        except Exception as e:
            rospy.logerr("    Request failed : " + str(e))

        rospy.logdebug("    Responding: " + str(path))
        rospy.logdebug("    Process took {0:.2f}ms".format(time.time() * 1000 - s))
        return memory_map.srv.MapGetOccupancyResponse(path)
