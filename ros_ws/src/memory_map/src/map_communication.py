#!/usr/bin/python
import json
import time

import rospy
import memory_map.msg
import memory_map.srv
from map_manager import SetMode, Map, DictManager
from occupancy import OccupancyGenerator


class Servers():
    GET_SERV        = "/memory/map/get"
    SET_SERV        = "/memory/map/set"
    TRANSFER_SERV   = "/memory/map/transfer"
    OCCUPANCY_SERV  = "/memory/map/get_occupancy"
    FILLWP_SERV     = "/memory/map/fill_waypoint"

class MapServices():
    def __init__(self, occupancy_generator):
        self.GetSERV       = rospy.Service(Servers.GET_SERV, memory_map.srv.MapGet,                self.on_get)
        self.SetSERV       = rospy.Service(Servers.SET_SERV, memory_map.srv.MapSet,                self.on_set)
        self.TransferSERV  = rospy.Service(Servers.TRANSFER_SERV, memory_map.srv.MapTransfer,      self.on_transfer)
        self.OccupancySERV = rospy.Service(Servers.OCCUPANCY_SERV, memory_map.srv.MapGetOccupancy, self.on_get_occupancy)
        self.FillWPSERV    = rospy.Service(Servers.FILLWP_SERV, memory_map.srv.FillWaypoint,       self.on_fill_waypoint)
        self.occupancy_generator = occupancy_generator

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
            path = self.occupancy_generator.generateLayer(Map, req.layer_name, req.margin)
        except Exception as e:
            rospy.logerr("    Request failed : " + str(e))

        rospy.logdebug("    Responding: " + str(path))
        rospy.logdebug("    Process took {0:.2f}ms".format(time.time() * 1000 - s))
        return memory_map.srv.MapGetOccupancyResponse(path)

    def on_fill_waypoint(self, req):
        s = time.time() * 1000
        rospy.loginfo("FILL_WAYPOINT:{} ({}, {})".format(str(req.waypoint.name if req.waypoint.name else "<no name>"), req.waypoint.x, req.waypoint.y))

        try:
            filled_waypoint, filled_waypoint_name = None, None
            if req.waypoint.name != "" and (req.waypoint.x == 0 and req.waypoint.y == 0):
                filled_waypoint = Map.get("/waypoints/{}/^".format(req.waypoint.name))
                filled_waypoint_name = req.waypoint.name
            elif req.waypoint.x != 0 and req.waypoint.y != 0:
                waypoints = Map.get("/waypoints/^").Dict
                for w in waypoints:
                    if waypoints[w].get("position/x") == round(req.waypoint.x, 3) and waypoints[w].get("position/y") == round(req.waypoint.y, 3):
                        filled_waypoint = waypoints[w]
                        filled_waypoint_name = w
                if not filled_waypoint:
                    rospy.logerr("    Request failed : could not find waypoint that corresponds to the given coordinates.")
            else:
                rospy.logerr("    Request failed : could not find what needs to be filled. EITHER name or X/Y needs to be filled.")
        except Exception as e:
            rospy.logerr("    Request failed : " + str(e))

        success, w = False, None
        if filled_waypoint_name and filled_waypoint:
            success = True
            w = memory_map.msg.Waypoint()
            w.name = filled_waypoint_name
            w.x, w.y = filled_waypoint.get("position/x"), filled_waypoint.get("position/y")
            rospy.logdebug("    Responding:{} ({}, {})".format(filled_waypoint_name, filled_waypoint.get("position/x"), filled_waypoint.get("position/y")))

        rospy.logdebug("    Responding:error")
        rospy.logdebug("    Process took {0:.2f}ms".format(time.time() * 1000 - s))
        return memory_map.srv.FillWaypointResponse(success, w)
