import math
import pyclipper  # pip install pyclipper
import rospy
from loader import LoadingHelpers
from visualization_msgs.msg import Marker


'''
LOW LEVEL DEFINITION CLASSES
'''


class Position():
    '''
    TODO: basic class only
    Fully describes a 2D position in the map. The position can acquire a reference name.
    Can be of type XY or XYA. The ROS tf frame_id is necessary.
    '''
    def __init__(self, initdict):
        LoadingHelpers.checkKeysExist(initdict, "frame_id", "x", "y", "type")
        self.frame_id = initdict["frame_id"]
        self.x = float(initdict["x"])
        self.y = float(initdict["y"])

        if "a" in initdict.keys():
            self.a = float(initdict["a"])
            self.has_angle = True
        else:
            self.has_angle = False

        self.CollisionType = initdict["type"]

    def get(self):
        response_dict = {
            "frame_id": self.frame_id,
            "x": self.x,
            "y": self.y,
            "has_angle": self.has_angle,
            "type": self.CollisionType
        }
        if self.has_angle:
            response_dict["a"] = self.a
        return response_dict

    def set(self, new_value_dict):
        if "x" in new_value_dict:
            self.x = float(new_value_dict["x"])
        if "y" in new_value_dict:
            self.x = float(new_value_dict["y"])
        if "frame_id" in new_value_dict:
            self.frame_id = float(new_value_dict["frame_id"])
        if "type" in new_value_dict:
            self.CollisionType = new_value_dict["type"]
        if "has_angle" in new_value_dict:
            self.has_angle = bool(new_value_dict["has_angle"])
            if self.has_angle:
                if not "a" in new_value_dict:
                    raise ValueError("Missing 'a' field because 'has_angle' was set to true.")
                self.a = new_value_dict["a"]
            else:
                self.a = None
        return True


class Shape():
    def __init__(self, initdict):
        LoadingHelpers.checkKeysExist(initdict, "type")
        self.Type = initdict["type"]
        if "viz_color" in initdict:
            self.viz_color = Color(initdict["viz_color"])

        if self.Type == "rect":
            self.width = initdict["width"]
            self.height = initdict["height"]
            self.generateRectPoints()
        elif self.Type == "circle":
            self.radius = initdict["radius"]
        elif self.Type == "polygon":
            self.points = initdict["points"]
        elif self.Type == "line":
            self.start = initdict["start"]
            self.end = initdict["end"]
        else:
            raise ValueError("'{}' shape type not recognized or supported.".format(self.Type))

    def generateRectPoints(self):
        self.points = [(0, 0), (self.width, 0), (self.width, self.height), (0, self.height)]

    def get(self):
        response_dict = {
            "type": self.Type
        }
        if self.Type == "rect":
            response_dict["width"] = self.width
            response_dict["height"] = self.height
            response_dict["points"] = self.points
        elif self.Type == "circle":
            response_dict["radius"] = self.radius
        elif self.Type == "polygon":
            response_dict["points"] = self.points
        elif self.Type == "line":
            response_dict["start"] = self.end
            response_dict["end"] = self.end
        return response_dict

    def set(self, new_value_dict):

        LoadingHelpers.checkKeysExist(new_value_dict, "type")
        if new_value_dict["type"] != self.Type:
            rospy.logerr("SET Trying to change a shape type to another. Can't dynamically change shape type (NotImplemented)")
            return False
        else:
            if new_value_dict["type"] == "rect":
                if "width" in new_value_dict:
                    self.width = float(new_value_dict["width"])
                    return True
                if "height" in new_value_dict:
                    self.height = float(new_value_dict["height"])
                    return True
                self.generateRectPoints()
            elif new_value_dict["type"] == "circle":
                if "radius" in new_value_dict:
                    self.radius = float(new_value_dict["radius"])
                    return True
                else: return False
            elif new_value_dict["type"] == "polygon":
                if "radius" in new_value_dict:
                    self.points = new_value_dict["points"]
                    return True
                else: return False
            elif new_value_dict["type"] == "line":
                if "start" in new_value_dict:
                    self.start = float(new_value_dict["start"])
                    return True
                if "end" in new_value_dict:
                    self.end = float(new_value_dict["end"])
                    return True

class Visual():
    def __init__(self, initdict):
        LoadingHelpers.checkKeysExist(initdict, "ns", "id", "type", "z", "scale", "orientation", "color")
        self.Dict = initdict
        self.NS = initdict["ns"]
        self.ID = int(initdict["id"])
        markerType = {
            "cube": Marker.CUBE,
            "sphere": Marker.SPHERE,
            "mesh": Marker.MESH_RESOURCE
        }
        self.Type = markerType[initdict["type"]]
        self.z = float(initdict["z"])
        self.Scale = initdict["scale"]
        self.Orientation = initdict["orientation"]
        self.Color = initdict["color"]

        if initdict["type"] == "mesh":
            LoadingHelpers.checkKeysExist(initdict, "mesh_path")
            self.mesh_path = initdict["mesh_path"]

    def get(self):
        return self.Dict

    def set(self, new_value_dict):
        self.__init__(new_value_dict)
        return True


class Trajectory():
    def __init__(self, initdict):
        pass

    def get(self):
        rospy.logerr("Trajectories not implemented yet.")
        raise NotImplementedError

    def set(self, new_value_dict):
        rospy.logerr("Trajectories not implemented yet.")
        raise NotImplementedError


class Color():
    def __init__(self, color):
        self.RGB = tuple(color)

        if color == "blue":
            self.RGB = (50, 50, 255)
        if color == "yellow":
            self.RGB = (255, 246, 0)
        if color == "gray":
            self.RGB = (175, 175, 175)

        self.R = self.RGB[0]
        self.G = self.RGB[1]
        self.B = self.RGB[2]

    def textColor(self):
        # https://stackoverflow.com/questions/1855884/determine-font-color-based-on-background-color
        lum = 1 - (0.299 * self.R + 0.587 * self.G + 0.114 * self.B) / 255
        return (255, 255, 255) if lum > 0.5 else (0, 0, 0)
