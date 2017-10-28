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

    def transform(self, add_x, add_y, add_a = 0.0):
        '''
        Returns a new Position object with the applied transformation.
        '''
        p = Position({"x": self.x + add_x, "y": self.y +
                      add_y, "type": self.CollisionType})
        if self.has_angle:
            p.a = self.a + add_a
            self.has_angle = True
        return p

    def angle(self):
        return self.a if self.has_angle else 0.0

    def tuple2(self):
        return (self.x, self.y)

    def tuple3(self):
        return (self.x, self.y, self.a) if self.has_angle else None


class Shape():
    def __init__(self, initdict):
        self.Type = initdict["type"]
        if "viz_color" in initdict:
            self.viz_color = Color(initdict["viz_color"])

        if self.Type == "rect":
            self.width = initdict["width"]
            self.height = initdict["height"]
            self.points = [(0, 0), (self.width, 0), (self.width, self.height), (0, self.height)]
        elif self.Type == "circle":
            self.radius = initdict["radius"]
        elif self.Type == "polygon":
            self.points = initdict["points"]
        elif self.Type == "line":
            self.start = initdict["start"]
            self.end = initdict["end"]
        else:
            raise ValueError("'{}' shape type not recognized or supported.".format(self.Type))

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

    def rotated(self, theta):
        """Rotates the given polygon which consists of corners represented as (x,y),
        around the ORIGIN, clock-wise, theta degrees"""
        if theta == 0:
            return self.points
        if self.Type not in ["rect", "polygon", "polyline"]:
            raise TypeError(
                "This shape ({0}) cannot be rotated.".format(self.Type))

        rotatedPolygon = []
        for corner in self.points:
            rotatedPolygon.append((corner[0] * math.cos(theta) - corner[1] * math.sin(theta),
                                   corner[0] * math.sin(theta) + corner[1] * math.cos(theta)))
        return rotatedPolygon

    def inflate(self, offset):
        # returns a new shape bigger/smaller to the original one, given the offset amount.
        if offset == 0:
            return self

        if self.Type == "circle":
            return Shape({"type": "circle", "radius": self.radius + offset})
        elif self.Type == "rect":
            return Shape({"type": "rect", "width": self.width + offset, "height": self.height + offset})
        elif self.Type == "polygon":
            clipper = pyclipper.PyclipperOffset()
            clipper.AddPath(self.points, pyclipper.JT_MITER,
                            pyclipper.ET_CLOSEDPOLYGON)
            solution = clipper.Execute(offset)[0]
            return Shape({"type": "polygon", "points": solution})
        else:
            raise TypeError("Can't inflate this shape type.")


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


class Trajectory():
    def __init__(self, initdict):
        pass


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
