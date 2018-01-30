from map_loader import LoadChecker
from shapes import Shape

class Position(object):
    def __init__(self, xml):
        self.x = float(xml.attrib["x"])
        self.y = float(xml.attrib["y"])
        self.a = float(xml.attrib["a"]) if "a" in xml.attrib else None


class Marker(object):
    def __init__(self, xml, super_position=None, super_shape=None):
        self.type        = None
        self.ns          = None
        self.scale       = None
        self.position    = None
        self.orientation = None
        self.color       = None

class Feature(object):
    def __init__(self, xml):
        self.layer = xml.attrib["layer"] if "layer" in xml.attrib else None

        pos = xml.find("position")
        self.position = Position(pos) if pos is not None else None

        shape = xml.find("shape")
        self.shape = Shape(shape) if shape is not None else None

        marker = xml.find("marker")
        self.marker = Marker(marker, self.position, self.shape) if marker is not None else None