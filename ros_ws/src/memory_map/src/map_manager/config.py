from map_loader import LoadChecker
from feature import Feature


class Layer(object):
    def __init__(self, xml):
        LoadChecker.checkAttribsExist(xml, "name")
        self.name = xml.attrib["name"]

        self.includes = []
        for include in xml.findall("include"):
            self.includes.append(include.attrib["name"])


class Color(object):
    def __init__(self, xml):
        LoadChecker.checkAttribsExist(xml, "name", "r", "g", "b")
        self.name = xml.attrib["name"]
        self.r = xml.attrib["r"]
        self.g = xml.attrib["g"]
        self.b = xml.attrib["b"]


class Config(object):
    CURRENT_TEAM = None

    LAYERS = []
    COLORS = []
    MARKERS = []

    @staticmethod
    def load(xml):
        LoadChecker.checkNodesExist(xml, "map", "layers", "colors")

        for layer in xml.find("layers").findall("layer"):
            Config.LAYERS.append(Layer(layer))

        for color in xml.find("colors").findall("color"):
            Config.COLORS.append(Color(color))
