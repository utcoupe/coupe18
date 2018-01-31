from feature import Feature
from map_loader import LoadChecker

class Object(object):
    def __init__(self, xml, classes):
        self.name = xml.attrib["name"] if "name" in xml.attrib else xml.attrib["class"]

        # Pre-loading class info if available
        if "class" in xml.attrib:
            base = [c for c in classes if c.name == xml.attrib["class"]]

        # Loading features list
        self.features = []
        if xml.find("features") is not None:
            for feature in xml.find("features").findall("feature"):
                self.features.append(Feature(feature))


class Container(Object):
    def __init__(self, xml, classes):
        super(Container, self).__init__(xml, classes)

        # Loading elements list
        LoadChecker.checkNodesExist(xml, "elements")
        LoadChecker.checkAttribsExist(xml.find("elements"), "min", "max")
        self.min = xml.find("elements").attrib["min"]
        self.max = xml.find("elements").attrib["max"]

        self.elements = []
        for o in xml.find("elements").findall("object"):
            self.elements.append(Object(o, classes))

        for c in xml.find("elements").findall("container"):
            self.elements.append(Container(c, classes))
