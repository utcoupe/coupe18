#!/usr/bin/env python

import std_msgs.msg
import geometry_msgs.msg

#TODO : add default
class Param(object):  # base class for parsing xml to param object
    TYPE_NAME = ""
    TYPE_ROS = None

    def __init__(self, xml=None):
        if xml is not None:
            self.parseDefinition(xml)
            if "preset" in xml.attrib and xml.attrib["preset"] == "true":
                preset = True
            else:
                preset = False
            self.parseValue(xml, preset)

    def getRos(self):
        for k in self.value:
            if isinstance(self.value[k], Param):
                print self.name
                self.value[k] = self.value[k].getRos()

        return self.TYPE_ROS(**self.value)

    def parseValue(self, xml, preset=False):
        self.preset = preset
        for child in xml:
            if child.tag in self.value:
                if isinstance(self.value[child.tag], Param):
                    self.value[child.tag].parseValue(child, preset)
                else:
                    self.value[child.tag] = child.text
            else:
                raise KeyError("PARSE ERROR ! Param {} does not have an attribute {}"
                               .format(self.TYPE_NAME, child.tag))

    def parseDefinition(self, xml):  # parse name, type, required and preset
        if "name" not in xml.attrib:
            raise KeyError("PARSE ERROR ! Params need a 'name' attribute")

        self.name = xml.attrib["name"].lower()

        self.required = (xml.attrib["optional"].lower() !=
                         "true") if "optional" in xml.attrib else True
        if "preset" in xml.attrib:
            self.preset = (xml.attrib["preset"].lower() == "true")
        else:
            self.preset = False

        if self.preset and self.required:
            raise KeyError("PARSE ERROR ! Param cannot be preset and optional")


def ParamCreator(xml):  # factory
    if "type" not in xml.attrib:
        raise KeyError("PARSE ERROR : Param definition need a type !")

    for cls in Param.__subclasses__():
        if cls.TYPE_NAME == xml.attrib["type"]:
            return cls(xml)

    raise ValueError("PARSE ERROR ! No parser defined for type '{}'"
                     .format(xml.attrib["type"]))


# Child classes - one for each type of param

class StringParser(Param):
    TYPE_NAME = "string"
    TYPE_ROS = std_msgs.msg.String

    def __init__(self, xml=None):
        self.value = {
            'data': None
        }
        super(StringParser, self).__init__(xml)


    def parseValue(self, xml, preset=False):
        self.preset = preset
        if not xml.text:
            if not preset:
                return
            else:
                raise KeyError("PARSE ERROR ! String param need a content !")
        self.value["data"] = xml.text

    def getRos(self):
        return self.value["data"]


class IntParser(Param):
    TYPE_NAME = "int"
    TYPE_ROS = std_msgs.msg.Int64

    def __init__(self, xml=None):
        self.value = {
            'data': None
        }
        super(IntParser, self).__init__(xml)

    def parseValue(self, xml, preset=False):
        self.preset = preset
        if not xml.text:
            if not preset:
                return
            else:
                raise KeyError("PARSE ERROR ! Int param need a content !")
        self.value["data"] = int(xml.text)

    def getRos(self):
        return self.value["data"]


class FloatParser(Param):
    TYPE_NAME = "float"
    TYPE_ROS = std_msgs.msg.Float64

    def __init__(self, xml=None):
        self.value = {
            'data': None
        }
        super(FloatParser, self).__init__(xml)

    def parseValue(self, xml, preset=False):
        self.preset = preset
        if not xml.text:
            if not preset:
                return
            else:
                raise KeyError("PARSE ERROR ! Float param need a content !")
        self.value["data"] = float(xml.text)

    def getRos(self):
        return self.value["data"]


class Pose2DParser(Param):
    TYPE_NAME = "pose2d"
    TYPE_ROS = geometry_msgs.msg.Pose2D

    def __init__(self, xml=None):
        self.value = {
            'x': FloatParser(),
            'y': FloatParser(),
            'theta': FloatParser()
        }
        super(Pose2DParser, self).__init__(xml)
