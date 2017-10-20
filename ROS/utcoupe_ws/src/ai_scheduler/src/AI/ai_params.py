#!/usr/bin/env python

# base class for parsing xml to param object
class Param(object):
    TYPE_NAME = ""

    def __init__(self, xml_definition):
        self.parseDefinition(xml_definition)

    def parseValue(self, xml):
        pass;

    def parseDefinition(self, xml): # parse name, type, required and preset
        if not "name" in xml.attrib:
            raise KeyError, "PARSING ERROR ! Params need a 'name' attribute"
        if not "type" in xml.attrib:
            raise KeyError, "PARSING ERROR ! Params need a 'type' attribute"


        self.name = xml.attrib["name"].lower()
        self.type = xml.attrib["type"].lower()

        self.required = (xml.attrib["optional"].lower() != "true") if "optional" in xml.attrib else True;
        self.preset = False

        childs = 0
        for c in xml: childs += 1

        if childs or xml.text: # parse preset
            self.value = self.parseValue(xml)
            self.preset = True
        else:
            self.value = None

# factory
def ParamCreator(xml):
    if not "type" in xml.attrib:
        raise KeyError, "PARSE ERROR : Param definition need a type !"

    for cls in Param.__subclasses__():
        if cls.TYPE_NAME == xml.attrib["type"]:
            return cls(xml)

    raise ValueError, "PARSE ERROR ! No parser defined for type '{}'".format(xml.attrib["type"])




# Child classes - one for each type of param

class StringParser(Param):
    TYPE_NAME = "string"

    def parseValue(self, xml):
        print xml.text
        return xml.text


class IntParser(Param):
    TYPE_NAME = "int"

    def parseValue(self, xml):
        return int(xml.text)

class FloatParser(Param):
    TYPE_NAME = "float"

    def parseValue(self, xml):
        return float(xml.text)
