import json
import rospy

class Condition(object):
    TYPE_NAME = ""

    def __init__(self, xml):
        # Check if mode is valid
        if "mode" not in xml.attrib():
            raise KeyError("This condition type requires a mode.")
        if xml.attrib["mode"] not in self.modes:
            raise ValueError("This mode '{}' does not exist with this condition type.".format(xml.attrib["mode"]))

        self.mode = xml.attrib["mode"]
        self.value = xml.tag
        self.check = self.modes[self.mode]


def ConditionCreator(xml):
    if "type" not in xml.attrib:
        raise KeyError("Conditions definitions need a type.")

    for cls in Condition.__subclasses__():
        if cls.TYPE_NAME == xml.attrib["type"]:
            return cls(xml)

    raise ValueError("No parser defined for type '{}' ! Please add a subclass"
                     "of 'Param' in the file ai_params.py".format(xml.attrib["type"]))


class MapDictCondition(Condition):
    TYPE_NAME = "map_dict"

    def __init__(self, xml):
        self.modes = {
            "is_equal": self._mode_is_equal
        }

        if "path" not in xml.attrib:
            raise KeyError("MapDict condition needs a 'path' attribute.")
        self.path = xml.attrib["path"]

        super(MapDictCondition, self).__init__(xml)

    def _mode_is_equal(self, communication):
        response = communication.SendRequest("/memory/map/get", {
            "request_path": self.path
        })
        if response.success:
            map_dict = json.loads(response.response)
            value_dict = json.loads(self.value)
            if json.dumps(map_dict, sort_keys=True) == json.dumps(value_dict, sort_keys=True):
                return True
        return False


class TFCondition(Condition):
    TYPE_NAME = "tf"
    pass


class MemoryCompareCondition(Condition):
    pass
