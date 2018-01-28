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

    def _check_attrib_keys_exist(self, xml, *keys_required):
        for k in keys_required:
            if k not in xml.attrib:
                m = "{} condition needs a '{}' attribute.".format(self.TYPE_NAME, k)
                rospy.logerr(m)
                raise ValueError(m)


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

        self._check_attrib_keys_exist(xml, "path")
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

    def __init__(self, xml):
        self.modes = {
            "is_equal": self._mode_max_distance
        }

        self._check_attrib_keys_exist(xml, "origin_frame", "target_frame")
        self.origin_frame = xml.attrib["origin_frame"]
        self.target_frame = xml.attrib["target_frame"]

        super(TFCondition, self).__init__(xml)

    def _mode_max_distance(self, communication):
        transform = communication.GetTFTransform(self.origin_frame, self.target_frame)
        return False


class MemoryCompareCondition(Condition):
    pass
