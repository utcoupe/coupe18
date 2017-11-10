#!/usr/bin/env python

import xml.etree.ElementTree as ET
import rospy


class BeltParser(object):
    """Class used to parse the definition XML"""
    def __init__(self, file):
        super(BeltParser, self).__init__()
        rospy.logdebug("[RECOGNITION] Parsing belt definition file ...")
        root = ET.parse(file).getroot()

        #  parse params
        if root.find("params") is None:
            raise KeyError("A 'params' element is required in the definition")

        self.Params = {c.tag: float(c.text) for c in root.find("params")}

        required = ["max_range", "angle"]
        for p in required:
            if p not in self.Params:
                raise KeyError("A '{}' element is required in the parameters"
                               .format(p))

        #  parse sensors
        if root.find("sensors") is None:
            raise KeyError("A 'sensors' element is required in the definition")

        sensors = []
        for sensor in root.find("sensors"):
            if "id" not in sensor.attrib:
                raise KeyError("A 'sensor' element need an 'id' atttribute")

            required = ["x", "y", "a"]
            for p in required:
                if sensor.find(p) is None:
                    raise KeyError("A '{}' element is required in the sensor"
                                   .format(p))

            sensors.append({
                "id": sensor.attrib["id"],
                "x": float(sensor.find("x").text),
                "y": float(sensor.find("y").text),
                "a": float(sensor.find("a").text)
            })

        self.Sensors = {s["id"]: s for s in sensors}
