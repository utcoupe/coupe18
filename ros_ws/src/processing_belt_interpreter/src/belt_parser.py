#!/usr/bin/env python

import xml.etree.ElementTree as ET
import rospy


class BeltParser(object):
    """Class used to parse the definition XML"""
    def __init__(self, file):
        super(BeltParser, self).__init__()
        rospy.logdebug("Parsing belt definition...")

        root = ET.parse(file).getroot()

        required = ["max_range", "angle", "precision"]

        #  parse params
        if required and root.find("params") is None:
            msg = "Can't parse belt definition file: a 'params' element is required. Shutting down."
            rospy.logfatal(msg)
            raise rospy.ROSInitException(msg)

        self.Params = {c.tag: float(c.text) for c in root.find("params")}

        for p in required:
            if p not in self.Params:
                msg = "Can't parse belt definition: a '{}' element is required in the parameters. Shutting down."\
                    .format(p)
                rospy.logfatal(msg)
                raise rospy.ROSInitException(msg)

        #  parse sensors
        if root.find("sensors") is None:
            msg = "Can't parse belt definition: a 'sensors' element is required. Shutting down."
            rospy.logfatal(msg)
            raise rospy.ROSInitException(msg)

        sensors = []
        for sensor in root.find("sensors"):
            if "id" not in sensor.attrib:
                rospy.logerr("Can't parse sensor definition: a 'id' attribute is required. Skipping this sensor.")
                continue

            required = ["x", "y", "a"]
            for p in required:
                if sensor.find(p) is None:
                    rospy.logerr("Can't parse sensor definition: a '{}' element is required. Skipping this sensor.".format(p))


            sensors.append({
                "id": sensor.attrib["id"],
                "x": float(sensor.find("x").text),
                "y": float(sensor.find("y").text),
                "a": float(sensor.find("a").text)
            })


        if not sensors:
            rospy.logwarn("No sensor found in belt definition.")

        rospy.logdebug("{} sensors found in belt definition".format(len(sensors)))

        self.Sensors = {s["id"]: s for s in sensors}
