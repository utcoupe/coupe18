# -*- coding: utf-8 -*-
import rospy
from definitions import *

class Task(object):
    def __init__(self, xml, status = TaskStatus.FREE):

        self.Name = xml.attrib["name"] if "name" in xml.attrib else None
        self.Reward = int(xml.attrib["reward"]) if "reward" in xml.attrib else 0
        self.Status = status

        self.Parent = None
        self.NeedsPrevious = False

        self.loadConditions(xml)

    def loadConditions(self, xml):
        self.Conditions = []
        if not "conditions" in [node.tag for node in xml]: return
        for condition in xml.find("conditions"):
            pass#self.Conditions.append()



    def getReward(self):
        return self.Reward

    def setParent(self, parent):
        self.Parent = parent
    def setStatus(self, new_status, refresh_parent = True):
        if self.Status != new_status:
            self.Status = new_status
            if refresh_parent and self.Parent:
                self.Parent.refreshStatus()
    def getStatus(self, text_version = False):
        return self.Status if not text_version else self.Status[0]

    def getStatusEmoji(self):
        return self.Status[1]
    def prettyprint(self, indentlevel):
        rospy.loginfo("\033[0m" + "  ║ " * (indentlevel - 1) + "  ╠═" + self.__repr__())
    def __repr__(self):
        return "<Task with No Name>"
