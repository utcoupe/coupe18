# -*- coding: utf-8 -*-
from definitions import *
from task import Task
from actionlist import ActionList


class Action(Task):
    def __init__(self, xml, actions, orders):
        super(Action, self).__init__(xml)
        self.Ref = xml.attrib["ref"]
        if not self.Name:
            self.Name = self.Ref
        self.loadxml(xml, actions, orders)

    def loadxml(self, xml, actions, orders):
        self.TASKS = ActionList(xml.find("actions"), actions, orders)
        self.TASKS.setParent(self)
        self.fetchBoundParams(xml)


    def getParamForBind(self, bind):
        for t in self.TASKS.TASKS:
            if t.getParamForBind(bind):
                return t.getParamForBind(bind)

    def fetchBoundParams(self, xml):
        if "params" not in [node.tag for node in xml]:
            return

        boundParamsList = []

        for param in xml.find("params"):
            if "name" not in param.attrib:
                raise KeyError("Parameters need a 'name' attribute ! (action '{}')".format(self.Name))

            name = param.attrib["name"]

            finalBind = self.getParamForBind(name)

            if not finalBind:
                raise KeyError("No parameter bound with '{}' !".format(name))
            else:
                boundParamsList.append(finalBind)

        self.BoundParams = {p.bind: p for p in boundParamsList}

    def setParameters(self, orderref_xml):
        for child in orderref_xml:
            name = child.tag
            if name not in self.BoundParams:
                raise KeyError("No bind found for '{}' !".format(name))
            self.BoundParams[name].parseValue(child)

        for p in self.BoundParams:
            if self.BoundParams[p].bind == p:
                self.BoundParams[p].checkValues()




    def getDuration(self):
        return self.TASKS.getDuration()

    def getNext(self):
        return self.TASKS.getNext()

    def refreshStatus(self):
        self.setStatus(self.TASKS.getStatus())

    def execute(self, communicator):
        if self.getStatus() in [TaskStatus.FREE, TaskStatus.PENDING]:
            next_tasks = self.getNext()
            if type(next_tasks) is list:
                raise NotImplementedError, "Simultaneous task launches aren't supported yet !" #TODO
            next_tasks.execute(communicator)
        else:
            raise ValueError, "ERROR asked to execute '{}' task that's not free".format(self.__repr__())

    def prettyprint(self, indentlevel):
        super(Action, self).prettyprint(indentlevel)
        self.TASKS.prettyprint(indentlevel + 1, hide = True)
    def __repr__(self):
        c = Console();c.setstyle(Colors.BOLD);c.setstyle(Colors.BLUE)
        c.addtext("[{} Action]".format(self.getStatusEmoji()))
        c.endstyle();c.setstyle(Colors.BLUE);c.addtext(" {0} ".format(self.Name));c.endstyle();c.setstyle(Colors.GRAY)

        c.addtext("[{} {}{}{}]".format(ExecutionMode.toEmoji(self.TASKS.executionMode),
                                        ExecutionOrder.toEmoji(self.TASKS.executionOrder),
                                        ", {}⚡".format(self.getReward()) if self.getReward() else "",
                                        ", ~{}⌛".format(int(self.TASKS.getDuration())) if self.TASKS.getDuration() else ""))
        return c.getText()