# -*- coding: utf-8 -*-
import rospy
from definitions import *
from task import Task
import params

class Order(Task):
    def __init__(self, xml):
        super(Order, self).__init__(xml)
        self.Ref = xml.attrib["ref"]
        if not self.Name:
            self.Name = self.Ref

        self.Duration = float(xml.attrib["duration"]) if "duration" in xml.attrib else 0.0  # Manually estimated time to execute this action
        self.Message = Message(xml.find("message"))
        self.Response = Response(xml.find("response"))
        self.TimeTaken = None

    def setParameters(self, orderref_xml):
        childs = [child for child in orderref_xml]
        self.Message.setParameters(childs)

    def getParamForBind(self, bind):
        for p in self.Message.Parameters:
            for b in p.getBoundParams():
                if b.bind == bind:
                    return b
        return False

    def getDuration(self):
        return self.Duration

    def resetStatus(self, refresh_parent=False): # wipes all progress of this list and all descendent tasks.
        self.setStatus(TaskStatus.FREE, refresh_parent) #TODO on all types of tasks, reset to needsprevious if it was like that at init

    def execute(self, communicator):
        self.setStatus(TaskStatus.WAITINGFORRESPONSE)
        rospy.loginfo("Executing task: {}...".format(self.__repr__()))

        self.Message.send(communicator, self.callback)

    def callback(self, res, time_taken):
        self.TimeTaken = time_taken
        # try:
        #     if res.result == res.RESULT_SUCCESS:
        #         result = True
        #     elif res.result in [res.RESULT_PAUSE, res.RESULT_FAIL]:
        #         pass
        # except AttributeError: # Otherwise, look for a bool type
        #     try: result = res.success
        #     except: pass
        #     try: result = res.response
        #     except: pass
        #     try: result = res.result
        #     except: pass

        #     try: reason = res.verbose_reason
        #     except: reason = ""
        result = self.Response.compare(res)

        if result is True:
            self.setStatus(TaskStatus.SUCCESS)
            rospy.loginfo("Task succeeded: {}".format(self.__repr__()))
        else:
            self.setStatus(TaskStatus.ERROR)
            rospy.logerr("Task failed: {}".format(self.__repr__()))

    def __repr__(self):
        c = Console()
        c.setstyle(Colors.BOLD)
        c.addtext("[{}{} Order] ".format(self.getStatusEmoji(),
                                        " , {0:.1f}⌛".format(self.TimeTaken) if self.getStatus() in [TaskStatus.SUCCESS,
                                        TaskStatus.ERROR, TaskStatus.PAUSED, TaskStatus.CRITICAL] else ""))
        c.endstyle()
        c.addtext("{}".format(self.Name))
        c.endstyle();c.setstyle(Colors.GRAY)
        c.addtext("{}".format(" [{}⚡]".format(self.Reward) if self.Reward else ""))
        return c.getText()


class Message():
    def __init__(self, xml):
        if "dest" not in xml.attrib:
            raise KeyError("PARSE ERROR ! Messages need a 'dest' attribute")

        self.Destination = xml.attrib["dest"]
        self.Timeout = int(xml.attrib["timeout"]) if "timeout" in xml.attrib else None
        self.Parameters = []

        # parse declaration of parameters
        paramsNames = []
        for param in xml.findall("param"):
            p = params.ParamCreator(param)
            self.Parameters.append(p)
            if p.name in paramsNames:
                raise KeyError("PARSE ERROR ! Param {} already defined here".format(p.name))
            paramsNames.append(p.name)

    def send(self, communicator, callback):
        ros_params = {p.name: p.getRos() for p in self.Parameters}
        communicator.SendRequest(self.Destination, ros_params, self.Timeout, callback)

    def setParameters(self, xml_list): # populate values of params
        for child in xml_list:
            name = child.tag
            parameters = [p for p in self.Parameters if p.name == name]

            if len(parameters) != 1:
                raise KeyError("PARSE ERROR ! Param {} defined {} times"
                               .format(child.tag, len(parameters)))

            param = parameters[0]
            if param.preset:
                raise KeyError("PARSE ERROR ! Param {} is preset, \
                                cannot modify it".format(param.name))
            param.parseValue(child)
        [p.checkValues() for p in self.Parameters if not p.bind]


class Response(object):
    def __init__(self, xml):
        results = {
            "success": TaskStatus.SUCCESS,
            "pause": TaskStatus.PAUSED,
            "error": TaskStatus.ERROR
        }
        self.Result = results[xml.attrib["result"]] #TODO not used yet
        self.Parameters = []

        # parse declaration of parameters
        _param_names = []
        for param in xml.findall("param"):
            p = params.ParamCreator(param)
            self.Parameters.append(p)
            if p.condition is None:
                raise KeyError("PARSE ERROR ! Param {} needs a 'condition' attribute.".format(p.name))
            if p.name in _param_names:
                raise KeyError("PARSE ERROR ! Param {} already defined here.".format(p.name))
            _param_names.append(p.name)

        # if not self.Parameters:
        #     raise ValueError("PARSE ERROR! Order response must have at least one param to be checked.")

    def compare(self, response):
        if response in [False, None]:
            return False
        for param in self.Parameters:
            if not param.compare(getattr(response, param.name)):
                return False
        return True
