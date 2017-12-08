import rospy
from definitions import *
from actionlist import ActionList
from task import Task

class Strategy(Task):
    def __init__(self, xml, actions, orders, communicator):
        super(Strategy, self).__init__(xml)
        self.Name = xml.attrib["name"]
        self.communicator = communicator
        self.loadxml(xml, actions, orders)

    def loadxml(self, xml, actions, orders):
        # Game Properties
        GameProperties.GAME_DURATION = int(xml.find('game').find("time").text) # Save game duration in seconds

        # Fill actions
        self.TASKS = ActionList(xml.find("actions"), actions, orders)
        self.TASKS_ONFINISH = ActionList(xml.find("actions_onfinish"), actions, orders)

    def canContinue(self):
        '''TODO
        if timer.is_finished():
            return False'''
        return self.getStatus() in [TaskStatus.FREE, TaskStatus.PENDING, TaskStatus.WAITINGFORRESPONSE]
    def getNext(self): # Returns the next free task (ActionList, Action or Order).
        return self.TASKS.getNext()

    def getStatus(self):
        return self.TASKS.getStatus()

    def PrettyPrint(self):
        rospy.loginfo('[STRATEGY] ' + self.__repr__())
        self.TASKS.prettyprint(1)
        self.TASKS_ONFINISH.prettyprint(1)
    def __repr__(self):
        return self.Name
