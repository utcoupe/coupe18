# -*- coding: utf-8 -*-
import copy
import random
import rospy
from definitions import *
from task import Task
from order import Order


class ActionList(Task):
    MAX_REPEATS = 50 # If repeat mode is 'while', this will be the repeat limit.

    def __init__(self, xml, actions, orders):
        super(ActionList, self).__init__(xml)
        self.Name = xml.attrib["name"] if "name" in xml.attrib else xml.tag
        self.executionMode  = ExecutionMode.fromText( xml.attrib["exec"])   if "exec"  in xml.attrib else ExecutionMode.ALL
        self.repeatMode     = RepeatMode.fromText(    xml.attrib["repeat"]) if "repeat"in xml.attrib else RepeatMode.ONCE
        self._repeats = 0 # used to track how many times the list already repeated
        self._successful_repeats = 0
        self._repeats_max = ActionList.MAX_REPEATS
        if self.repeatMode == RepeatMode.ONCE: self._repeats_max = 1
        if self.repeatMode == RepeatMode.FOR:  self._repeats_max = int(xml.attrib["repeat"])

        self.executionOrder = ExecutionOrder.fromText(xml.attrib["order"])  if "order" in xml.attrib else ExecutionOrder.LINEAR
        self.Conditions = xml.find("conditions") if "conditions" in xml else None # Conditions that must be true before executing the actions.
        self.TASKS = self.loadxml(xml, actions, orders)

    def loadxml(self, xml, actions, orders):
        tasks = []
        for node_xml in xml:
            tag = node_xml.tag
            if tag == "actionlist":
                i = ActionList(node_xml, actions, orders)
                i.setParent(self)
                if "needsprevious" in node_xml.attrib and node_xml.attrib["needsprevious"] == 'true':
                    i.Status = TaskStatus.NEEDSPREVIOUS
                tasks.append(i)
            elif tag == "actionref" or tag == "orderref":
                instances = [action for action in actions if action.Ref == node_xml.attrib["ref"]] if tag == "actionref" else \
                            [order  for order  in orders  if order.Ref  == node_xml.attrib["ref"]]
                if len(instances) != 1:
                    raise KeyError, "{} action instance(s) found with the name '{}'.".format(len(instances), node_xml.attrib["ref"])
                i = copy.deepcopy(instances[0])
                i.setParent(self)
                i.Reward = int(node_xml.attrib["reward"]) if "reward" in node_xml.attrib else i.Reward
                i.setParameters(node_xml)
                if "needsprevious" in node_xml.attrib and node_xml.attrib["needsprevious"] == 'true':
                    i.Status = TaskStatus.NEEDSPREVIOUS
                if "name" in node_xml.attrib:
                    i.Name = node_xml.attrib["name"]
                tasks.append(i)
            elif tag == "conditions":
                self.loadConditions(node_xml)
            elif tag == "team":
                if node_xml.attrib["name"] == GameProperties.CURRENT_TEAM:
                    tasks += self.loadxml(node_xml, actions, orders)
            else:
                rospy.logwarn("WARNING Element skipped at init because '{}' type was not recognized.".format(tag))
        return tasks

    def loadConditions(self, xml):
        #Conditions definition
        for conditions_xml in xml:
            if conditions_xml.tag == "needsprevious":
                #TODO check if it's possible (if there's a valid task before)
                self.NeedsPrevious = True

    def getReward(self):
        return self.Reward + sum([task.getReward() for task in self.TASKS])

    def getActiveReward(self):
        return self.getReward() if self.getStatus() == TaskStatus.SUCCESS else sum([task.getActiveReward() for task in self.TASKS])

    def getDuration(self):
        return sum([task.getDuration() for task in self.TASKS])

    def getNext(self):
        # Decides the next task(s) to execute based on the childs' statuses and the ExecutionOrder XML setting.
        for task in self.TASKS: # Execute any pending task in any case  #TODO#1 will create problems ?
            if task.getStatus() == TaskStatus.PENDING: return task

        if   self.executionOrder == ExecutionOrder.LINEAR:
            for task in self.TASKS:
                if task.getStatus() in [TaskStatus.FREE, TaskStatus.PENDING]: return task

        elif self.executionOrder == ExecutionOrder.RANDOM:
            free_tasks = [task for task in self.TASKS if task.getStatus() == TaskStatus.FREE]
            return free_tasks[random.randint(0, len(free_tasks) - 1)]

        elif self.executionOrder == ExecutionOrder.SIMULTANEOUS:
            return [task for task in self.TASKS if task.getStatus() in [TaskStatus.FREE, TaskStatus.PENDING]] #TODO#1

        elif self.executionOrder == ExecutionOrder.FASTEST:
            record, result = 10000000, None
            for task in self.TASKS:
                if task.getDuration() < record:
                    record = task.getDuration()
                    result = task
            return result

        elif self.executionOrder == ExecutionOrder.MOSTREWARD:
            record, result = -1, None
            for task in [task for task in self.TASKS if task.getStatus() == TaskStatus.FREE]:
                if task.getReward() > record:
                    record = task.getReward()
                    result = task
            return result

    def execute(self, communicator):
        if self.getStatus() in [TaskStatus.FREE, TaskStatus.PENDING]:
            self.getNext().execute(communicator)
        else:
            rospy.logerr("ERROR asked to execute a task that's not free")

    def resetStatus(self, refresh_parent=False): # wipes all progress of this list and all descendent tasks.
        self.setStatus(TaskStatus.FREE, refresh_parent)
        if not refresh_parent: #TODO ~~~~
            self._repeats = 0
        for task in self.TASKS:
            task.resetStatus()

    def _markSuccess(self):
        if self.repeatMode != RepeatMode.ONCE:
            if self.repeatMode == RepeatMode.WHILE or self.repeatMode == RepeatMode.FOR:
                self._repeats += 1
                self._successful_repeats += 1
                if self._repeats < self._repeats_max: # if repeat limit not reached yet, mark everything as free
                    print "resetting status of task and children"
                    self.resetStatus(refresh_parent=True)
                    return
                elif self._successful_repeats < self._repeats_max:
                    self.setStatus(TaskStatus.ERROR)
                    return

        self.setStatus(TaskStatus.SUCCESS)

    def refreshStatus(self):
        # unblock or block tasks that need previous tasks
        previous_task = self.TASKS[0]
        for task in self.TASKS[1:]:
            if task.getStatus() == TaskStatus.NEEDSPREVIOUS:
                if previous_task.getStatus() == TaskStatus.SUCCESS:
                    task.setStatus(TaskStatus.FREE, refresh_parent = False)
                if previous_task.getStatus() in [TaskStatus.BLOCKED, TaskStatus.ERROR]:
                    task.setStatus(TaskStatus.BLOCKED)
            previous_task = task

        # Decides the status of the list based on the childs' statuses and the ExecutionMode XML setting.
        child_statuses = [task.getStatus() for task in self.TASKS]
        # CAUTION The order of conditions do count!
        if TaskStatus.CRITICAL in child_statuses:
            self.setStatus(TaskStatus.CRITICAL);return
        if TaskStatus.WAITINGFORRESPONSE in child_statuses:
            self.setStatus(TaskStatus.WAITINGFORRESPONSE);return

        if self.executionMode == ExecutionMode.ONE:
            if len([1 for c in child_statuses if c == TaskStatus.SUCCESS]) == 1:
                self._markSuccess()
                for task in self.TASKS:
                    if task.getStatus() in [TaskStatus.FREE, TaskStatus.PENDING]:
                        task.setStatus(TaskStatus.BLOCKED)
                return

        if TaskStatus.PAUSED in child_statuses:
            self.setStatus(TaskStatus.PAUSED);return
        if TaskStatus.PENDING in child_statuses:
            self.setStatus(TaskStatus.PENDING);return
        if TaskStatus.FREE in child_statuses:
            self.setStatus(TaskStatus.PENDING);return

        if self.executionMode == ExecutionMode.ALL:
            if len([1 for c in child_statuses if c == TaskStatus.SUCCESS]) == len(child_statuses):
                self._markSuccess();return
        elif self.executionMode == ExecutionMode.ATLEASTONE:
            if len([1 for c in child_statuses if c == TaskStatus.SUCCESS]) >= 1:
                self._markSuccess();return

        if TaskStatus.ERROR in child_statuses:
            if self.repeatMode == RepeatMode.WHILE or self.repeatMode == RepeatMode.FOR:
                self._repeats += 1
                if self._repeats < self._repeats_max: # if repeat limit not reached yet, mark everything as free
                    print "resetting status of task and children"
                    self.resetStatus(refresh_parent=True)
                    self.setStatus(TaskStatus.PENDING)
                    return
                elif self._successful_repeats < self._repeats_max:
                    self.setStatus(TaskStatus.ERROR)
                    return
            else:
                self.setStatus(TaskStatus.ERROR)
                #TODO Block all dependent nodes
                return

    def prettyprint(self, indentlevel, hide = False):
        if not hide:
            super(ActionList, self).prettyprint(indentlevel)
        for task in self.TASKS:
            task.prettyprint(indentlevel + (1 if not hide else 0))

    def __repr__(self):
        c = Console();c.setstyle(Colors.BOLD);c.setstyle(Colors.RED)
        c.addtext("[{} ActionList] {} ".format(self.getStatusEmoji(), self.Name))
        c.endstyle();c.setstyle(Colors.GRAY)
        c.addtext("[{}{}{}{}{}]".format(ExecutionMode.toEmoji(self.executionMode),
                                       " " + ExecutionOrder.toEmoji(self.executionOrder),
                                       " {}/{}".format(str(self._repeats), str(self._repeats_max)) \
                                            + RepeatMode.toEmoji(self.repeatMode) if self.repeatMode != RepeatMode.ONCE else "",
                                       ", {}⚡".format(self.getReward()) if self.getReward() else "",
                                       ", ~{}⌛".format(int(self.getDuration())) if self.getDuration() else ""))
        return c.getText()
