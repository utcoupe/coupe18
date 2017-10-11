#!/usr/bin/python
# -*- coding: utf-8 -*-
from random import randint
from conditions import *
import rospy, time
import copy

'''
GENERAL TOBEDONE
	- Service responses codes instead of simple true/false
	- Parameters
	- Conditions (chest, needsprevious..)

TESTS TOBEDONE
	- Fastest ExecutionOrder

WARNINGS
	- Paused tasks re-activation not implemented
'''

# utf-8 boxes link : http://jrgraphix.net/r/Unicode/2500-257F

#/*====================================
#=            Base classes            =
#====================================*/

class TaskStatus:
	CRITICAL            = ('CRITICAL'			, '💔')					# Fatal error, system will shutdown.
	WAITINGFORRESPONSE  = ('WAITINGFORRESPONSE'	, '💬')					# Order sent service or action message, waiting for response callback.
	NEEDSPREVIOUS       = ('NEEDSPREVIOUS'      , '↳')					# Task can't execute yet because it needs the previous task to be at SUCCESS still.
	PENDING             = ('PENDING'            , '⋯')					# For lists only. Active when one or not all child tasks are still active.
	FREE                = ('FREE'				, '⬜')					# Free task, not activated yet.
	PAUSED              = ('PAUSED'				, '🔶')					# TODO
	ERROR               = ('ERROR'				, '⛔', "error_msg")		# Error. Order couldn't be done, AI will try to find an alternative path of orders in the tree.
	BLOCKED             = ('BLOCKED'			, '◼')					# Node can't execute because conditions aren't fully satisfied.
	SUCCESS             = ('SUCCESS'			, '🆗', 0.0)				# Order and lists complete.
	def toEmoji(status):
		return status[1]

class ExecutionMode():
	ALL                 = ('all'				, '⚫')	# Will execute all tasks (except the blocked ones).
	ONE                 = ('one'				, '1')	# Will execute tasks in the list until one is successful.
	ATLEASTONE          = ('+'					, '+')	# At least one task in the list must be executed, will try to execute all.
	@staticmethod
	def toEmoji(mode):
		return mode[1]
	@staticmethod
	def fromText(text):
		if text == 'all'           : return ExecutionMode.ALL
		elif text == 'one'         : return ExecutionMode.ONE
		elif text == '+'           : return ExecutionMode.ATLEASTONE
		else: raise ValueError, "ExecutionMode '{}' not recognized.".format(text)

class ExecutionOrder():
	LINEAR              = ('linear'				, '⬇')	# Will follow linearly the order given in the XML file.
	RANDOM              = ('random'				, '🌀')	# Will pick a random free task.
	SIMULTANEOUS        = ('simultaneous'		, '⇶')	# Will activate all tasks at once.
	FASTEST             = ('fastest'			, '🕒')	# Will sort the tasks from least Duration to most.
	MOSTREWARD          = ('mostreward'			, '⚡')	# Will sort the tasks from most Reward to least. Execute tasks with same reward amount linearly.
	@staticmethod
	def toEmoji(order):
		return order[1]
	@staticmethod
	def fromText(text):
		if text == 'linear'        : return ExecutionOrder.LINEAR
		elif text == 'random'      : return ExecutionOrder.RANDOM
		elif text == 'simultaneous': return ExecutionOrder.SIMULTANEOUS
		elif text == 'fastest'     : return ExecutionOrder.FASTEST
		elif text == 'mostreward'  : return ExecutionOrder.MOSTREWARD
		else: raise ValueError, "ExecutionOrder '{}' not recognized.".format(text)


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
	def getStatus(self, str = False):
		return self.Status if not str else self.Status[0]

	def getStatusEmoji(self):
		return self.Status[1]
	def prettyprint(self, indentlevel):
		rospy.loginfo("\033[0m" + "  ║ " * (indentlevel - 1) + "  ╠═" + self.__repr__())
	def __repr__(self):
		return "<Task with No Name>"

class GameProperties():
	GAME_DURATION = None
	REWARD_POINTS = 0

#/*=====  End of Base classes  ======*/







class Strategy(Task):
	def __init__(self, xml, actions, orders):
		super(Strategy, self).__init__(xml)
		self.Name = xml.attrib["name"]
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






class ActionList(Task):
	def __init__(self, xml, actions, orders):
		super(ActionList, self).__init__(xml)
		self.Name = xml.attrib["name"] if "name" in xml.attrib else xml.tag
		self.executionMode    = ExecutionMode.fromText( xml.attrib["exec"])  if "exec"  in xml.attrib else ExecutionMode.ALL
		self.executionOrder   = ExecutionOrder.fromText(xml.attrib["order"]) if "order" in xml.attrib else ExecutionOrder.LINEAR
		self.Conditions = xml.find("conditions") if "conditions" in xml else None # Conditions that must be true before executing the actions.

		self.TASKS = None
		self.loadxml(xml, actions, orders)
		#if len(self.TASKS) < 2: raise ValueError, "ERROR {} task in a list, not accepted.".format(len(self.TASKS))

	def loadxml(self, xml, actions, orders):
		self.TASKS = []
		nextneedsprevious = False
		for node_xml in xml:
			tag = node_xml.tag
			if tag == "actionlist":
				i = ActionList(node_xml, actions, orders)
				i.setParent(self)
				if nextneedsprevious:
					i.Status = TaskStatus.NEEDSPREVIOUS;nextneedsprevious = False
				self.TASKS.append(i)
			elif tag == "actionref":
				instances = [action for action in actions if action.Ref == node_xml.attrib["ref"]]
				if len(instances) != 1:
					raise KeyError, "{} action instance(s) found with the name '{}'.".format(len(instances), node_xml.attrib["ref"])
				i = copy.deepcopy(instances[0])
				i.setParent(self)
				if nextneedsprevious:
					i.Status = TaskStatus.NEEDSPREVIOUS;nextneedsprevious = False
				self.TASKS.append(i)
			elif tag == "orderref":
				instances = [order for order in orders if order.Ref == node_xml.attrib["ref"]]
				if len(instances) != 1:
					raise KeyError, "{} order instance(s) found with the name '{}'.".format(len(instances), node_xml.attrib["ref"])
				i = copy.deepcopy(instances[0])
				i.setParent(self)
				if nextneedsprevious:
					i.Status = TaskStatus.NEEDSPREVIOUS;nextneedsprevious = False
				self.TASKS.append(i)
			elif tag == "nextneedsprevious":
				nextneedsprevious = True
			elif tag == "conditions":
				self.loadConditions(node_xml)
			else:
				rospy.logwarn("WARNING Element skipped because '{}' type was not recognized.".format(tag))

	def loadConditions(self, xml):
		#Conditions definition
		for conditions_xml in xml:
			if conditions_xml.tag == "needsprevious":
				#TODO check if it's possible (if there's a valid task before)
				self.NeedsPrevious = True

	def getReward(self):
		return self.Reward + sum([task.getReward() for task in self.TASKS])
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
			return free_tasks[randint(0, len(free_tasks) - 1)]

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
			raise ValueError, "ERROR asked to execute a task that's not free"

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
		# The order of conditions do count!
		if TaskStatus.CRITICAL in child_statuses:
			self.setStatus(TaskStatus.CRITICAL);return
		if TaskStatus.WAITINGFORRESPONSE in child_statuses:
			self.setStatus(TaskStatus.WAITINGFORRESPONSE);return

		if self.executionMode == ExecutionMode.ONE:
			if len([1 for c in child_statuses if c == TaskStatus.SUCCESS]) == 1:
				self.setStatus(TaskStatus.SUCCESS)
				#TODO block all dependent tasks too
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
				self.setStatus(TaskStatus.SUCCESS);return
		elif self.executionMode == ExecutionMode.ATLEASTONE:
			if len([1 for c in child_statuses if c == TaskStatus.SUCCESS]) >= 1:
				self.setStatus(TaskStatus.SUCCESS);return

		if TaskStatus.ERROR in child_statuses:
			self.setStatus(TaskStatus.ERROR)
			#TODO Block all dependent nodes
			return

	def find_executable_childs(self):
		pass

	def prettyprint(self, indentlevel, hide = False):
		if not hide:
			super(ActionList, self).prettyprint(indentlevel)
		for task in self.TASKS:
			task.prettyprint(indentlevel + (1 if not hide else 0))
	def __repr__(self):
		c = Console();c.setstyle(Colors.BOLD);c.setstyle(Colors.RED)
		c.addtext("[{} ActionList] {} ".format(self.getStatusEmoji(), self.Name))
		c.endstyle();c.setstyle(Colors.GRAY)
		c.addtext("[{} {}{}{}]".format(ExecutionMode.toEmoji(self.executionMode),
												ExecutionOrder.toEmoji(self.executionOrder),
												", {}⚡".format(self.getReward()) if self.getReward() else "",
												", ~{}⌛".format(int(self.getDuration())) if self.getDuration() else ""))
		return c.getText()





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






class Order(Task):
	def __init__(self, xml):
		super(Order, self).__init__(xml)
		self.Ref = xml.attrib["ref"]
		if not self.Name:
			self.Name = self.Ref

		self.Duration = float(xml.attrib["duration"]) if "duration" in xml.attrib else 0.0 # Manually estimated time to execute this action
		#self.Reward   = int(xml.attrib["reward"]) if "reward" in xml.attrib else 0 #TODO delete line :)
		#self.Ratio = self.Reward / self.Duration # TODO Implement ?

		self.Message = Message(xml.find("message"))
		self.TimeTaken = None

	def getDuration(self):
		return self.Duration

	def execute(self, communicator):
		'''if len(kwargs) != len(self.NeededParamsIDs):
			raise ValueError, "ERROR missing or too many parameters for message."'''
		self.setStatus(TaskStatus.WAITINGFORRESPONSE)
		rospy.loginfo("Executing task : {}...".format(self.__repr__()))


		response, self.TimeTaken = self.Message.send(communicator)

		# After response
		if response.response_code == 200:
			self.setStatus(TaskStatus.SUCCESS)
			rospy.logdebug("[AI] Order '{}' succeeded!".format(self.Name))
			#GameProperties.REWARD_POINTS += self.getReward() #TODO
		else:
			self.setStatus(TaskStatus.ERROR)
			rospy.logerr("[AI] Order '{}' execution failed (code : {}, reason : '{}')".format(self.Name, response.response_code, response.reason))

	def __repr__(self):
		c = Console()
		c.setstyle(Colors.BOLD)
		c.addtext("[{}{} Order] ".format(self.getStatusEmoji(),
										" , {0:.1f}⌛".format(self.TimeTaken) if self.getStatus() in [TaskStatus.SUCCESS,
										TaskStatus.ERROR, TaskStatus.PAUSED, TaskStatus.CRITICAL] else ""))
		c.endstyle()
		c.addtext("{}{}".format(self.Name, " [{}⚡]".format(self.Reward) if self.Reward else ""))
		return c.getText()



class Message():
	def __init__(self, xml):
		self.Department  = xml.attrib["department"]
		self.Destination = xml.find("dest").text if len(xml.findall("dest")) > 0 else "main"
		self.Command     = xml.find("command").text

		# Save which parameters are needed for the message
		self.NeededParamsIDs = []
		for param in xml.find("params").findall("param"):
			self.NeededParamsIDs.append(param.attrib["id"])


		self.Parameters = {}
		for param in xml.find("params"):
			pass
		#self.check_valid() TODO reput
		self.Parameters = xml.find("params").text #TODO do full params handling, this is just pasting the params tag

	def check_valid(self): #checks if all parameters are valid
		#TODO forget that ? hardcoded
		if self.Department not in ["ai", "mapmanager", "localization", "perception", "movement"]:
			raise KeyError, "ERROR '{}' department doesn't exist ! (may need to update the check list in the ai_classes.py file)".format(self.Department)


	def send(self, communicator):
		params = None #TODO
		self.startTime = time.time()
		response = communicator.SendGenericCommand(self.Department, self.Destination, self.Command, self.Parameters)
		self.TimeTaken = time.time() - self.startTime
		return response, self.TimeTaken






class Colors():
	BOLD  = "\033[1m"
	GRAY  = "\033[92m"
	BLUE  = "\033[95m"
	RED   = "\033[91m"
	RESET = "\033[0m"
class Console():
	def __init__(self):
		self.Text = Colors.RESET

	def setstyle(self, color):
		self.Text += color
	def addtext(self, text):
		self.Text += text
	def endstyle(self):
		self.Text += Colors.RESET

	def getText(self):
		self.endstyle()
		return self.Text
