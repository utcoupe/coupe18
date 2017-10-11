'''
NOT DONE YET. BASIC STRUCTURE ONLY.
'''

class ConditionManager():
	def __init__(self):
		pass

	def createCondition(self, xml):
		if xml.tag == "position":
			return Position(xml)
		elif xml.tag == "chest":
			return Chest(xml)
		elif xml.tag == "aaa":
			pass
		else:
			raise ValueError, "ERROR condition '{}' type not recognised.".format(xml.tag)

	def checkConditions():
		for condition in self.Conditions:
			if condition.checkCondition():
				return 200, ""
			else:
				return 500, "ERROR Condition '{}' not executable.".format()



class Condition():
	def __init__(self, xml):
		pass

	def verifyCondition(self, communicator):
		pass

class Position(Condition):
	def __init__(self):
		pass

class Chest(Condition):
	def __init__(self):
		pass


'''
LIST OF CONDITIONS:
	- Position : robot or object has to be at a certain position to validate the condition.
		- @type : [entity, waypoint, object, xy, xya] type of position given.
		-

	- Chest : robot's particular chest has to be at a certain state (full, empty, quantity_in, quantity_left).
		- @qty : [full, empty, 1+, 2+, -1, -2]:
			full : chest is full (capacity is defined in the robot's XML definition file).
			empty : chest is empty.
			1+, 2+... : chest has to contain at least 1, 2... element(s).
			-1, -2... : chest has to have at least 1, 2... empty space(s)
'''
