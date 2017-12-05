#!/bin/usr/python
from timer import *
from ai_loader import AILoader

class RobotAI():
    def __init__(self):
        self.strategy = None # Loaded when AI started with the right strategy.
        self.timer = GameTimer() # Timer client.
        self._loader = AILoader()

    def get_strategies(self):
        return self._loader.get_strategies()

    def start(self, strategyname, communicator):
        self.strategy = self._loader.load(strategyname, communicator)
        rospy.loginfo("[AI] Loaded strategy '{}', starting actions...".format(strategyname))
        self.strategy.execute()

    def halt(self):
        pass
