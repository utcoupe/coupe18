#!/bin/usr/python
from timer import *
from ai_loader import AILoader

class RobotAI():
    def __init__(self):
        self.timer = GameTimer() # Timer client.
        self._loader = AILoader()

    def get_strategies(self):
        return self._loader.get_strategies()

    def start(self, strategyname, communicator):
        strategy = self._loader.load(strategyname, communicator)
        rospy.loginfo("[AI] Loaded strategy '{}', starting actions...".format(strategyname))
        self.execute(strategy)

    def halt(self):
        pass

    def execute(self, strategy):
        strategy.PrettyPrint()
        # Run the whole AI until there are no orders left to execute
        while not rospy.is_shutdown():
            if strategy.canContinue() and not self.timer.finished:
                strategy.getNext().execute(strategy.communicator)
            else:
                rospy.loginfo("[AI] In-Game actions finished!")
                break
            strategy.PrettyPrint()
