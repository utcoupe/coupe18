#!/bin/usr/python
import rospy
from timer_client import TimerClient
from game_status_client import GameStatusClient, GameStatusConstants
from ai_loader import AILoader

class RobotAI():
    def __init__(self):
        self.timer = TimerClient() # Timer client.
        self.game_status = GameStatusClient()
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
            print "new turnldkjfhglskdjfhglskdfjhglskdjfhglskdjfhglskdjfghlskdjfhglskdjfghsldkfjghslkdfjghlskdfjghsldkfgjhs"
            if self.game_status.status == GameStatusConstants.STATUS_HALT:
                rospy.logwarn("[AI] detected game_status STATUS_HALT, aborting actions.")
                break

            if strategy.canContinue():
                strategy.getNext().execute(strategy.communicator)
            else:
                rospy.loginfo("[AI] In-Game actions finished!")
                break
            strategy.PrettyPrint()
