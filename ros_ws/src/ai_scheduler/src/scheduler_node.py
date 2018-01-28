#!/usr/bin/python
import time
import rospy
from scheduler_communication import AICommunication
from ai import RobotAI, GameProperties

from drivers_ard_hmi.msg import SetStrategies, SetTeams, HMIEvent
from ai_game_status.srv import SetStatus
from ai_game_status import StatusServices


class AINode():
    def __init__(self):
        self.DepartmentName, self.PackageName = "ai", "scheduler"

        rospy.init_node(self.PackageName, log_level = rospy.DEBUG)

        self.AI = RobotAI()
        self.AI.load_game_properties() # fetch available strategies and teams

        self._hmi_init = False
        self._ai_start_request = False
        rospy.Subscriber("/feedback/ard_hmi/hmi_event", HMIEvent, self.on_hmi_event)

        # Sending init status to ai/game_status, subscribing to game_status status pub.
        status_services = StatusServices(self.DepartmentName, self.PackageName, None, self.on_game_status)
        status_services.ready(True) # Tell ai/game_status the node initialized successfuly.

        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            if not self._hmi_init:
                self.send_strategies()
                self.send_teams()
            if self._ai_start_request:
                self.AI.start(AICommunication())
                self._ai_start_request = False
            r.sleep()

    def send_strategies(self):
        pub = rospy.Publisher("/feedback/ard_hmi/set_strategies", SetStrategies, queue_size=10)
        time.sleep(0.3)
        pub.publish(GameProperties.AVAILABLE_STRATEGIES)

    def send_teams(self):
        pub = rospy.Publisher("/feedback/ard_hmi/set_teams", SetTeams, queue_size=10)
        time.sleep(0.3)
        pub.publish(GameProperties.AVAILABLE_TEAMS)

    def send_set_ingame(self): # TODO wrong place ?
        pub = rospy.Publisher("/ai/game_status/set_status", SetStatus, queue_size=10)
        time.sleep(0.3)
        s = SetStatus()
        s.new_game_status = 2 # STATUS_INGAME TODO do more "better" ?
        pub.publish(s)

    # def runAI(self):
    #     #Debug: show task tree when starting the system
    #     rospy.loginfo("[AI] Launching robot with strategy :")
    #     self.AI.strategy.PrettyPrint()

    #     # Run the whole AI until there are no orders left to execute
    #     while not rospy.is_shutdown():
    #         if self.AI.strategy.canContinue():
    #             self.AI.strategy.getNext().execute(self.communication)
    #         else:
    #             rospy.loginfo("[AI] In-Game actions finished!")
    #             break
    #         self.AI.strategy.PrettyPrint()

    def on_game_status(self, msg):
        if msg.game_status == msg.STATUS_HALT:
            self.AI.halt()

    def on_hmi_event(self, req):
        if req.event == req.EVENT_HMI_INITIALIZED:
            time.sleep(0.5)
            self._hmi_init = True
        if req.event == req.EVENT_START:
            GameProperties.CURRENT_STRATEGY = GameProperties.AVAILABLE_STRATEGIES[req.chosen_strategy_id]
            GameProperties.CURRENT_TEAM     = GameProperties.AVAILABLE_TEAMS[req.chosen_team_id]
            rospy.set_param("/current_strategy", GameProperties.CURRENT_STRATEGY)
            rospy.set_param("/current_team",     GameProperties.CURRENT_TEAM)
            self._ai_start_request = True
            rospy.loginfo("[AI] Starting actions ! Strategy '{}' and team '{}'.".format(GameProperties.CURRENT_STRATEGY, GameProperties.CURRENT_TEAM))
        elif req.event == req.EVENT_GAME_CANCEL: # TODO remove ? Should be trigerred by a game_status HALT
            rospy.logwarn("[AI] HMI Asked to stop ! Stopping strategy execution.")
            self.AI.halt()

'''
PACKAGE STARTING POINT HERE
'''
if __name__ == "__main__":
    node = AINode()
