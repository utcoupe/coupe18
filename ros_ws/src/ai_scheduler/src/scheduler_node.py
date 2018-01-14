#!/usr/bin/python
import time
import rospy
from scheduler_communication import AICommunication
from scheduler_services import AIServices
from ai import RobotAI

from ai_scheduler.srv import AICommand, AICommandResponse
from drivers_ard_hmi.msg import SetStrategies, HMIEvent
from ai_game_status.srv import SetStatus
from ai_game_status_services import StatusServices

class AINode():
    def __init__(self):
        self.DepartmentName, self.PackageName = "ai", "scheduler"

        rospy.init_node(self.PackageName, log_level = rospy.DEBUG)

        self.AI = RobotAI()
        self.available_strategies = self.AI.get_strategies()

        rospy.Subscriber("/feedback/ard_hmi/hmi_event", HMIEvent, self.on_hmi_event)

        # Sending init status to ai/game_status, subscribing to game_status status pub.
        status_services = StatusServices(self.DepartmentName, self.PackageName, None, self.on_game_status)
        status_services.ready(True) # Tell ai/game_status the node is initialized.
        rospy.loginfo("[AI] Ready. Waiting for activation.")
        self.send_strategies()
        rospy.spin()

    def send_strategies(self):
        pub = rospy.Publisher("/feedback/ard_hmi/set_strategies", SetStrategies, queue_size=10)
        time.sleep(0.3)
        pub.publish(SetStrategies(self.available_strategies))

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
        print "hi"
        if req.event == 1: #req.EVENT_START: TODO adapt on HMI branch and merge
            strat_name = self.available_strategies[req.chosen_strategy_id]
            rospy.loginfo("[AI] Starting actions ! Strategy '{}'.".format(strat_name))
            self.AI.start(strat_name, AICommunication())
        elif req.event == req.EVENT_GAME_CANCEL:
            rospy.logwarn("[AI] HMI Asked to stop ! Stopping strategy execution.")
            self.AI.halt()
        return AICommandResponse()

'''
PACKAGE STARTING POINT HERE
'''
if __name__ == "__main__":
    node = AINode()
