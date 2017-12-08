#!/usr/bin/python
import rospy
from scheduler_communication import AICommunication
from scheduler_services import AIServices
from ai import RobotAI

from ai_scheduler.srv import AICommand, AICommandResponse
# from drivers_ard_hmi.srv import SetStrategies

# from drivers_hmi.srv import SetStrategies

class AINode():
    def __init__(self):
        self.DepartmentName, self.PackageName = "ai", "scheduler"

        self.NODE = rospy.init_node(self.PackageName, log_level = rospy.DEBUG)
        rospy.Service("/ai/scheduler/command", AICommand, self.on_command)

        self.AI = RobotAI()
        # self.services = AIServices(self.DepartmentName, self.PackageName)

        rospy.loginfo("[AI] Ready.")
        # self.send_strategies()
        rospy.spin()

    def send_strategies(self):
        rospy.wait_for_service("/drivers/ard_hmi/set_strategies", timeout = 20) # TODO evaluate and put good timeout
        s = rospy.ServiceProxy("/drivers/ard_hmi/set_strategies", SetStrategies)
        s(self.AI.get_strategies())

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

    def on_command(self, req):
        if req.command == req.COMMAND_START:
            self.AI.start(req.strategy_name, AICommunication())
        elif req.command == req.COMMAND_HALT:
            self.AI.halt()
        return AICommandResponse()

'''
PACKAGE STARTING POINT HERE
'''
if __name__ == "__main__":
    node = AINode()
