#!/usr/bin/python
import os
import rospy
import xml.etree.ElementTree as ET
from tasks import Strategy, ActionList, Action, Order

from memory_definitions.srv import GetDefinition

class AILoader():
    STRATEGIES_FILE = "1_strategies.xml"
    ACTIONS_FILE    = "2_actions.xml"
    ORDERS_FILE     = "3_orders.xml"

    def __init__(self):
        pass# self.xml_dirpath = os.path.dirname(__file__) + "/../../def/"

    def get_strategies(self):
        strat_path = self._get_path(self.STRATEGIES_FILE)
        xml_strategies = ET.parse(strat_path).getroot()
        return [child.attrib["name"] for child in xml_strategies]

    def load(self, strategyname, communicator):
        orders = self._load_orders()
        return self._load_strategy(strategyname, self._load_actions(orders), orders, communicator)

    def _load_orders(self):
        xml_orders = ET.parse(self._get_path(self.ORDERS_FILE)).getroot()

        orders = []
        for order_xml in xml_orders:
            orders.append(Order(order_xml))
        return orders

    def _load_actions(self, orders):
        xml_actions = ET.parse(self._get_path(self.ACTIONS_FILE)).getroot()

        # actions_names = [action.attrib["ref"] for action in xml_actions]
        actions = []
        for action_xml in xml_actions:
            actions.append(Action(action_xml, actions, orders))
        return actions

    def _load_strategy(self, strategyname, actions, orders, communicator):
        xml_strategies = ET.parse(self._get_path(self.STRATEGIES_FILE)).getroot()

        strategy_xml = xml_strategies.findall("strategy[@name='{}']".format(strategyname))
        if len(strategy_xml) == 1:
            return Strategy(strategy_xml[0], actions, orders, communicator)
        else:
            rospy.logerr("FAIL Too many or no strategy to load with name '{}'. Aborting.".format(strategyname))
            return None

    def _get_path(self, filename):
        get_def = rospy.ServiceProxy('/memory/definitions/get', GetDefinition)

        try:
            get_def.wait_for_service(10)
            res = get_def('ai/' + filename)
            if not res.success:
                rospy.logerr("Error when fetching '{}' definition file".format(filename))
            return res.path
        except rospy.ServiceException as exc:
            rospy.logerr("Unhandled error while getting def file: {}".format(str(exc)))
            raise Exception()
