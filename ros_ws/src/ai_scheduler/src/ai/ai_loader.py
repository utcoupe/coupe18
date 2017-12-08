#!/usr/bin/python
import os
import rospy
import xml.etree.ElementTree as ET
from tasks import Strategy, ActionList, Action, Order


class AILoader():
    STRATEGIES_PATH = "1_strategies.xml"
    ACTIONS_PATH    = "2_actions.xml"
    ORDERS_PATH     = "3_orders.xml"

    def __init__(self):
        self.xml_dirpath = os.path.dirname(__file__) + "/../../def/"

    def get_strategies(self):
        xml_strategies = ET.parse(self.xml_dirpath + self.STRATEGIES_PATH).getroot()
        return [child.attrib["name"] for child in xml_strategies]

    def load(self, strategyname, communicator):
        orders = self._load_orders()
        return self._load_strategy(strategyname, self._load_actions(orders), orders, communicator)

    def _load_orders(self):
        xml_orders = ET.parse(self.xml_dirpath + self.ORDERS_PATH).getroot()

        orders = []
        for order_xml in xml_orders:
            orders.append(Order(order_xml))
        return orders

    def _load_actions(self, orders):
        xml_actions = ET.parse(self.xml_dirpath + self.ACTIONS_PATH).getroot()

        # actions_names = [action.attrib["ref"] for action in xml_actions]
        actions = []
        for action_xml in xml_actions:
            actions.append(Action(action_xml, actions, orders))
        return actions

    def _load_strategy(self, strategyname, actions, orders, communicator):
        xml_strategies = ET.parse(self.xml_dirpath + self.STRATEGIES_PATH).getroot()

        strategy_xml = xml_strategies.findall("strategy[@name='{}']".format(strategyname))
        if len(strategy_xml) == 1:
            return Strategy(strategy_xml[0], actions, orders, communicator)
        else:
            rospy.logerr("FAIL Too many or no strategy to load with the given name. Aborting.")
            return None
