#!/bin/usr/python
import os, time
from ai_classes import Task, TaskStatus, Strategy, Action, Order
import xml.etree.ElementTree as ET
from timer import *

class RobotAI():
    def __init__(self, strategyname):
        self.xml_dirpath = os.path.dirname(__file__) + "/../Definitions/"
        self.AvailableStrategies = None # loaded later (TODO Populate! used to be sent to the robot's onboard display)

        orders = self.loadOrders()
        self.Strategy = self.loadStrategy(strategyname, self.loadActions(orders), orders)
        self.Timer = GameTimer()

    def loadOrders(self):
        self.XML_ORDERS = ET.parse(self.xml_dirpath + "Orders_simple.xml").getroot()

        orders = []
        for order_xml in self.XML_ORDERS:
            orders.append(Order(order_xml))
        return orders

    def loadActions(self, orders):
        self.XML_ACTIONS = ET.parse(self.xml_dirpath + "Actions_simple.xml").getroot()

        actionnames = [action.attrib["ref"] for action in self.XML_ACTIONS]
        actions = []
        for action_xml in self.XML_ACTIONS:
            actions.append(Action(action_xml, actions, orders))
        return actions

    def loadStrategy(self, strategyname, actions, orders):
        self.XML_STRATEGIES = ET.parse(self.xml_dirpath + "Strategies_simple.xml").getroot()
        self.AvailableStrategies = [child.attrib["name"] for child in self.XML_STRATEGIES]

        strategy_xml = self.XML_STRATEGIES.findall("strategy[@name='" + strategyname + "']")
        if len(strategy_xml) == 1:
            return Strategy(strategy_xml[0], actions, orders)
        else:
            print "FAIL Too many or no strategy to load with the given name. Aborting."
            return None
