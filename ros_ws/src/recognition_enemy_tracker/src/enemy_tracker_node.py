#!/usr/bin/python2.7
# -*-coding:Utf-8 -*

from processing_belt_interpreter.msg import BeltFiltered
from geometry_msgs.msg import RectangleStamped
import rospy
from libtools import Rect


class EnemyTrackerNode():
    """Track enemies"""
    
    def __init__(self):
        self._node = rospy.init_node('enemy_tracker')
        self._namespace = '/recognition/enemy_tracker/'
        self._belt_sub = rospy.Subscriber(
            '/processing/belt_interpreter/points', BeltFiltered, self.callbackPos)

    def importPoint(self, beltData):
        rect = {}
        for rect in belt_info.unknow_rects
            # TODO check referentiel
            rect.append(Rect(rect.x, rect.y, rect.w, rect.h))
        trackEnemies(rect)

    def trackEnemies(self, rectList):
        pass

if __name__ == '__main__':
    enemy_tracker_node = EnemyTrackerNode()
    rospy.loginfo('Enemy tracker node started')
    rospy.spin()
