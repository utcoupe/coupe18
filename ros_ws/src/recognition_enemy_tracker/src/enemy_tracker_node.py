#!/usr/bin/python2.7
# -*-coding:Utf-8 -*

from processing_belt_interpreter.msg import BeltFiltered, RectangleStamped
import rospy
from libtools import Rect


class EnemyTrackerNode():
    """Track enemies"""
    
    def __init__(self):
        self._node = rospy.init_node('enemy_tracker')
        self._namespace = '/recognition/enemy_tracker/'
        self._belt_sub = rospy.Subscriber(
            '/processing/belt_interpreter/points', BeltFiltered, self.importPoint)
        self.maxRectHistory = 3
        self.rect=[]


    def importPoint(self, beltData):
        rect = []
        for rect in beltData.unknown_rects:
            # TODO check referentiel
            rect.append(Rect(rect.x, rect.y, rect.w, rect.h))
        self.saveRect(rect)
        self.trackEnemies()

    def saveRect(rect):
        while len(self.rect) >= maxRectHistory:
            del self.rect[0]
        self.rect.append(rect)


    def trackEnemies(self):
        pass

if __name__ == '__main__':
    enemy_tracker_node = EnemyTrackerNode()
    rospy.loginfo('Enemy tracker node started')
    rospy.spin()
