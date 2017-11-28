#!/usr/bin/python2.7
# -*-coding:Utf-8 -*

from processing_belt_interpreter.msg import BeltFiltered, RectangleStamped
from libtools import Rect
import rospy


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
        rects = []
        for rect in beltData.unknown_rects:
            # TODO check referentiel
            rects.append(Rect(rect.x, rect.y, rect.w, rect.h))
        self.saveRect(rects)
        self.trackEnemies()

    def saveRect(self, rect):
        while len(self.rect) >= self.maxRectHistory:
            del self.rect[0]
        self.rect.append(rect)

    def trackEnemies(self):
        pass

if __name__ == '__main__':
    enemy_tracker_node = EnemyTrackerNode()
    rospy.loginfo('Enemy tracker node started')
    rospy.spin()
