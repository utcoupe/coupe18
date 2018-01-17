#!/usr/bin/python2.7
# -*-coding:Utf-8 -*

from processing_belt_interpreter.msg import BeltFiltered, RectangleStamped
from libtools import Rect
from enemy_tracker_tracker import EnemiesData
from tf import TransformListener
import enemy_tracker_properties
import rospy


class EnemyTrackerNode():
    """Track enemies"""

    def __init__(self):
        self._node = rospy.init_node('enemy_tracker')
        self._namespace = '/recognition/enemy_tracker/'
        self._belt_sub = rospy.Subscriber(
            '/processing/belt_interpreter/points', BeltFiltered, self.importPoint)
        self.configure(None)
        self._publisher = rospy.Publisher('{}enemies'.format(self._namespace))
        self.rect = []
        self.data = []
        self._tf = TransformListener()

    def importPoint(self, beltData):
        rects = []
        for rect in beltData.unknown_rects:
            try:
                rect = self._tl.transformPose("/map", rect)
            except Exception as e:
                rospy.logwarn("Frame map may not exist, cannot process sensor data : {}".format(e))
                break
            rects.append(Rect(rect.x, rect.y, rect.w, rect.h, rect.header.stamp))
        if(len(rects)==0)
            return False
        self.saveRect(rects)
        self.trackEnemies()
        return True

    def saveRect(self, rect):
        if len(self.rect) >= self.maxRectHistory:
            del self.rect[0]
        self.rect.append(rect)

    def trackEnemies(self):
        pass
    
    def updateData(auto_detect_change = False):
        pass
    
    def configure(self, prop):
        if prop is None:
            #Default values
            prop = {'maxRectHistory': '3','maxEnemiesVelocity': '0.5'}
        self.maxRectHistory = int(prop['maxRectHistory'])
        self.maxEnemiesVelocity = float(prop['maxEnemiesVelocity'])

if __name__ == '__main__':
    enemy_tracker_node = EnemyTrackerNode()
    rospy.loginfo('Enemy tracker node started')
    #TODO Use rospy.get_param(name) instead
    enemy_tracker_node.configure(enemy_tracker_properties.EnemyTrackerProperties())
    rospy.spin()
