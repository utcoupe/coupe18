#!/usr/bin/python2.7
# -*-coding:Utf-8 -*

class Enemy():
    """Enemy's datas"""

    def __init__(self):
        self.pos_history = []

    def addPos(self, pos, error):
        if len(self.pos_history) >= self.maxRectHistory:
            del self.pos_history[0]
        self.pos_history.append([pos,error])
    def setSize(self, width, height):
        pass
    
    def setOrientation(self, orientation):
        pass

    def speed(self):
        """Return the current speed of the enemy"""
        at, ax, ay = pos_history[-1][0]
        bt, bx, by = pos_history[-2][0]
        at=at-bt
        ax=ax-bx
        ay=ay-by
        return [ax/at,ay/at]

    def canItBeMe(self, point, error):
        pass

    def configure(self, prop):
        if prop is None:
            #Default values
            prop = {'maxPosHistory': '3'}
        self.maxPosHistory = int(prop['maxPosHistory'])


if __name__ == '__main__':
    enemy_tracker_node = EnemyTrackerNode()
    rospy.loginfo('Enemy tracker node started')
    #TODO Use rospy.get_param(name) instead
    enemy_tracker_node.configure(
        enemy_tracker_properties.EnemyTrackerProperties())
    rospy.spin()
