#!/usr/bin/python2.7
# -*-coding:Utf-8 -*

import rospy

class EnemyTrackerNode():
    """Track enemies"""

    def __init__(self):
        self._node = rospy.init_node('enemy_tracker')
        self._namespace = '/recognition/enemy_tracker/'

if __name__ == '__main__':
    enemy_tracker_node = EnemyTrackerNode()
    rospy.loginfo('Enemy tracker node started')
    rospy.spin()
