#!/usr/bin/python2.7
# -*-coding:Utf-8 -*

import rospy


class ActuatorsNode():
    """Dispatch commands from AI to the correct node"""

    def __init__(self):
        self.node = rospy.init_node('actuators')


if __name__ == '__main__':
    actuators_node = ActuatorsNode()
    rospy.spin()
