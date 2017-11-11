#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Pose2D


class Localizer(object):
    def __init__(self):
        super(Localizer, self).__init__()

        rospy.init_node('localizer', anonymous=False)

        self.pub = rospy.Publisher('/recognition/localizer', Pose2D,
                                   queue_size=10)

        # TODO : subscribe to all sources of info
        self.subAsserv = rospy.Subscriber("/robot/pose2d", Pose2D,
                                          self.callbackAsserv)

        self.dataAsserv = None

        rospy.loginfo("Localizer node initialized")

    def callbackAsserv(self, data):
        self.dataAsserv = data

    def run(self):
        rospy.loginfo("Localizer node started")
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            data = self.calculate()
            if data:
                self.pub.publish(self.calculate())
            rate.sleep()

    #  TODO : merge all data gathered
    def calculate(self):
        return self.dataAsserv


if __name__ == '__main__':
    loc = Localizer()
    loc.run()
