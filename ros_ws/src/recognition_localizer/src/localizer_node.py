#!/usr/bin/env python
import rospy
import tf
import tf2_ros
from geometry_msgs.msg import Pose2D, TransformStamped


class Localizer(object):
    def __init__(self):
        super(Localizer, self).__init__()

        rospy.init_node('localizer_node', anonymous=False)

        self._br = tf2_ros.TransformBroadcaster()

        # TODO : subscribe to all sources of info
        self._sub_asserv = rospy.Subscriber("/drivers/ard_asserv/pose2d", Pose2D,
                                          self.callback_asserv)

        self._data_asserv = None

        status_services = self._get_status_services("recognition", "localizer")
        status_services.ready(True) # Tell ai/game_status the node initialized successfuly.

    def _get_status_services(self, ns, node_name, arm_cb=None, status_cb=None):
        import sys, os
        sys.path.append(os.environ['UTCOUPE_WORKSPACE'] + '/ros_ws/src/ai_game_status/')
        from init_service import StatusServices
        return StatusServices(ns, node_name, arm_cb, status_cb)

    def callback_asserv(self, data):
        self._data_asserv = data

    def run(self):
        rospy.loginfo("Localizer node started")
        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            data = self.calculate()
            if data:
                t = TransformStamped()

                t.header.stamp = rospy.Time.now()
                t.header.frame_id = "map"
                t.child_frame_id = "robot"
                t.transform.translation.x = data.x
                t.transform.translation.y = data.y
                t.transform.translation.z = 0.0
                q = tf.transformations.quaternion_from_euler(0, 0, data.theta)
                t.transform.rotation.x = q[0]
                t.transform.rotation.y = q[1]
                t.transform.rotation.z = q[2]
                t.transform.rotation.w = q[3]

                self._br.sendTransform(t)

            rate.sleep()

    #  TODO : merge all data gathered
    def calculate(self):
        return self._data_asserv

if __name__ == '__main__':
    loc = Localizer()
    loc.run()
