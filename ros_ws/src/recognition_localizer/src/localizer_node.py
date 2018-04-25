#!/usr/bin/env python
import rospy
import tf
import tf2_ros
import math
from geometry_msgs.msg import Pose2D, TransformStamped
from ai_game_status import StatusServices
from processing_lidar_objects.msg import Obstacles
from recognition_localizer.srv import *
from recognition_objects_classifier.msg import *

class Localizer(object):
    def __init__(self):
        super(Localizer, self).__init__()

        rospy.init_node('localizer_node', anonymous=False)

        self._br = tf2_ros.TransformBroadcaster()

        # TODO : subscribe to all sources of info
        self._sub_asserv = rospy.Subscriber("/drivers/ard_asserv/pose2d", Pose2D,
                                          self.callback_asserv)
        self._lidar_sub = rospy.Subscriber("/processing/lidar_objects/obstacles", Obstacles, self.lidar_callback)
        self._srv_lidar_pos = rospy.Service("/recognition/localizer/lidar_pos", Correct_pos, self.callback_lidar_pos)
        self._data_asserv = None

        self._segments = {'map': [], 'unknown': []}
        self._to_process = {'rects': [], 'circles': [], 'segments': []}
        self.LIMIT = 0.1
        self.TABLE_X = 3
        self.TABLE_Y = 2

        # Tell ai/game_status the node initialized successfuly.
        StatusServices("recognition", "localizer").ready(True)

    def lidar_callback(self, data):
        self._to_process['segments'] = []
        self._to_process['segments'] = [SegmentObstacleStamped(data.header, s) for s in data.segments]

    def callback_asserv(self, data):
        self._data_asserv = data

    def clear_objects(self):
        self._segments['map'] = []
        self._segments['unknown'] = []

    def callback_lidar_pos(self, info):
        return self.process_segments()


    def process_segments(self):
        sum_x = 0
        sum_y = 0
        count_x = 0
        count_y = 0
        gap_x = 0
        gap_y = 0
        count = 0
        sum = 0
        ecart = 0
        rospy.loginfo('----------')
        for segment in self._to_process['segments']:
            seg = segment_properties(segment)
            #rospy.loginfo('center_y : ' + str(seg.center_y) + ' norm : ' + str(seg.norm))
            if (seg.y < 0.1 and seg.y > -0.1 and seg.center_y < self.LIMIT and seg.center_y > -self.LIMIT and seg.norm > 0.15):
                rospy.loginfo('seg y : ' + str(seg.y) + ' seg center y : ' + str(seg.center_y) + ' norm : ' + str(seg.norm))
                e = math.acos(seg.x / (seg.norm))
                e -= math.pi
                if seg.y > 0:
                    e = -e
                rospy.loginfo('e : ' + str(e))
                sum += e
                count += 1
            if (seg.x < 0.1 and seg.x > -0.1 and seg.center_x < self.LIMIT and seg.center_x > -self.LIMIT and seg.norm > 0.15):
                rospy.loginfo('seg x : ' + str(seg.x) + ' seg center x : ' + str(seg.center_x) + ' norm : ' + str(seg.norm))
                e = math.acos(seg.y / (seg.norm))
                if seg.x < 0:
                    e = -e
                rospy.loginfo('e : ' + str(e))
                sum += e
                count += 1
            # if (seg.x < 0.05 or seg.y < 0.05) and seg.norm > 0.2:
            #     if (seg.center_x < self.LIMIT and seg.center_x > -self.LIMIT) \
            #             or (seg.center_x < self.TABLE_X + self.LIMIT and seg.center_x > self.TABLE_X - self.LIMIT):
            #         sum_x += seg.center_x
            #         count_x += 1a
            #     elif (seg.center_y < self.LIMIT and seg.center_y > -self.LIMIT) :
            #         #if (seg.center_y < self.TABLE_Y + self.LIMIT and seg.center_y > self.TABLE_Y - self.LIMIT):
            #          #   seg.center_y -= self.TABLE_Y
            #         rospy.loginfo('OK center_y : ' + str(seg.center_y))
            #         sum_y += seg.center_y
            #         count_y += 1
        if count_x > 0:
            gap_x = sum_x / count_x
        if count_y > 0:
            gap_y = sum_y / count_y
        if count > 0:
            ecart = math.degrees(sum / count)

        self._to_process['segments'] = []
        # rospy.loginfo('gap_x : ' + str(gap_x) + ' ;gap_y : ' + str(gap_y))
        # ret = 'gap_x : ' + str(gap_x) + ';gap_y : ' + str(gap_y) + \
        #         ' count_x : ' + str(count_x) + ' ;count_y : ' + str(count_y)
        ret = 'Ecart angulaire : ' + str(ecart) + " count : " + str(count)
        return ret

    def run(self):
        rospy.loginfo("Localizer node started")
        rate = rospy.Rate(20)
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

class segment_properties:
    def __init__(self, seg):
        segment = seg.segment
        self.x = segment.first_point.x - segment.last_point.x
        self.y = segment.first_point.y - segment.last_point.y
        self.vect = [self.x, self.y]
        self.norm = math.sqrt(self.x * self.x + self.y * self.y)
        self.center_x = (segment.first_point.x + segment.last_point.x) / 2
        self.center_y = (segment.first_point.y + segment.last_point.y) / 2

if __name__ == '__main__':
    loc = Localizer()
    loc.run()
