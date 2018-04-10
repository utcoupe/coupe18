import math

import actionlib
import rospy
import tf2_ros
from actuators_abstract import ActuatorsAbstract
from drivers_ax12.msg import Ax12CommandGoal, Ax12CommandAction
from geometry_msgs.msg import TransformStamped, PointStamped
from movement_actuators.msg import ArmAction


class ActuatorsArm(ActuatorsAbstract):
    def __init__(self):
        ActuatorsAbstract.__init__(self, action_name='arm',
                                   action_type=ArmAction)

        self.ORIGIN_FRAME = "arm_origin"

        self._broadcaster = tf2_ros.StaticTransformBroadcaster()
        self._buffer = tf2_ros.Buffer()
        self._listener = tf2_ros.TransformListener(self._buffer)

        self._motor1 = {
            'id': 3,
            'center': 450,
            'length': 0.117,
            'min': 150,
            'max': 750
        }

        self._motor2 = {
            'id': 1,
            'center': 512,
            'length': 0.054,
            'min': 200,
            'max': 800
        }

        self._max_range = self._motor1['length'] + self._motor2['length']

        self.ORIGIN_X = 0.0
        self.ORIGIN_Y = 0.0

        self._pub_static_transform(self.ORIGIN_X, self.ORIGIN_Y)

        self._client = actionlib.ActionClient('/drivers/ax12', Ax12CommandAction)
        self._client.wait_for_server(rospy.Duration(10))

        rospy.logdebug('Dispatcher server found')

    def _process_action(self, goal):
        point = PointStamped()
        point.header.frame_id = goal.frame_id
        point.header.stamp = rospy.Time.now()
        point.point.x = goal.x
        point.point.y = goal.y

        try:
            point = self._buffer.transform_full(point,
                                                target_frame=self.ORIGIN_FRAME,
                                                target_time=rospy.Time(),
                                                fixed_frame=goal.frame_id)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logerr("Can't transform given position into origin frame : %s" % e)
            return False

        self._buffer.transform(point, self.ORIGIN_FRAME)

        x_rel = point.point.x
        y_rel = point.point.y

        if math.sqrt(x_rel * x_rel + y_rel * y_rel) > self._max_range:
            rospy.logerr("Can't reach goal, max range of %s" % self._max_range)
            return False

        q2_toacos = (math.pow(x_rel, 2) + math.pow(y_rel, 2)
                     - math.pow(self._motor1['length'], 2)
                     - math.pow(self._motor2['length'], 2)) / (2 * self._motor1['length'] * self._motor2['length'])

        if abs(q2_toacos) > 1:
            rospy.logerr("Can't reach goal, not on possible trajectory")
            return False

        q2 = math.acos(q2_toacos)

        if y_rel == 0:
            q1 = 0
        elif x_rel == 0:
            q1 = y_rel / abs(y_rel) * math.pi / 2
        else:
            q1 = math.atan(y_rel / x_rel)

        q1 -= math.atan((self._motor2['length'] * math.sin(q2)) /
                        (self._motor1['length'] + self._motor2['length'] * math.cos(q2)))

        if q2 > math.pi:
            q2 = -(2 * math.pi - q2)

        if q1 > math.pi:
            q1 = -(2 * math.pi - q1)

        rospy.logdebug('Computed angles : %f and %f' % (q1, q2))

        q2_value = self._motor2['center'] + int((q2 / (15.0 / 18.0 * math.pi)) * 512.0)
        q1_value = self._motor1['center'] + int((q1 / (15.0 / 18.0 * math.pi)) * 512.0)

        if q2_value >= self._motor2['max'] or q2_value <= self._motor2['min']:
            rospy.logerr("Value not in range for motor 2: %d" % q2_value)

            return False

        if q1_value >= self._motor1['max'] or q1_value <= self._motor1['min']:
            rospy.logerr("Value not in range for motor 1: %d" % q1_value)
            return False

        rospy.logdebug('Sending positions %d and %d' % (q1_value, q2_value))

        goal1 = Ax12CommandGoal()
        goal1.mode = goal1.JOINT
        goal1.motor_id = self._motor1['id']
        goal1.position = q1_value

        self._client.send_goal(goal1)

        goal2 = Ax12CommandGoal()
        goal2.mode = goal2.JOINT
        goal2.motor_id = self._motor2['id']
        goal2.position = q2_value

        self._client.send_goal(goal2)

        return True

    def _pub_static_transform(self, x, y):
        rospy.logdebug("Publishing the static transform")

        tr = TransformStamped()

        tr.header.stamp = rospy.Time.now()
        tr.header.frame_id = "/robot"
        tr.child_frame_id = self.ORIGIN_FRAME

        tr.transform.translation.x = x
        tr.transform.translation.y = y
        tr.transform.translation.z = 0
        tr.transform.rotation.w = 1

        self._broadcaster.sendTransform(tr)
