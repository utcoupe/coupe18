
import actionlib
import rospy
from actuators_abstract import ActuatorsAbstract
from movement_actuators.msg import BarrelAction, BarrelResult, DispatchAction, DispatchGoal
import drivers_ard_others.msg

from actionlib.action_client import CommState

import matplotlib.pyplot as plt

class Color:
    UNKNOWN = 0
    ORANGE = 1
    GREEN = 2

class ActuatorsBarrel(ActuatorsAbstract):
    def __init__(self):

        self.COLOR_SENSOR_TOPIC = '/drivers/ard_others/color'
        self.BARREL_NAME = 'barrel' # name in the dispatcher
        self.PRESET_BIN = 'BIN'
        self.PRESET_NORMAL = 'HIGH'
        self.PRESET_CANON = 'CANON'

        self.ORANGE_HUE = 15
        self.GREEN_HUE = 120
        self.HUE_MARGIN = 15

        self.SATURATION_TRESH = 90

        self.BALL_COUNT = 8

        self._client = actionlib.SimpleActionClient('/movement/actuators/dispatch', DispatchAction)
        self._client.wait_for_server(rospy.Duration(10))

        self._curr_color = Color.UNKNOWN

        self._color_client = rospy.Subscriber(self.COLOR_SENSOR_TOPIC, drivers_ard_others.msg.Color, self._color_callback)

        self._team_color = Color.ORANGE if rospy.get_param('/current_team', 'orange') == 'orange' else Color.GREEN

        ActuatorsAbstract.__init__(self,
                                   action_name='barrel',
                                   action_type=BarrelAction)

        self._is_running = False
        self._doing_back = False
        self._doing_forth = False

        self._curr_goal_id = None

        self._timer = None

    def _process_action(self, goal, goal_id):
        if self._is_running:
            rospy.logerr("Received a goal but another one is in process !")
            return False

        self._is_running = True
        self._curr_goal_id = goal_id

        if goal.timeout > 0:
            self._timer = rospy.Timer(rospy.Duration(goal.timeout), self._trigger_timeout, oneshot=True)
        else:
            rospy.logwarn('No timeout is set, the action might take forever')

        if not goal.sort:
            rospy.logdebug('Starting goal chain with no sort')
            self._start_goal_chain(self.PRESET_CANON)

        return True

    def _forth_done_cb(self, state, result):
        rospy.logdebug('forth_done')
        self._doing_forth = False

        g = DispatchGoal()
        g.name = self.BARREL_NAME
        g.order = 'JOINT'
        g.preset = self.PRESET_NORMAL

        self._doing_back=True
        self._client.send_goal(g, done_cb=self._back_done_cb)

    def _back_done_cb(self, state, result):
        rospy.logdebug('back_done')
        self._finish_action(result.success)

    def _trigger_timeout(self, event):
        # TODO cancel goals
        self._finish_action(False)

    def _color_callback(self, msg):

        if msg.hue <= self.ORANGE_HUE + self.HUE_MARGIN \
            and msg.hue >= self.ORANGE_HUE - self.HUE_MARGIN\
                and msg.saturation > self.SATURATION_TRESH:
            self._curr_color = Color.ORANGE

        elif msg.hue <= self.GREEN_HUE + self.HUE_MARGIN \
            and msg.hue >= self.GREEN_HUE - self.HUE_MARGIN \
                and msg.saturation > self.SATURATION_TRESH:
            self._curr_color = Color.GREEN

        else:
            self._curr_color = Color.UNKNOWN

        if self._is_running and \
            not self._doing_back and \
            not self._doing_forth \
            and self._curr_color != Color.UNKNOWN:

            if self._team_color != self._curr_color:
                rospy.logfatal('Starting goal chain for ennemy color ball (h: %d, s: %d, v %d)'
                               % (msg.hue, msg.saturation, msg.lightness))
                self._start_goal_chain(self.PRESET_BIN)
            else:
                rospy.logfatal('Starting goal chain for ally color balll (h: %d, s: %d, v %d)'
                               % (msg.hue, msg.saturation, msg.lightness))
                self._start_goal_chain(self.PRESET_CANON)


    def _start_goal_chain(self, preset):
        g = DispatchGoal()
        g.name = self.BARREL_NAME
        g.order = 'JOINT'
        g.preset = preset

        self._doing_forth = True
        self._client.send_goal(g, done_cb=self._forth_done_cb)

    def _finish_action(self, success):
        self._is_running = False
        self._doing_forth = False
        self._doing_back = False

        self._client.stop_tracking_goal()

        if self._timer:
            self._timer.shutdown()
            self._timer = None

        self._action_reached(self._curr_goal_id, success, BarrelResult(success=success))

        self._curr_goal_id = None