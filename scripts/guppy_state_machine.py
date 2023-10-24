#!/usr/bin/env python3
from math import pi, copysign, atan2, sin, cos, radians, degrees

import rospy
import smach

from std_srvs.srv import SetBool
from std_srvs.srv import Empty, EmptyResponse
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist


class Controller():
    enabled_ = False
    depth_ = 0.0
    depth_eps_ = 0.1
    surge_effort = 0.0

    def __init__(self):
        self.rate = rospy.Rate(10)

        rospy.Subscriber('depth', Float64, self.depth_callback)
        self.start_service_ = rospy.Service('~start', Empty, self.handle_start)

        rospy.loginfo('State machine waiting sevices')
        rospy.wait_for_service('/pid/switch')
        self.pid_enable_service = rospy.ServiceProxy('/pid/switch', SetBool)

        self.command_publisher_ = rospy.Publisher('teleop_command', Twist, queue_size=1) # teleop_command
        self.depth_sp_publisher = rospy.Publisher('/pid/depth_pid/setpoint', Float64, queue_size=1) # pid
        # self.depth_sp_publisher = rospy.Publisher('depth_pid/setpoint', Float64, queue_size=1) # pid

        self.state_change_time = rospy.Time.now()
        rospy.loginfo('State machine started')

    def handle_start(self, req):
        self.enabled_ = True
        return EmptyResponse()

    def submerge(self, depth):
        self.depth_sp_publisher.publish(depth)

    def emerge(self):
        self.depth_sp_publisher.publish(0)

    def switch_autopilot(self, en):
        resp = self.pid_enable_service(en)

    def depth_approached(self, d):
        if abs(d - self.depth_) < self.depth_eps_:
            return True
        else:
            return False

    def depth_callback(self, msg):
        self.depth_ = msg.data

    def move(self, x, y):
        cmd = Twist()
        cmd.linear.x = x
        cmd.linear.y = y
        self.command_publisher_.publish(cmd)


class Wait(smach.State):
    def __init__(self, c, pid_enabled=False):
        smach.State.__init__(self, outcomes=['start'])
        self.cntr = c
        self.pid_enabled = pid_enabled

    def execute(self, userdata):
        self.cntr.enabled_ = False
        self.cntr.switch_autopilot(self.pid_enabled)
        while not rospy.is_shutdown():
            if self.cntr.enabled_:
                return 'start'
            else:
                self.cntr.rate.sleep()


class Submerge(smach.State):
    def __init__(self, c, depth):
        smach.State.__init__(self, outcomes=['depth_approached'])
        self.cntr = c
        self.depth = depth

    def execute(self, userdata):
        self.cntr.state_change_time = rospy.Time.now()
        self.cntr.switch_autopilot(True)
        self.cntr.submerge(self.depth)

        while not rospy.is_shutdown():
            if self.cntr.depth_approached(self.depth):
                return 'depth_approached'
            else:
                self.cntr.rate.sleep()


class Emerge(smach.State):
    def __init__(self, c):
        smach.State.__init__(self, outcomes=['aborted'])
        self.cntr = c

    def execute(self, userdata):
        self.cntr.state_change_time = rospy.Time.now()
        self.cntr.switch_autopilot(True)
        self.cntr.emerge()
        while not rospy.is_shutdown():
            if self.cntr.depth_approached(0) or ((rospy.Time.now() - self.cntr.state_change_time) > rospy.Duration(10)):
                self.cntr.switch_autopilot(False)
                return 'aborted'
            else:
                self.cntr.rate.sleep()


class Move(smach.State):
    def __init__(self, c, effort_x, effort_y, time):
        smach.State.__init__(self, outcomes=['timeout'])
        self.cntr = c
        self.time = time
        self.effort_x = effort_x
        self.effort_y = effort_y

    def execute(self, userdata):
        self.cntr.switch_autopilot(True)
        self.cntr.state_change_time = rospy.Time.now()
        while not rospy.is_shutdown():
            if (rospy.Time.now() - self.cntr.state_change_time) > rospy.Duration(self.time):
                self.cntr.move(0.0, 0.0)
                return 'timeout'
            else:
                self.cntr.move(self.effort_x, self.effort_y)
                self.cntr.rate.sleep()


if __name__ == '__main__':
    rospy.init_node('state_machine')

    c = Controller()
    sm = smach.StateMachine(outcomes=['aborted'])
    with sm:
        smach.StateMachine.add('WAIT', Wait(c),
                               transitions={'start': 'SUBMERGE'})

        smach.StateMachine.add('SUBMERGE', Submerge(c, 0.5),
                               transitions={'depth_approached': 'MOVE_FORWARD1'})

        smach.StateMachine.add('MOVE_FORWARD1', Move(c, effort_x=0.1, effort_y=0, time=5),
                               transitions={'timeout': 'MOVE_LEFT'})

        smach.StateMachine.add('MOVE_LEFT', Move(c, effort_x=0, effort_y=0.2, time=5),
                               transitions={'timeout': 'MOVE_FORWARD2'})

        smach.StateMachine.add('MOVE_FORWARD2', Move(c, effort_x=0.1, effort_y=0, time=5),
                               transitions={'timeout': 'MOVE_RIGHT'})

        smach.StateMachine.add('MOVE_RIGHT', Move(c, effort_x=0, effort_y=-0.2, time=5),
                               transitions={'timeout': 'EMERGE'})

        smach.StateMachine.add('EMERGE', Emerge(c),
                               transitions={'aborted': 'WAIT'})

    rospy.sleep(rospy.Duration(1))
    outcome = sm.execute()
