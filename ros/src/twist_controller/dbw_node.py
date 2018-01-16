#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from dbw_mkz_msgs.msg import ThrottleCmd, SteeringCmd, BrakeCmd, SteeringReport
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import Vector3
import math

from twist_controller import Controller


'''
You can build this node only after you have built (or partially built) the `waypoint_updater` node.

You will subscribe to `/twist_cmd` message which provides the proposed linear and angular velocities.
You can subscribe to any other message that you find important or refer to the document for list
of messages subscribed to by the reference implementation of this node.

One thing to keep in mind while building this node and the `twist_controller` class is the status
of `dbw_enabled`. While in the simulator, its enabled all the time, in the real car, that will
not be the case. This may cause your PID controller to accumulate error because the car could
temporarily be driven by a human instead of your controller.

We have provided two launch files with this node. Vehicle specific values (like vehicle_mass,
wheel_base) etc should not be altered in these files.

We have also provided some reference implementations for PID controller and other utility classes.
You are free to use them or build your own.

Once you have the proposed throttle, brake, and steer values, publish it on the various publishers
that we have created in the `__init__` function.

'''

VEHICLE_MASS = rospy.get_param('~vehicle_mass', 1736.35)
FUEL_CAPACITY = rospy.get_param('~fuel_capacity', 13.5)
BRAKE_DEADBAND = rospy.get_param('~brake_deadband', .1)
DECEL_LIMIT = rospy.get_param('~decel_limit', -5)
ACCEL_LIMIT = rospy.get_param('~accel_limit', 1.)
WHEEL_RADIUS = rospy.get_param('~wheel_radius', 0.2413)
WHEEL_BASE = rospy.get_param('~wheel_base', 2.8498)
STEER_RATIO = rospy.get_param('~steer_ratio', 14.8)
MAX_LAT_ACCEL = rospy.get_param('~max_lat_accel', 3.)
MAX_STEER_ANGLE = rospy.get_param('~max_steer_angle', 8.)

class DBWNode(object):
    def __init__(self):
        rospy.init_node('dbw_node')

      
        self.twist_cmd = TwistStamped()
        self.current_velocity = TwistStamped()
        self.dbw_enabled = False
        self.velocity_cte = 0

        self.steer_pub = rospy.Publisher('/vehicle/steering_cmd',
                                         SteeringCmd, queue_size=1)
        self.throttle_pub = rospy.Publisher('/vehicle/throttle_cmd',
                                            ThrottleCmd, queue_size=1)
        self.brake_pub = rospy.Publisher('/vehicle/brake_cmd',
                                         BrakeCmd, queue_size=1)

        self.controller = Controller()

        rospy.Subscriber('/twist_cmd', TwistStamped, self.twist_cb, queue_size=1)
        rospy.Subscriber('/current_velocity', TwistStamped, self.current_velocity_cb, queue_size=1)
        rospy.Subscriber('/vehicle/dbw_enabled', Bool, self.dbw_enabled_cb, queue_size=1)

        self.loop()

    def twist_cb(self, TwistMsg):
        #rospy.loginfo("inside twist_cb")
        self.twist_cmd = TwistMsg
        
    def current_velocity_cb(self, TwistMsg):
        #rospy.loginfo("inside current_velocity_cb")
        self.current_velocity = TwistMsg
        
    def dbw_enabled_cb(self, Bool):
        #rospy.loginfo("inside dbw_enabled_cb")
        self.dbw_enabled = Bool.data
        rospy.loginfo("dbw_enabled = %s", str(self.dbw_enabled))
        
    def loop(self):
        rate = rospy.Rate(50) # 50Hz
        while not rospy.is_shutdown():
            throttle, brake, steering = self.controller.control(self.twist_cmd, self.current_velocity)
            if self.dbw_enabled:
                self.publish(throttle, brake, steering)
            rate.sleep()

    def publish(self, throttle, brake, steer):
        tcmd = ThrottleCmd()
        tcmd.enable = True
        tcmd.pedal_cmd_type = ThrottleCmd.CMD_PERCENT
        tcmd.pedal_cmd = throttle
        self.throttle_pub.publish(tcmd)

        scmd = SteeringCmd()
        scmd.enable = True
        scmd.steering_wheel_angle_cmd = steer
        self.steer_pub.publish(scmd)

        bcmd = BrakeCmd()
        bcmd.enable = True
        bcmd.pedal_cmd_type = BrakeCmd.CMD_TORQUE
        bcmd.pedal_cmd = brake
        self.brake_pub.publish(bcmd)


if __name__ == '__main__':
    DBWNode()
