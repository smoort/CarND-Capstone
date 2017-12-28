import rospy
from math import atan

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
MIN_SPEED = 0

class YawController(object):
    def __init__(self):
        self.wheel_base = WHEEL_BASE
        self.steer_ratio = STEER_RATIO
        self.min_speed = MIN_SPEED
        self.max_lat_accel = MAX_LAT_ACCEL

        self.min_angle = -MAX_STEER_ANGLE
        self.max_angle = MAX_STEER_ANGLE


    def get_angle(self, radius):
        angle = atan(self.wheel_base / radius) * self.steer_ratio
        #rospy.loginfo("inside yaw_controller get_angle, angle = %s", str(angle))
        return max(self.min_angle, min(self.max_angle, angle))

    def get_steering(self, linear_velocity, angular_velocity, current_velocity):
        angular_velocity = current_velocity * angular_velocity / linear_velocity if abs(linear_velocity) > 0. else 0.

        if abs(current_velocity) > 0.1:
            max_yaw_rate = abs(self.max_lat_accel / current_velocity);
            angular_velocity = max(-max_yaw_rate, min(max_yaw_rate, angular_velocity))

        return self.get_angle(max(current_velocity, self.min_speed) / angular_velocity) if abs(angular_velocity) > 0. else 0.0;
