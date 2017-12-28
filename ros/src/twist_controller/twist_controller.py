import rospy
from yaw_controller import YawController

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self):
        # TODO: Implement
        pass

    def control(self, twist_cmd, current_velocity):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        yaw_controller = YawController()
        steering = yaw_controller.get_steering(twist_cmd.twist.linear.x, twist_cmd.twist.angular.z, current_velocity.twist.linear.x)
        rospy.loginfo("inside control function, linear_velocity, angular_velocity, steering = %s, %s, %s", str(twist_cmd.twist.linear.x), str(twist_cmd.twist.angular.z), str(steering))
        return 0.2, 0., steering
