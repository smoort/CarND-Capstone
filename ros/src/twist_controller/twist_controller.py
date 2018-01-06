import rospy
from yaw_controller import YawController
from lowpass import LowPassFilter
from pid import PID

GAS_DENSITY = 2.858
ONE_MPH = 0.44704
VEHICLE_MASS = rospy.get_param('~vehicle_mass', 1736.35)
ACCEL_LIMIT = rospy.get_param('~accel_limit', 1.)
DECEL_LIMIT = rospy.get_param('~decel_limit', -5)
WHEEL_RADIUS = rospy.get_param('~wheel_radius', 0.2413)
BRAKE_LIMIT = VEHICLE_MASS * DECEL_LIMIT * WHEEL_RADIUS

class Controller(object):
    def __init__(self):
        self.steering_low_pass_filter = LowPassFilter(0.80)
        self.throttle_low_pass_filter = LowPassFilter(0.80)
        self.throttle_pid = PID(0.5,-0.0002,0,0,1)

    def control(self, twist_cmd, current_velocity):

        velocity_diff = twist_cmd.twist.linear.x - current_velocity.twist.linear.x

        #rospy.loginfo("twist_controller current velocity, end_velocity, diff = %s, %s, %s", current_velocity.twist.linear.x, twist_cmd.twist.linear.x, velocity_diff)
        if velocity_diff > 0:
            throttle = (velocity_diff * 0.5 * ACCEL_LIMIT) - 0.02
            #throttle = self.throttle_pid.step(velocity_diff,0) - 0.02
            #throttle = self.throttle_pid.step(velocity_diff,1)
            #if throttle > 1:
            #    throttle = 1
            brake = 0.0
        elif velocity_diff < 0:
            throttle = 0.0 - 0.02
            #throttle = 0.0
            brake = velocity_diff * 0.1 * BRAKE_LIMIT
        else:
            throttle = 0.0 - 0.02
            brake = 0.0
            
        throttle = self.throttle_low_pass_filter.filter(throttle)
        
        yaw_controller = YawController()
        steering = yaw_controller.get_steering(twist_cmd.twist.linear.x, twist_cmd.twist.angular.z, current_velocity.twist.linear.x)
        steering = self.steering_low_pass_filter.filter(steering)
        
        #rospy.loginfo("twist_controller current velocity, end_velocity, throttle = %s, %s, %s, %s, %s", current_velocity.twist.linear.x, twist_cmd.twist.linear.x,throttle, brake, steering)
        #rospy.loginfo("throttle, brake, steering = %s, %s, %s", throttle, brake, steering)
        return throttle, brake, steering
        #return 0.0, 0.5, 0, 0
