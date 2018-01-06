#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32
from geometry_msgs.msg import TwistStamped

import math
import tf

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 50 # Number of waypoints we will publish. You can change this number
WHEEL_BASE = rospy.get_param('~wheel_base', 2.8498)
LATENCY = 0.200     # 200ms latency assumed
DECEL_LIMIT = rospy.get_param('~decel_limit', -5)
ACCEL_LIMIT = rospy.get_param('~accel_limit', 1.)

class WaypointUpdater(object):

    def __init__(self):
    
        rospy.init_node('waypoint_updater')

        self.current_position = []
        self.current_position_available = False
        self.base_wps = []
        self.traffic_light_state = -1
        self.current_velocity = TwistStamped()
        
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/current_velocity', TwistStamped, self.current_velocity_cb)
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=10)

        prev_closes_wp_idx = 0
        r = rospy.Rate(10)  # 10 Hz
        while True:
            if self.current_position_available:
                prev_x = self.current_position[0]
                prev_y = self.current_position[1]
                new_closest_wp_idx = self.get_closest_waypoint(self.current_position, prev_closes_wp_idx, self.base_wps)
                final_wps = self.get_forward_waypoints(new_closest_wp_idx, self.current_velocity, self.base_wps)
                #rospy.loginfo('current, final positions = %s, %s', str(self.current_position), str(final_wps[0].pose.pose.position))
                final_lane = Lane()
                final_lane.header.frame_id = '/world'
                final_lane.header.stamp = rospy.Time.now()
                final_lane.waypoints = final_wps
                self.final_waypoints_pub.publish(final_lane)
                prev_closes_wp_idx = new_closest_wp_idx
            r.sleep()
        
        rospy.spin()

    def pose_cb(self, msg):
        #rospy.loginfo("inside current cb")
        current_position = []
        current_position.append(msg.pose.position.x)
        current_position.append(msg.pose.position.y)
        current_position.append(msg.pose.position.z)
        quat = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
        euler = tf.transformations.euler_from_quaternion(quat)
        current_position.append(euler[2])
        self.current_position = current_position
        self.current_position_available = True
        return

    def waypoints_cb(self, waypoints):
        #rospy.loginfo("inside base cb")
        base_wps = []
        for i in range(len(waypoints.waypoints)):
            base_wp  = []
            base_wp.append(waypoints.waypoints[i].pose.pose.position.x)
            base_wp.append(waypoints.waypoints[i].pose.pose.position.y)
            base_wp.append(waypoints.waypoints[i].pose.pose.position.z)
            base_wp.append(waypoints.waypoints[i].twist.twist.linear.x)
            if i == 0:
                dist_from_prev_point = math.sqrt((base_wp[0] - waypoints.waypoints[len(waypoints.waypoints)-1].pose.pose.position.x) ** 2
                                                +(base_wp[1] - waypoints.waypoints[len(waypoints.waypoints)-1].pose.pose.position.y) ** 2)
                yaw = math.atan2((base_wp[1] - waypoints.waypoints[len(waypoints.waypoints)-1].pose.pose.position.y),
                                 (base_wp[0] - waypoints.waypoints[len(waypoints.waypoints)-1].pose.pose.position.x))
            else:
                dist_from_prev_point = math.sqrt((base_wp[0] - base_wps[i-1][0]) ** 2
                                                +(base_wp[1] - base_wps[i-1][1]) ** 2)
                yaw = math.atan2((base_wp[1] - base_wps[i-1][1]), (base_wp[0] - base_wps[i-1][0]))
            base_wp.append(dist_from_prev_point)
            base_wp.append(yaw)
            base_wps.append(base_wp)
        self.base_wps = base_wps
        return
        
    def current_velocity_cb(self, TwistMsg):
        #rospy.loginfo("inside current_velocity_cb")
        self.current_velocity = TwistMsg
        if self.current_velocity.twist.linear.x < 0:
            self.current_velocity.twist.linear.x = 0
            
    def traffic_cb(self, traffic_light_state):
        self.traffic_light_state = traffic_light_state.data
        #rospy.loginfo("inside waypoint_updater traffic_cb, light state = %s", self.traffic_light_state)
        return
        
    def get_closest_waypoint(self, current_position, prev_closest_wp_idx, base_wps):
        # Finds the closest waypoint ahead of the vehicle's current position and direction

        closest_point = []
        closest_dist = 9999999999
        closest_point_not_found = True
        i = prev_closest_wp_idx
        new_closest_wp_idx = prev_closest_wp_idx
        base_wps_count = len(base_wps)
        
        car_yaw = current_position[3]
        extra_distance = (WHEEL_BASE / 2) + (self.current_velocity.twist.linear.x * LATENCY)
        current_position[0] = current_position[0] + (extra_distance * math.cos(car_yaw))
        current_position[1] = current_position[1] + (extra_distance * math.sin(car_yaw))
        
        while closest_point_not_found:
            base_wp = base_wps[i]

            distance = math.sqrt(((current_position[0] - base_wp[0]) ** 2) +
                                 ((current_position[1] - base_wp[1]) ** 2) +
                                 ((current_position[2] - base_wp[2]) ** 2))
            
            if distance < closest_dist:
                shift_x = base_wp[0] - current_position[0];
                shift_y = base_wp[1] - current_position[1];
                new_base_x = (shift_x * math.cos(0-car_yaw) - shift_y * math.sin(0-car_yaw));
                if (new_base_x > 0):
                    closest_point = [base_wp[0], base_wp[1], base_wp[2]]
                    closest_dist = distance
                    new_closest_wp_idx = i
                                
            if distance > closest_dist:
                closest_point_not_found = False
            
            i = i + 1
            if i >= base_wps_count:
                i = 0
            
            if ((closest_point_not_found) and ((i - prev_closest_wp_idx) > 200)):
                closest_point_not_found = False
                new_closest_wp_idx = i
        
        return new_closest_wp_idx

    def get_forward_waypoints(self, new_closest_wp_idx, current_velocity, base_wps):
        # Finds the defined number of waypoints ahead of the vehicle's current position along with their velocities

        start_wp = new_closest_wp_idx
        start_velocity = current_velocity.twist.linear.x
        base_wps_count = len(base_wps)
        
        end_wp = start_wp + LOOKAHEAD_WPS
        if end_wp > base_wps_count:
            end_wp = end_wp - base_wps_count
        end_velocity = base_wps[end_wp][3]

        if (self.traffic_light_state != -1) and (start_wp <= self.traffic_light_state):
            #rospy.loginfo("red light spotted at start wp, end wp = %s, %s, %s", start_wp, end_wp, self.traffic_light_state)
            if self.traffic_light_state <= end_wp:
                end_wp = self.traffic_light_state
                end_velocity = 0
                #rospy.loginfo("red light spotted at start wp, end wp, light wp = %s, %s", start_wp, end_wp)
        
        num_of_way_points = end_wp - start_wp + 1
        if num_of_way_points < 0:
            num_of_way_points = num_of_way_points + base_wps_count
        
        forward_wps = []
        j = start_wp
        v = current_velocity.twist.linear.x

        for i in range(num_of_way_points):
            wp = Waypoint()
            wp.pose.header.frame_id = '/world'
            wp.pose.header.stamp = rospy.Time.now()
            
            wp.pose.pose.position.x = base_wps[j][0]
            wp.pose.pose.position.y = base_wps[j][1]
            wp.pose.pose.position.z = base_wps[j][2]
            wp.twist.twist.linear.x = base_wps[j][3]
            
            velocity_delta = base_wps[j][3] - v
            if j == base_wps_count - 1:
                dist_to_next_point = base_wps[0][4]
            else:
                dist_to_next_point = base_wps[j+1][4]
            
            if velocity_delta > 0:
                max_accel = math.sqrt(ACCEL_LIMIT * dist_to_next_point)
                #rospy.loginfo("max_accel = %s", max_accel)
                if velocity_delta > max_accel:
                    wp.twist.twist.linear.x = v + max_accel
            if velocity_delta < 0:
                max_decel = math.sqrt(abs(DECEL_LIMIT) * dist_to_next_point) * 0.25
                #rospy.loginfo("max_decel = %s", max_decel)
                if velocity_delta < -max_decel:
                    wp.twist.twist.linear.x = v - max_decel
                    
            
            
            """
            
            velocity_step = 0.5
            if velocity_delta > 0:
                if velocity_delta > velocity_step:
                    wp.twist.twist.linear.x = current_velocity.twist.linear.x + velocity_step
            if velocity_delta < 0:
                if velocity_delta < -velocity_step:
                    wp.twist.twist.linear.x = current_velocity.twist.linear.x - velocity_step
            
            """        

            v = wp.twist.twist.linear.x
            
            forward_wps.append(wp)
            j = j + 1
            if j >= base_wps_count:
                j = 0
        """
                
        velocity_step = 0.5
        if end_velocity == 0:
            #rospy.loginfo("stop light, len of forward_wps = %s", len(forward_wps))
            end_velocity_chunk = 0
            for i in range(len(forward_wps)):
                if forward_wps[len(forward_wps)-1 - i].twist.twist.linear.x > end_velocity_chunk:
                    forward_wps[len(forward_wps)-1 - i].twist.twist.linear.x = end_velocity_chunk
                    end_velocity_chunk = end_velocity_chunk + velocity_step
                #rospy.loginfo("slowdown velocity = %s", forward_wps[len(forward_wps)-1 - i].twist.twist.linear.x)
        """
        
        if end_velocity == 0:
            target_velocity = 0
            for i in range(len(forward_wps)):
                if forward_wps[len(forward_wps)-1 - i].twist.twist.linear.x > target_velocity:
                    forward_wps[len(forward_wps)-1 - i].twist.twist.linear.x = target_velocity
                    dist_from_prev_point = base_wps[end_wp - i][4]
                    target_velocity = target_velocity + (math.sqrt(abs(DECEL_LIMIT) * dist_from_prev_point) * 0.25)
                #rospy.loginfo("slowing down wp, velocity = %s, %s", str(len(forward_wps)-1 - i), forward_wps[len(forward_wps)-1 - i].twist.twist.linear.x)
                
        #rospy.loginfo("no_of_wp, velocity_delta, start_velocity, end_velocity = %s, %s, %s, %s", len(forward_wps), forward_wps[0].twist.twist.linear.x-current_velocity.twist.linear.x, forward_wps[0].twist.twist.linear.x, forward_wps[-1].twist.twist.linear.x)
        #rospy.loginfo("count, start_wp, end_wp = %s, %s, %s", num_of_way_points, start_wp, end_wp)

        return forward_wps

if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.loginfo('Could not start waypoint updater node.')
