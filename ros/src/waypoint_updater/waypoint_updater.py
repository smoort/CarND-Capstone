#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32
from geometry_msgs.msg import TwistStamped

import math
import tf

'''

This node will publish waypoints from the car's current position to some `x` distance ahead 
along with target velocity for each of the points

'''

LOOKAHEAD_WPS = 40                                  # Number of waypoints we will publish.
WHEEL_BASE = rospy.get_param('~wheel_base', 2.8498) # Wheel base is used to calculate the front end of the car from the current position given
LATENCY = 0.200                                     # This is the latency assumed, time taken for the command to be executed 
DECEL_LIMIT = rospy.get_param('~decel_limit', -5)   # Sets the max deceleration, used to calculate the quantum of velocity reduction between each waypoint
ACCEL_LIMIT = rospy.get_param('~accel_limit', 1.)   # Sets the max acceleration, used to calculate the quantum of velocity increase between each waypoint

class WaypointUpdater(object):

    def __init__(self):
    
        rospy.init_node('waypoint_updater')

        self.current_position = []
        self.current_position_available = False
        self.base_wps = []
        self.traffic_light_state = -1
        self.current_velocity = TwistStamped()
        
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb, queue_size=1)
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb, queue_size=1)
        rospy.Subscriber('/current_velocity', TwistStamped, self.current_velocity_cb, queue_size=1)
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb, queue_size=1)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        prev_closes_wp_idx = 0
        r = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            if self.current_position_available:
                # Find the closest base wp index in the direction of the car using the car's current position and yaw
                new_closest_wp_idx = self.get_closest_waypoint(self.current_position, prev_closes_wp_idx, self.base_wps)
                rospy.loginfo('closest idx = %s', str(new_closest_wp_idx))
                
                # Find the forward waypoints from the calculated nearest waypoint along with the target velocity for each point
                final_wps = self.get_forward_waypoints(new_closest_wp_idx, self.current_velocity, self.base_wps)
                rospy.loginfo('current car position, first waypoint x = %s, %s', str(self.current_position), str(final_wps[0].pose.pose.position.x))
                
                final_lane = Lane()
                final_lane.header.frame_id = '/world'
                final_lane.header.stamp = rospy.Time.now()
                final_lane.waypoints = final_wps
                
                # Publish the forward waypoints
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
        # Get car yaw by converting car orientation from quaternion to euler
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
        next_best_closest_wp_idx = prev_closest_wp_idx
        base_wps_count = len(base_wps)
        
        # Calculate the new position of the car factoring for latency, its current velocity and wheel base
        car_yaw = current_position[3]
        #rospy.loginfo("car position before adjustment, wp, carx, cary, car_yaw = %s, %s, %s, %s", i, current_position[0],current_position[1],car_yaw)
        extra_distance = (WHEEL_BASE / 2) + (self.current_velocity.twist.linear.x * LATENCY)
        current_position[0] = current_position[0] + (extra_distance * math.cos(car_yaw))
        current_position[1] = current_position[1] + (extra_distance * math.sin(car_yaw))
        
        #rospy.loginfo("car position after adjustment, wp, carx, cary, car_yaw = %s, %s, %s, %s", i, current_position[0],current_position[1],car_yaw)
        one_loop_done = False
        while closest_point_not_found:
            base_wp = base_wps[i]

            distance = math.sqrt(((current_position[0] - base_wp[0]) ** 2) +
                                 ((current_position[1] - base_wp[1]) ** 2) +
                                 ((current_position[2] - base_wp[2]) ** 2))
            
            # Check if the car is closer to the current base wp than any of the earlier wps
            if distance < closest_dist:
                next_best_closest_wp_idx = i  # Hold this value as the best possible close point in case we are not able to find any ahead of the car
                
                # Check if the current base wp is ahead of the car by transforming it to car coordinates
                shift_x = base_wp[0] - current_position[0]
                shift_y = base_wp[1] - current_position[1]
                new_base_x = (shift_x * math.cos(0-car_yaw) - shift_y * math.sin(0-car_yaw))
                #rospy.loginfo("inside get_closest_waypoint < closest_dist, wp, car_x, new_base_x = %s, %s, %s", next_best_closest_wp_idx, str(current_position[0]), str(new_base_x))
                if new_base_x > 0:
                    #rospy.loginfo("inside get_closest_waypoint next closest point, wp, distance, closest_dist = %s, %s, %s", i, distance, closest_dist)
                    closest_point = [base_wp[0], base_wp[1], base_wp[2]]
                    closest_dist = distance
                    new_closest_wp_idx = i
            
            # If the distance starts diverging, stop the search
            if distance > closest_dist:
                #rospy.loginfo("inside exceed distance, wp, distance, closest_dist = %s, %s, %s", i, distance, closest_dist)
                closest_point_not_found = False
            
            # If the base wp list has reached its end, start over again from beginning just once
            i = i + 1
            if i >= base_wps_count:
                if one_loop_done:
                    # The base wp list has been traversed on full loop.  Stop the search at this point and go with the best closest wp identified, even if it is behind the car
                    rospy.loginfo("inside second loop, chosing next best point and exiting while loop with wp = %s", next_best_closest_wp_idx)
                    new_closest_wp_idx = next_best_closest_wp_idx
                    closest_point_not_found = False  #  Set the search flag off so that the while loop is broken
                else:
                    # Reset the base wp list search from beginning, should be done only once
                    rospy.loginfo("inside one loop done, i = %s", i)
                    i = 0
                    one_loop_done = True  # Flag to indicate that the search has crossed the end of list and started from beginning
            
        return new_closest_wp_idx

    def get_forward_waypoints(self, new_closest_wp_idx, current_velocity, base_wps):
        # Finds the defined number of waypoints ahead of the vehicle's current position along with their velocities

        start_wp = new_closest_wp_idx
        start_velocity = current_velocity.twist.linear.x
        base_wps_count = len(base_wps)
        
        end_wp = start_wp + LOOKAHEAD_WPS
        # if end waypoint gets past the waypoint list, start over from the beginning
        if end_wp >= base_wps_count:
            end_wp = end_wp - base_wps_count
        end_velocity = base_wps[end_wp][3]

        # If red light detected ahead of the car, set the end wp to the red light stop line waypoint and end velocity to 0
        if (self.traffic_light_state != -1) and (start_wp <= self.traffic_light_state):
            rospy.loginfo("red light spotted at start wp, end wp = %s, %s, %s", start_wp, end_wp, self.traffic_light_state)
            if self.traffic_light_state <= end_wp:
                end_wp = self.traffic_light_state
                end_velocity = 0
                rospy.loginfo("red light spotted at start wp, end wp, light wp = %s, %s", start_wp, end_wp)
        
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
            
            
            #rospy.loginfo("checking velocity at x %s, velocity_delta, base_velocity, v = %s, %s, %s", wp.pose.pose.position.x, velocity_delta, base_wps[j][3], v)
            
            
            # Set the velocity for each waypoint as the desired velocity provided as input
            # In case the target velocity is different from the current velocity, the velocity adjustment is done gradually to avoid jerk
            
            #  Limit the max acceleration / deceleration between two points to the max allowed limit, the distance between the waypoints is used to control this
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
            #  Limit the max acceleration / deceleration between two points to a set value
            velocity_step = 0.5
            if velocity_delta > 0:
                if velocity_delta > velocity_step:
                    wp.twist.twist.linear.x = v + velocity_step
            if velocity_delta < 0:
                if velocity_delta < -velocity_step:
                    wp.twist.twist.linear.x = v - velocity_step
            
            """
            
            #rospy.loginfo("setting velocity at x %s = %s", wp.pose.pose.position.x, wp.twist.twist.linear.x)
            v = wp.twist.twist.linear.x
            
            forward_wps.append(wp)
            j = j + 1
            if j >= base_wps_count:
                j = 0
        

        # If end velocity is 0, braking has to be applied to bring the car to a gradual stop.
        # This is achieved by setting velocity from the end of the list, starting with 0 and increasing it gradually until the velocity equals the target velocity set for the waypoint
        
        """
        # Limit teh deceleration between two points to a set value
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
        
        # Limit teh deceleration between two points to the DECEL_LIMIT provided.
        if end_velocity == 0:
            target_velocity = 0
            for i in range(len(forward_wps)):
                if forward_wps[len(forward_wps)-1 - i].twist.twist.linear.x > target_velocity:
                    forward_wps[len(forward_wps)-1 - i].twist.twist.linear.x = target_velocity
                    dist_from_prev_point = base_wps[end_wp - i][4]
                    target_velocity = target_velocity + (math.sqrt(abs(DECEL_LIMIT) * dist_from_prev_point) * 0.25)
                #rospy.loginfo("slowing down wp, velocity = %s, %s", str(len(forward_wps)-1 - i), forward_wps[len(forward_wps)-1 - i].twist.twist.linear.x)
                
        #rospy.loginfo("no_of_wp, start position, end position = %s, (%s,%s) (%s,%s)", len(forward_wps), forward_wps[0].pose.pose.position.x, forward_wps[0].pose.pose.position.y, forward_wps[-1].pose.pose.position.x, forward_wps[-1].pose.pose.position.y)
        rospy.loginfo("velocity_delta, start_velocity, end_velocity = %s, %s, %s", forward_wps[0].twist.twist.linear.x-current_velocity.twist.linear.x, forward_wps[0].twist.twist.linear.x, forward_wps[-1].twist.twist.linear.x)
        #rospy.loginfo("count, start_wp, end_wp = %s, %s, %s", num_of_way_points, start_wp, end_wp)

        return forward_wps

if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.loginfo('Could not start waypoint updater node.')
