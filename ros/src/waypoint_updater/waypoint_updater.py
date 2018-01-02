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

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number
WHEEL_BASE = rospy.get_param('~wheel_base', 2.8498)
LATENCY = 0.500     # 500ms latency assumed

class WaypointUpdater(object):

    def __init__(self):
    
        rospy.init_node('waypoint_updater')

        # TODO: Add other member variables you need below
        self.current_position = []
        self.base_wps = []
        self.base_wps_count = 0
        self.prev_closest_idx = 0
        self.final_lane = Lane()
        self.final_wps = []
        self.traffic_light_state = -1
        self.current_velocity = TwistStamped()
        
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/current_velocity', TwistStamped, self.current_velocity_cb)
        
        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=10)
        
        rospy.spin()

    def pose_cb(self, msg):
        # TODO: Implement
        #rospy.loginfo("inside current cb")
        self.current_position = []
        self.current_position.append(msg.pose.position.x)
        self.current_position.append(msg.pose.position.y)
        self.current_position.append(msg.pose.position.z)
        quat = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
        euler = tf.transformations.euler_from_quaternion(quat)
        self.current_position.append(euler[2])
        #rospy.loginfo("yaw = %s", euler[2])
        self.get_forward_waypoints()
        #rospy.loginfo('current, final positions = %s, %s', str(self.current_position), str(self.final_wps[0].pose.pose.position))
        
        self.final_lane.header.frame_id = '/world'
        self.final_lane.header.stamp = rospy.Time.now()
        self.final_lane.waypoints = self.final_wps
        self.final_waypoints_pub.publish(self.final_lane)
        
        return

    def waypoints_cb(self, waypoints):
        # TODO: Implement
        #rospy.loginfo("total base_waypoints = %s", len(waypoints.waypoints))
        #rospy.loginfo("inside base cb")
        self.base_wps = []
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
                dist_from_prev_point = math.sqrt((base_wp[0] - self.base_wps[i-1][0]) ** 2
                                                +(base_wp[1] - self.base_wps[i-1][1]) ** 2)
                yaw = math.atan2((base_wp[1] - self.base_wps[i-1][1]), (base_wp[0] - self.base_wps[i-1][0]))
            base_wp.append(dist_from_prev_point)
            base_wp.append(yaw)
            self.base_wps.append(base_wp)
        self.base_wps_count = len(self.base_wps)
        return
        
    def current_velocity_cb(self, TwistMsg):
        #rospy.loginfo("inside current_velocity_cb")
        self.current_velocity = TwistMsg

    def get_forward_waypoints(self):
        # Finds the defined number of waypoints ahead of the vehicle's current position
        #rospy.loginfo("inside get forward waypoints")
        #rospy.loginfo("prev_closest_idx = %s", str(self.prev_closest_idx))
        #rospy.loginfo("value of first base_waypoint = %s", str(self.base_wps[self.prev_closest_idx]))
        closest_point = []
        closest_dist = 9999999999
        closest_point_not_found = True
        one_loop_done = False
        i = self.prev_closest_idx
        cp = self.current_position
        
        
        car_yaw = self.current_position[3]
        #cos_yaw = min(1,max(self.current_position[3],-1))
        #sin_yaw = max(-1,min(self.current_position[3],1))
        extra_distance = (WHEEL_BASE / 2) + (self.current_velocity.twist.linear.x * LATENCY)
        cp[0] = self.current_position[0] + (extra_distance * math.cos(car_yaw))
        cp[1] = self.current_position[1] + (extra_distance * math.sin(car_yaw))
        #rospy.loginfo("yaw, before, after = %s, %s, %s, [%s,%s], [%s,%s]", car_yaw, math.cos(car_yaw), math.sin(car_yaw), self.current_position[0],self.current_position[1], cp[0], cp[1])
        
        if i == 0:
            ref_yaw = math.atan2((self.base_wps[i][1] - self.base_wps[len(self.base_wps)-1][1]), (self.base_wps[i][0] - self.base_wps[len(self.base_wps)-1][0]))
        else:
            ref_yaw = math.atan2((self.base_wps[i][1] - self.base_wps[i-1][1]), (self.base_wps[i][0] - self.base_wps[i-1][0]))
        
        while closest_point_not_found:
            base_wp = self.base_wps[i]
            #rospy.loginfo("total base_waypoint = %s", len(base_wp))
            #rospy.loginfo("current_position1 length = %s", len(cp))
            #rospy.loginfo("current position = %s, %s, %s", str(cp[0]), str(cp[1]), str(cp[2]))
            #rospy.loginfo("i = %s", str(i))
            distance = math.sqrt(((cp[0] - base_wp[0]) ** 2) +
                                 ((cp[1] - base_wp[1]) ** 2) +
                                 ((cp[2] - base_wp[2]) ** 2))
            #rospy.loginfo("distance = %s", str(distance))
            if distance < closest_dist:
                #rospy.loginfo("inside < section : idx, distance, closest_dist = %s, %s, %s", str(i), str(distance), str(closest_dist))
                shift_x = base_wp[0] - cp[0];
                shift_y = base_wp[1] - cp[1];
                car_ref_yaw = math.atan2(shift_y, shift_x)

                #new_base_x = (shift_x * math.acos(0-ref_yaw) - shift_y * math.asin(0-ref_yaw));
                #new_pt_position = new_base_x - cp[0]
                #if (new_base_x - cp[0] > 0)
                
                #if((car_ref_yaw < (ref_yaw + math.pi/2)) && (car_ref_yaw > (ref_yaw - math.pi/2))
                if(abs(car_ref_yaw - ref_yaw) < (math.pi/2)):
                    #rospy.loginfo("inside yaw match section : idx, distance, closest_dist = %s, %s, %s", str(i), str(distance), str(closest_dist))
                    closest_point = [base_wp[0], base_wp[1], base_wp[2]]
                    closest_dist = distance
                    self.prev_closest_idx = i
            
            if distance > closest_dist:
                #rospy.loginfo("inside > section closest_dist = %s", str(closest_dist))
                closest_point_not_found = False
            
            i = i + 1
            if i > len(self.base_wps):
                #rospy.loginfo("inside loop overflow, i = %s", str(i))
                if one_loop_done:
                    rospy.loginfo("inside loop overflow, one loop done")
                    closest_point_not_found = False
                else:
                    rospy.loginfo("inside loop overflow, one loop NOT done")
                    one_loop_done = True
                    i = 0
            
            if ((closest_point_not_found) and ((i - self.prev_closest_idx) > 200)):
                closest_point_not_found = False
                self.prev_closest_idx = i
        
        #rospy.loginfo("ref_yaw, car_ref_yaw = %s, %s", str(ref_yaw), str(car_ref_yaw))
        
        """
        
        need_to_stop = False
        if self.traffic_light_state != -1:
            #rospy.loginfo("inside waypoint_updater get_forward_waypoints tl is red state, end_wp, end x  = %s, %s, %s", self.traffic_light_state, end_wp, self.base_wps[end_wp][0])
            if self.traffic_light_state <= end_wp:
                end_wp = self.traffic_light_state
                end_velocity = 0
                need_to_stop = True

        for i in range(num_of_way_points):
            wp = Waypoint()
            wp.pose.header.frame_id = '/world'
            wp.pose.header.stamp = rospy.Time.now()
            wp.pose.pose.position.x = self.base_wps[j][0]
            wp.pose.pose.position.y = self.base_wps[j][1]
            wp.pose.pose.position.z = self.base_wps[j][2]
            #wp.twist.twist.linear.x = 10
            wp.twist.twist.linear.x = v
            forward_wps.append(wp)
            if j > len(self.base_wps):
                j = 0
            v = v + velocity_delta
        
        #if need_to_stop:
            #rospy.loginfo("need_to_stop current velocity, end_velocity = %s, %s", self.final_wps[0].twist.twist.linear.x, self.final_wps[-1].twist.twist.linear.x)
        """
        #rospy.loginfo("start wp = %s", self.prev_closest_idx)
        start_wp = self.prev_closest_idx
        if start_wp > self.base_wps_count:
            start_wp = start_wp - self.base_wps_count
        #start_velocity = self.base_wps[start_wp][3]
        start_velocity = self.current_velocity.twist.linear.x
        
        end_wp = start_wp + LOOKAHEAD_WPS
        if end_wp > self.base_wps_count:
            end_wp = end_wp - self.base_wps_count
        end_velocity = self.base_wps[end_wp][3]

        if self.traffic_light_state != -1:
            #rospy.loginfo("red light spotted at wp = %s", self.traffic_light_state)
            if self.traffic_light_state <= end_wp:
                rospy.loginfo("red light close at wp = %s", self.traffic_light_state)
                end_wp = self.traffic_light_state
                end_velocity = 0
        
        num_of_way_points = end_wp - start_wp
        if num_of_way_points < 0:
            num_of_way_points = num_of_way_points + self.base_wps_count
        
        velocity_delta = (end_velocity - start_velocity) / num_of_way_points
        rospy.loginfo("start_wp, end_wp, start_velocity, end_velocity = %s, %s, %s, %s", start_wp, end_wp, start_velocity, end_velocity)
        if 0 < velocity_delta < 1.0:
            velocity_delta = 1.0
        if -1 < velocity_delta < 0:
            velocity_delta = -1.0
        
        forward_wps = []
        v = start_velocity
        j = start_wp
        
        for i in range(LOOKAHEAD_WPS):
            #final_wps.append(self.base_wps[j])
            wp = Waypoint()
            wp.pose.header.frame_id = '/world'
            wp.pose.header.stamp = rospy.Time.now()
            wp.pose.pose.position.x = self.base_wps[j][0]
            wp.pose.pose.position.y = self.base_wps[j][1]
            wp.pose.pose.position.z = self.base_wps[j][2]
            v = v + velocity_delta
            if velocity_delta > 0:
                if v > end_velocity:
                    v = end_velocity
            elif velocity_delta < 0:
                if v < end_velocity:
                    v = end_velocity
            wp.twist.twist.linear.x = v
            forward_wps.append(wp)
            if j > len(self.base_wps):
                j = 0

        forward_wps[-1].twist.twist.linear.x = end_velocity
        self.final_wps = forward_wps
        #rospy.loginfo("no_of_wp, velocity_delta, start_velocity, end_velocity = %s, %s, %s, %s", len(self.final_wps), velocity_delta, forward_wps[0].twist.twist.linear.x, forward_wps[-1].twist.twist.linear.x)
        return

    def traffic_cb(self, traffic_light_state):
        # TODO: Callback for /traffic_waypoint message. Implement
        self.traffic_light_state = traffic_light_state.data
        #rospy.loginfo("inside waypoint_updater traffic_cb, light state = %s", self.traffic_light_state)
        return

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.loginfo('Could not start waypoint updater node.')
