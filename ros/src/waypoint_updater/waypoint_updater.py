#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint

import math

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


class WaypointUpdater(object):

    def __init__(self):
    
        rospy.init_node('waypoint_updater')

        # TODO: Add other member variables you need below
        self.current_position = []
        self.base_wps = []
        self.prev_closest_idx = 0
        self.final_lane = Lane()
        self.final_wps = []
        
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        
        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=10)
        
        rospy.spin()

    def pose_cb(self, msg):
        # TODO: Implement
        #rospy.loginfo("inside current cb")
        self.current_position = []
        self.current_position.append(msg.pose.position.x)
        self.current_position.append(msg.pose.position.y)
        self.current_position.append(msg.pose.position.z)

        self.final_wps = self.get_forward_waypoints()
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
            self.base_wps.append(base_wp)
        return

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
        final_wps = []
        j = self.prev_closest_idx + 20
        
        for i in range(LOOKAHEAD_WPS):
            #final_wps.append(self.base_wps[j])
            wp = Waypoint()
            wp.pose.header.frame_id = '/world'
            wp.pose.header.stamp = rospy.Time.now()
            wp.pose.pose.position.x = self.base_wps[j][0]
            wp.pose.pose.position.y = self.base_wps[j][1]
            wp.pose.pose.position.z = self.base_wps[j][2]
            wp.twist.twist.linear.x = 10
            final_wps.append(wp)
            if j > len(self.base_wps):
                j = 0
        
        return final_wps
    
    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        pass

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
