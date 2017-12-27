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

    # TODO: Add other member variables you need below
    current_position = [0,0,0]
    base_wps = []
    final_wps = []
    prev_closest_idx = 0

    def __init__(self):
        rospy.init_node('waypoint_updater')
        
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        
        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=10)
        
        rospy.spin()

    def pose_cb(self, msg):
        # TODO: Implement
        #rospy.logerr("inside current cb")
        WaypointUpdater.current_position = []
        WaypointUpdater.current_position.append(msg.pose.position.x)
        WaypointUpdater.current_position.append(msg.pose.position.y)
        WaypointUpdater.current_position.append(msg.pose.position.z)

        WaypointUpdater.final_wps = self.get_forward_waypoints()
        rospy.logerr('current, final positions = %s, %s', str(WaypointUpdater.final_wps[0]), str(WaypointUpdater.current_position))
        return

    def waypoints_cb(self, waypoints):
        # TODO: Implement
        #rospy.loginfo("total base_waypoints = %s", len(waypoints.waypoints))
        rospy.logerr("inside base cb")
        WaypointUpdater.base_wps = []
        for i in range(len(waypoints.waypoints)):
            base_wp  = []
            base_wp.append(waypoints.waypoints[i].pose.pose.position.x)
            base_wp.append(waypoints.waypoints[i].pose.pose.position.y)
            base_wp.append(waypoints.waypoints[i].pose.pose.position.z)
            WaypointUpdater.base_wps.append(base_wp)
        return

    def get_forward_waypoints(self):
        # Finds the defined number of waypoints ahead of the vehicle's current position
        #rospy.logerr("inside get forward waypoints")
        #rospy.logerr("prev_closest_idx = %s", str(WaypointUpdater.prev_closest_idx))
        #rospy.logerr("value of first base_waypoint = %s", str(WaypointUpdater.base_wps[WaypointUpdater.prev_closest_idx]))
        closest_point = []
        closest_dist = 9999999999
        closest_point_not_found = True
        one_loop_done = False
        i = WaypointUpdater.prev_closest_idx
        cp = WaypointUpdater.current_position
        
        while closest_point_not_found:
            base_wp = WaypointUpdater.base_wps[i]
            #rospy.logerr("total base_waypoint = %s", len(base_wp))
            #rospy.logerr("current_position1 length = %s", len(cp))
            #rospy.logerr("current position = %s, %s, %s", str(cp[0]), str(cp[1]), str(cp[2]))
            #rospy.logerr("i = %s", str(i))
            distance = math.sqrt(((cp[0] - base_wp[0]) ** 2) +
                                 ((cp[1] - base_wp[1]) ** 2) +
                                 ((cp[2] - base_wp[2]) ** 2))
            #rospy.logerr("distance = %s", str(distance))
            if distance < closest_dist:
                #rospy.logerr("inside < section : idx, distance, closest_dist = %s, %s, %s", str(i), str(distance), str(closest_dist))
                closest_point = [base_wp[0], base_wp[1], base_wp[2]]
                closest_dist = distance
                WaypointUpdater.prev_closest_idx = i
            
            if distance > closest_dist:
                #rospy.logerr("inside > section closest_dist = %s", str(closest_dist))
                closest_point_not_found = False
            
            i = i + 1
            if i > len(WaypointUpdater.base_wps):
                rospy.logerr("inside loop overflow, i = %s", str(i))
                if one_loop_done:
                    rospy.logerr("inside loop overflow, one loop done")
                    closest_point_not_found = False
                else:
                    rospy.logerr("inside loop overflow, one loop NOT done")
                    one_loop_done = True
                    i = 0
                
        final_wps = []
        j = WaypointUpdater.prev_closest_idx
        for i in range(LOOKAHEAD_WPS):
            final_wps.append(WaypointUpdater.base_wps[j])
            if j > len(WaypointUpdater.base_wps):
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
        rospy.logerr('Could not start waypoint updater node.')
