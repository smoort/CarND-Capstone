#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import tf
import cv2
import yaml
import math

STATE_COUNT_THRESHOLD = 3
TL_LOOKAHEAD_WPS = 120 # Number of waypoints to look ahead for traffic light.

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.base_wps = []
        self.current_position = []
        self.stop_line_positions = []
        self.traffic_light_status = []
        self.prev_closest_idx = 0
        
        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.lights = []
        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)
        self.simulation = True
        
        self.bridge = CvBridge()
        self.light_classifier = TLClassifier(self.simulation)
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0

        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb)


        
        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

                                
                                              
                                              

                                         
                                              
                         
                            

        rospy.spin()

    def pose_cb(self, msg):
        #rospy.loginfo("inside current cb")
        self.current_position = []
        self.current_position.append(msg.pose.position.x)
        self.current_position.append(msg.pose.position.y)
        self.current_position.append(msg.pose.position.z)
        return

    def waypoints_cb(self, waypoints):
        #rospy.loginfo("inside tl_detector waypoints_cb")
        self.base_wps = []
        for i in range(len(waypoints.waypoints)):
            base_wp  = []
            base_wp.append(waypoints.waypoints[i].pose.pose.position.x)
            base_wp.append(waypoints.waypoints[i].pose.pose.position.y)
            base_wp.append(waypoints.waypoints[i].pose.pose.position.z)
            base_wp.append(i)
            if i == 0:
                dist_from_prev_point = math.sqrt((base_wp[0] - waypoints.waypoints[len(waypoints.waypoints)-1].pose.pose.position.x) ** 2
                                                +(base_wp[1] - waypoints.waypoints[len(waypoints.waypoints)-1].pose.pose.position.y) ** 2)
                yaw = math.atan2((base_wp[1] - waypoints.waypoints[len(waypoints.waypoints)-1].pose.pose.position.y),
                                 (base_wp[0] - waypoints.waypoints[len(waypoints.waypoints)-1].pose.pose.position.x))
            else:
                dist_from_prev_point = math.sqrt((base_wp[0] - self.base_wps[i-1][0]) ** 2
                                                +(base_wp[1] - self.base_wps[i-1][1]) ** 2)
                yaw = math.atan2((base_wp[1] - self.base_wps[i-1][1]), (base_wp[0] - self.base_wps[i-1][0]))
                
            base_wp.append(yaw)
            base_wp.append(dist_from_prev_point)
            
            self.base_wps.append(base_wp)
            
        self.get_closest_waypoint_to_stopline()
            
        return
        
    def get_closest_waypoint_to_stopline(self):
        #rospy.loginfo("inside tl_detector get_closest_waypoint_to_stopline")
        stop_line_positions = self.config['stop_line_positions']
        base_wp_idx = 0
        enhanced_stop_line_positions = []
        for i in range(len(stop_line_positions)):
            stop_line_point = stop_line_positions[i]
            stop_line_point_x = stop_line_point[0]
            stop_line_point_y = stop_line_point[1]
            closest_point_not_found = True
            closest_dist = 9999999999
            closest_base_wp_idx = 0
            
            while closest_point_not_found:
                
                base_wp = self.base_wps[base_wp_idx]
                base_wp_x = base_wp[0]
                base_wp_y = base_wp[1]
                base_wp_yaw = base_wp[4]
                
                distance = math.sqrt(((base_wp_x - stop_line_point_x) ** 2) + ((base_wp_y - stop_line_point_y) ** 2))
                #rospy.loginfo("inside tl_detector get_closest_waypoint_to_stopline, stop line base_wp_idx, distance, closest_dist = %s, %s, %s", base_wp_idx, str(distance), str(closest_dist))
                if distance < closest_dist:
                    #wp_to_stop_line_yaw = math.atan2((base_wp_y - stop_line_point_y), (base_wp_x - stop_line_point_x))
                    wp_to_stop_line_yaw = math.atan2((stop_line_point_y - base_wp_y), (stop_line_point_x - base_wp_x))
                    #rospy.loginfo("base_wp_idx, distance, closest_dist, bp_yaw, spl_yaw = %s, %s, %s, %s, %s", base_wp_idx, str(distance), str(closest_dist), str(base_wp_yaw), str(wp_to_stop_line_yaw))
                    #if(abs(wp_to_stop_line_yaw - base_wp_yaw) < (math.pi/2)):
                    closest_dist = distance
                    closest_base_wp_idx = base_wp_idx
                        
                if distance > closest_dist:
                #rospy.loginfo("inside > section closest_dist = %s", str(closest_dist))
                    closest_point_not_found = False
                
                base_wp_idx = base_wp_idx + 1
                if base_wp_idx >= len(self.base_wps):
                    base_wp_idx = 0
                    
            stop_line_point.append(closest_base_wp_idx)
            enhanced_stop_line_positions.append(stop_line_point)
            #rospy.loginfo("inside tl_detector get_closest_waypoint_to_stopline, stop line idx, sp = %s, %s", i, str(stop_line_point))
        self.stop_line_positions = enhanced_stop_line_positions
        

    def traffic_cb(self, msg):
        #rospy.loginfo("inside tl_detector traffic_cb")
        self.traffic_light_status = []
        traffic_lights = msg.lights
        for i in range(len(traffic_lights)):
            self.traffic_light_status.append(traffic_lights[i].state)
        #rospy.loginfo("inside tl_detector traffic_cb, light status = %s", str(self.traffic_light_status))

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        #rospy.loginfo("inside image_cb")
        self.has_image = True
        self.camera_image = msg
        light_wp, state = self.process_traffic_lights()

        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
        #rospy.loginfo("inside tl_detector image_cb, light_wp state = %s, %s", light_wp, state)
        if self.state != state:
            self.state_count = 0
            self.state = state
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            self.last_state = self.state
            light_wp = light_wp if state == TrafficLight.RED else -1
            rospy.loginfo("inside tl_detector image_cb, light_wp state = %s, %s", light_wp, state)
            self.last_wp = light_wp
            self.upcoming_red_light_pub.publish(Int32(light_wp))
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
        self.state_count += 1

    def get_closest_waypoint(self):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        #TODO implement
        
        closest_point = []
        closest_dist = 9999999999
        closest_point_not_found = True
        one_loop_done = False
        car_closest_base_wp_idx = self.prev_closest_idx
        
        while closest_point_not_found:
            base_wp = self.base_wps[car_closest_base_wp_idx]
            #rospy.loginfo("current position = %s, %s, %s", str(self.current_position[0]), str(self.current_position[1]), str(self.current_position[2]))
            distance = math.sqrt(((self.current_position[0] - base_wp[0]) ** 2) +
                                 ((self.current_position[1] - base_wp[1]) ** 2) +
                                 ((self.current_position[2] - base_wp[2]) ** 2))
            
            if distance < closest_dist:
                #rospy.loginfo("inside yaw match section : idx, distance, closest_dist = %s, %s, %s", str(i), str(distance), str(closest_dist))
                closest_point = [base_wp[0], base_wp[1], base_wp[2]]
                closest_dist = distance
                self.prev_closest_idx = car_closest_base_wp_idx
            
            if distance > closest_dist:
                #rospy.loginfo("inside > section closest_dist = %s", str(closest_dist))
                closest_point_not_found = False
            
            car_closest_base_wp_idx = car_closest_base_wp_idx + 1
            if car_closest_base_wp_idx > len(self.base_wps):
                #rospy.loginfo("inside loop overflow, car_closest_base_wp_idx = %s", str(car_closest_base_wp_idx))
                if one_loop_done:
                    rospy.loginfo("inside loop overflow, one loop done")
                    closest_point_not_found = False
                else:
                    rospy.loginfo("inside loop overflow, one loop NOT done")
                    one_loop_done = True
                    car_closest_base_wp_idx = 0
            
        #rospy.loginfo("inside tl_detector get_closest_waypoint, car_closest_base_wp_idx, point = %s, %s", str(car_closest_base_wp_idx), str(closest_point))

        return car_closest_base_wp_idx

    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        if(not self.has_image):
            self.prev_light_loc = None
            return False


        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")
        
                                               

        #Get classification
        
        """
        rospy.loginfo("requested classification at %s", rospy.get_rostime())
        tl_status = self.light_classifier.get_classification(cv_image)        
        state = 4
        if tl_status == 2:
            state = 0
        if tl_status == 3:
            state = 1
        if tl_status == 1:
            state = 2
        rospy.loginfo("requested classification at %s", rospy.get_rostime())
        rospy.loginfo("got classification at wp, classification = %s, %s", self.current_position[0], state)            
        rospy.loginfo("stub, real = %s, %s", self.traffic_light_status[light], state)
        """
        
        #return state
        return self.traffic_light_status[light]

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
       
        car_position = self.get_closest_waypoint()

        #TODO find the closest visible traffic light (if one exists)
        
        closest_dist = 9999999
        closest_stop_line_wp = 0
        closest_light_id = 0
        
        for i in range(len(self.stop_line_positions)):
            stop_line_point = self.stop_line_positions[i]
            stop_line_wp = stop_line_point[2]
            distance_to_light = stop_line_wp - car_position
            if distance_to_light < 0:
                distance_to_light = len(self.base_wps) - car_position + stop_line_wp
            
            #rospy.loginfo("distance_to_light, closest_dist = %s, %s", distance_to_light, closest_dist)
            if (distance_to_light < closest_dist) and (stop_line_wp > car_position):
                closest_dist = distance_to_light
                closest_stop_line_wp = stop_line_wp
                closest_light_id = i
        
        #rospy.loginfo("car wp, closest_stop_line_wp = %s, %s", car_position, closest_stop_line_wp)
        
        light = False
        if closest_dist < TL_LOOKAHEAD_WPS:
            light = True

        if light:
            state = self.get_light_state(closest_light_id)
            rospy.loginfo("nearing light at car_wp, stop_line_wp, state = %s, %s, %s", car_position, closest_stop_line_wp, state)
            return closest_stop_line_wp, state
        self.waypoints = None
        return -1, TrafficLight.UNKNOWN

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
