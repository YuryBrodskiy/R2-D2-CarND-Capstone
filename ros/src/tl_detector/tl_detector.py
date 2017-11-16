#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
# import tf
import cv2
import math
import yaml

STATE_COUNT_THRESHOLD = 3
TL_MAX_DIST = 50
CAR_WP_LIGHT_WP_OFFSET = 20
INF = 100000
TIME_EXECUTION = False


def dist_fn(x1, x2, y1, y2):
    return math.sqrt((x1 - x2)**2 + (y1 - y2)**2)


class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.lights = []

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0
        self.tl_wp_mapping = {}
        self.tl_initialized = False
        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        sub1 = rospy.Subscriber(
            '/current_pose', PoseStamped, self.pose_cb)
        sub2 = rospy.Subscriber(
            '/base_waypoints', Lane, self.waypoints_cb)
        '''
        /vehicle/traffic_lights provides you with the location of the traffic
        light in 3D map space and helps you acquire an accurate ground truth
        data source for the traffic light classifier by sending the current
        color state of all traffic lights in the simulator. When testing on
        the vehicle, the color state will not be available. You'll need to rely
        on the position of the light and the camera image to predict it.
        '''
        sub3 = rospy.Subscriber(
            '/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb,
            queue_size=1)
        sub6 = rospy.Subscriber(
            '/image_color', Image, self.image_cb, queue_size=1,
            buff_size=2**24)

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()
        self.tl_initialized = True
        rospy.logwarn("TLClassifier initialized")
        # self.listener = tf.TransformListener()


        self.upcoming_red_light_pub = rospy.Publisher(
            '/traffic_waypoint', Int32, queue_size=1, latch=True)
        self.debug_image_pub = rospy.Publisher(
            '/debug_image', Image, queue_size=1, latch=True)
        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        if self.waypoints is None:
            self.waypoints = waypoints
        if len(self.waypoints.waypoints) != len(waypoints.waypoints):
            rospy.logwarn("Map changed!")
            self.tl_wp_mapping = {}
            self.waypoints = waypoints

    def traffic_cb(self, msg):
        self.lights = msg.lights

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes
           the index of the waypoint closest to the red light's stop line
           to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        self.has_image = True
        self.camera_image = msg
        if not self.tl_initialized:
            return
        light_wp, state = self.process_traffic_lights()

        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
        if self.state != state:
            self.state_count = 0
            self.state = state
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            self.last_state = self.state
            light_wp = light_wp if state == TrafficLight.RED else -1
            self.last_wp = light_wp
            self.upcoming_red_light_pub.publish(Int32(light_wp))
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
        self.state_count += 1

    def get_closest_waypoint(self, x, y):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        # DONE(see--): implement
        dists = [
            dist_fn(x, wp.pose.pose.position.x,
                    y, wp.pose.pose.position.y)
            for wp in self.waypoints.waypoints]
        min_index = min(range(len(dists)), key=dists.__getitem__)
        return min_index

    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in
                 styx_msgs/TrafficLight)

        """
        if not self.has_image:
            self.prev_light_loc = None
            return False

        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")

        # Get classification
        tic = rospy.get_time()
        tl_state, debug_image = self.light_classifier.get_classification(
            cv_image)
        toc = rospy.get_time()
        if TIME_EXECUTION:
            rospy.logwarn("Detect / classify took %f secs" % (toc - tic))
        tic = rospy.get_time()
        self.debug_image_pub.publish(
            self.bridge.cv2_to_imgmsg(debug_image, encoding="rgb8"))
        toc = rospy.get_time()
        if TIME_EXECUTION:
            rospy.logwarn("Publishing took %f secs" % (toc - tic))

        tl_state = TrafficLight.GREEN
        return tl_state

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a
                 traffic light (-1 if none exists)
            int: ID of traffic light color (specified
                 in styx_msgs/TrafficLight)

        """
        light = None
        tic = rospy.get_time()
        # List of positions that correspond to the line to stop in front
        # of for a given intersection
        stop_line_positions = self.config['stop_line_positions']
        if self.pose and self.waypoints:
            min_index_diff = INF
            car_wp = self.get_closest_waypoint(
                self.pose.pose.position.x, self.pose.pose.position.y)
            for tl_pos in stop_line_positions:
                tl_key = str(tl_pos[0]) + str(tl_pos[1])
                if tl_key not in self.tl_wp_mapping:
                    light_wp = self.get_closest_waypoint(tl_pos[0], tl_pos[1])
                    self.tl_wp_mapping[tl_key] = light_wp
                else:
                    light_wp = self.tl_wp_mapping[tl_key]
                index_diff = light_wp - car_wp
                # check that traffic light is in front
                if index_diff >= -CAR_WP_LIGHT_WP_OFFSET \
                   and index_diff < min_index_diff \
                   and index_diff < TL_MAX_DIST:
                    light = light_wp
                    min_index_diff = index_diff
                    # rospy.logwarn(
                    #     "Car wp: " + str(car_wp) + " light: " + str(light_wp))
        toc = rospy.get_time()
        if TIME_EXECUTION:
            rospy.logwarn("WP stuff took  %f secs" % (toc - tic))
        # TODO find the closest visible traffic light (if one exists)
        if light is not None:
            state = self.get_light_state(light)
            rospy.logwarn("Light: %s, State: %s" % (light, state))

            return light, state

        self.waypoints = None
        return -1, TrafficLight.UNKNOWN


if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
