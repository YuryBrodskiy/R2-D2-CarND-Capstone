#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped

from styx_msgs.msg import Lane
from visualization_msgs.msg import Marker, MarkerArray
import tf2_ros
import math

from utils import *
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
        self.base_waypoints = None
        self.current_pose = None
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        self.transform_broadcaster = tf2_ros.TransformBroadcaster()
        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)
        self.marker_publish = rospy.Publisher('visualization_marker_array', MarkerArray, queue_size=1)
        # TODO: Add other member variables you need below

    def spin(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.publish_final_waypoints()
            rate.sleep()
    def pose_cb(self, msg):
        # TODO: Implement
        self.current_pose = msg


    def pub_tf(self, waypoints, color, ns, time=rospy.Time.from_sec(0.0)):
        marker_array = MarkerArray()
        for index, waypoint in enumerate(waypoints):
            marker = Marker()
            marker.header.frame_id = "world"
            marker.header.stamp = time
            marker.ns = ns
            marker.id = index
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose = waypoint.pose.pose

            marker.scale.x = self.get_waypoint_velocity(waypoint)
            marker.scale.y = self.get_waypoint_velocity(waypoint)
            marker.scale.z = self.get_waypoint_velocity(waypoint)
            marker.color.a = 0.5 # Don't' forget to set the alpha!
            marker.color.r = color[0]
            marker.color.g = color[1]
            marker.color.b = color[2]
            marker_array.markers.append(marker)
        self.marker_publish.publish(marker_array)


    def waypoints_cb(self, lane):
        # TODO: Implement
        self.base_waypoints = lane.waypoints
        self.pub_tf(self.base_waypoints, [0.0, 1.0, 0.0], "r2d2_road")



    def publish_final_waypoints(self):
        if self.base_waypoints is not None and self.current_pose is not None:
            final_waypoints = Lane()
            final_waypoints.header = self.current_pose.header
            final_waypoints.header.stamp = rospy.Time.now()
            final_waypoints.waypoints = []

            def dist_current(wp):
                wpp = wp.pose.pose.position
                cpp = self.current_pose.pose.position
                return math.sqrt((cpp.x - wpp.x) ** 2 + (cpp.y - wpp.y) ** 2 + (cpp.z - wpp.z) ** 2)
            dist_min = 30
            index = 0
            for i, waypoint in enumerate(self.base_waypoints):
                dist_c = dist_current(waypoint)
                if dist_c < 30 and isInFront(self.current_pose.pose, waypoint.pose.pose):
                    if dist_c < dist_min:
                        index = i
                        dist_min = dist_c
                       # final_waypoints.waypoints.append((i, waypoint))
                else:
                    pass
            end = min([index+LOOKAHEAD_WPS, len(self.base_waypoints)])
            # final_waypoints.waypoints = sorted(final_waypoints.waypoints, key=dist_current)
            final_waypoints.waypoints = self.base_waypoints[index:end]
            # sort by distnace
            # self.pub_tf(final_waypoints.waypoints[:LOOKAHEAD_WPS])
            #print("Got heres")
            #vim rospy.logwarn("works")
            print("Base ", len(self.base_waypoints))
            print("Final ", len(final_waypoints.waypoints))
            self.final_waypoints_pub.publish(final_waypoints)
            self.pub_tf(final_waypoints.waypoints, [0.0, 0.0, 1.0], "r2d2_final", rospy.Time.now())

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
        wpu = WaypointUpdater()
        wpu.spin()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
