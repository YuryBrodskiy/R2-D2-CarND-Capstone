#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped

from styx_msgs.msg import TrafficLightArray, Lane
from visualization_msgs.msg import Marker, MarkerArray

import std_msgs.msg

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

class TrafficLightConverter(object):

    def __init__(self):
        rospy.init_node('traffic_light_converter')
        self.traffic_lights = None
        self.current_pose = None
        self.base_waypoints = None
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_lights_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        self.marker_publish = rospy.Publisher('visualization_marker_array', MarkerArray, queue_size=1)
        self.red_light_publish = rospy.Publisher('traffic_waypoint',  std_msgs.msg.Int32, queue_size=1)
    def spin(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.publish_traffic_lights()
            rate.sleep()

    def waypoints_cb(self, lane):
        # TODO: Implement
        self.base_waypoints = lane.waypoints

    def pose_cb(self, msg):

        self.current_pose = msg

    def traffic_lights_cb(self, traffic_lights):
        # TODO: Implement
        self.traffic_lights = traffic_lights.lights

    def pub_tf(self, traffic_lights, ns):
        marker_array = MarkerArray()

        y = std_msgs.msg.ColorRGBA()
        y.r, y.g, y.b, y.a = 1.0, 1.0, 0.0, 0.5

        g = std_msgs.msg.ColorRGBA()
        g.r, g.g, g.b, g.a = 0.0, 1.0, 0.0, 0.5

        r = std_msgs.msg.ColorRGBA()
        r.r, r.g, r.b, r.a = 1.0, 0.0, 0.0, 0.5

        u = std_msgs.msg.ColorRGBA()
        u.r, u.g, u.b, u.a = 1.0, 1.0, 1.0, 0.5

        colors = [r, y, g, None, u]
        for index, tl in enumerate(traffic_lights):
            marker = Marker()
            marker.header.frame_id = "world"
            marker.header.stamp = rospy.Time.now()
            marker.ns = ns
            marker.id = index
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose = tl.pose.pose

            marker.scale.x = 20
            marker.scale.y = 20
            marker.scale.z = 20
            marker.color = colors[tl.state]
            marker_array.markers.append(marker)
        self.marker_publish.publish(marker_array)

    def publish_traffic_lights(self):
        waypoint_i = -1
        if self.traffic_lights is not None and self.current_pose is not None and self.base_waypoints is not None:
            self.pub_tf(self.traffic_lights, "r2d2_traffic_lights_sim")

            def dist_current(wp):
                wpp = wp.pose.pose.position
                cpp = self.current_pose.pose.position
                return math.sqrt((cpp.x - wpp.x) ** 2 + (cpp.y - wpp.y) ** 2 + (cpp.z - wpp.z) ** 2)
            dist_min = 300
            index = None
            for i, tl in enumerate(self.traffic_lights):
                dist_c = dist_current(tl)
                if dist_c < 300 and isInFront(self.current_pose.pose, tl.pose.pose):
                    if dist_c < dist_min:
                        index = i
                        dist_min = dist_c
                else:
                    pass
            if index is not None:
                upcomming_tl = self.traffic_lights[index]
                self.pub_tf([upcomming_tl], "r2d2_traffic_lights_up")

                def dist_current_tl(wp):
                    wpp = wp.pose.pose.position
                    cpp = upcomming_tl.pose.pose.position
                    return math.sqrt((cpp.x - wpp.x) ** 2 + (cpp.y - wpp.y) ** 2 + (cpp.z - wpp.z) ** 2)

                if upcomming_tl.state == 0:
                    waypoint = min(self.base_waypoints, key=dist_current_tl)
                    waypoint_i = self.base_waypoints.index(waypoint)

        self.red_light_publish.publish(waypoint_i)

    #
    #
    # def get_waypoint_velocity(self, waypoint):
    #     return waypoint.twist.twist.linear.x
    #
    # def set_waypoint_velocity(self, waypoints, waypoint, velocity):
    #     waypoints[waypoint].twist.twist.linear.x = velocity
    #
    # def distance(self, waypoints, wp1, wp2):
    #     dist = 0
    #     dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
    #     for i in range(wp1, wp2+1):
    #         dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
    #         wp1 = i
    #     return dist


if __name__ == '__main__':
    try:
        wpu = TrafficLightConverter()
        wpu.spin()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
