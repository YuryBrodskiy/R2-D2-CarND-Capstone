#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
import tf

import math

'''
This node will publish waypoints from the car's current position
to some `x` distance ahead.

As mentioned in the doc, you should ideally
first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node,
you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of
traffic lights and their current status in `/vehicle/traffic_lights`
message. You can use this message to build this node as well as to
verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 200  # Number of waypoints we will publish. You can change this number  # noqa
SEARCH_INTERVAL = 100


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber(
            '/current_pose', PoseStamped, self.pose_cb, queue_size=1)
        rospy.Subscriber(
            '/base_waypoints', Lane, self.waypoints_cb, queue_size=1)

        # TODO: Add a subscriber for
        # /traffic_waypoint and /obstacle_waypoint below
        self.final_waypoints_pub = rospy.Publisher(
            'final_waypoints', Lane, queue_size=1)
        self.pose = None
        self.waypoints = None
        # reduce computation
        self.last_index = None

        # TODO: Add other member variables you need below
        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg.pose
        if self.waypoints is not None:
            next_waypoint_index = self.next_waypoint()
            if self.last_index is None:
                self.last_index = next_waypoint_index
            next_wps = self.waypoints[
                next_waypoint_index: next_waypoint_index + LOOKAHEAD_WPS]
            diff = LOOKAHEAD_WPS - len(next_wps)
            if diff > 0:
                rospy.logwarn("WaypointUpdater: End of waypoints")
                next_wps.extend(self.waypoints[:diff])
            lane = Lane()
            lane.header.frame_id = '/world'
            lane.header.stamp = rospy.Time.now()
            lane.waypoints = next_wps
            self.final_waypoints_pub.publish(lane)

    def waypoints_cb(self, waypoints):
        # TODO: Implement
        if self.waypoints is None:
            self.waypoints = waypoints.waypoints

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        pass

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message.
        # We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt(
            (a.x - b.x)**2 + (a.y - b.y)**2 + (a.z - b.z)**2)
        for i in range(wp1, wp2 + 1):
            dist += dl(waypoints[wp1].pose.pose.position,
                       waypoints[i].pose.pose.position)
            wp1 = i
        return dist

    def next_waypoint(self):
        position = self.pose.position
        # do not always search all 10902 waypoints
        if self.last_index is None:
            dists = [(position.x - wp.pose.pose.position.x)**2 -
                     (position.y - wp.pose.pose.position.y)**2
                     for wp in self.waypoints]
            min_index = min(range(len(dists)), key=dists.__getitem__)
            min_dist = dists[min_index]
            self.last_index = min_index
        else:
            search_waypoints = self.waypoints[
                self.last_index - SEARCH_INTERVAL:
                self.last_index + SEARCH_INTERVAL + 1]
            dists = [math.sqrt(
                (position.x - wp.pose.pose.position.x)**2 +
                (position.y - wp.pose.pose.position.y)**2)
                for wp in search_waypoints]
            min_index = min(range(len(dists)), key=dists.__getitem__)
            min_dist = dists[min_index]
            # shift back
            min_index = min_index + self.last_index - SEARCH_INTERVAL
            # reset
            if min_dist > 8:
                rospy.logwarn("WaypointUpdater: Reset %s" % min_dist)
                self.last_index = None

        # check if point is in front of us
        # adapted from here: https://github.com/jeremy-shannon/CarND-Capstone/blob/master/ros/src/waypoint_updater/waypoint_updater.py  # noqa
        while True:
            if min_index >= len(self.waypoints):
                min_index = 0
            closest_wp_position = self.waypoints[min_index].pose.pose.position
            heading = math.atan2(closest_wp_position.y - position.y,
                                 closest_wp_position.x - position.x)
            quaternion = (self.pose.orientation.x,
                          self.pose.orientation.y,
                          self.pose.orientation.z,
                          self.pose.orientation.w)
            picht, roll, yaw = tf.transformations.euler_from_quaternion(
                quaternion)
            if abs(heading - yaw) > math.pi / 4:
                min_index += 1
            else:
                break

        return min_index


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
