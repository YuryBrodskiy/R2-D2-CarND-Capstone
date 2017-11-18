#!/usr/bin/env python
import rospy
from std_msgs.msg import Bool
from dbw_mkz_msgs.msg import ThrottleCmd, SteeringCmd, BrakeCmd, SteeringReport
from geometry_msgs.msg import TwistStamped, PoseStamped
from styx_msgs.msg import Lane

from twist_controller import Controller

'''
You can build this node only after you have built (or partially built) the `waypoint_updater` node.

You will subscribe to `/twist_cmd` message which provides the proposed linear and angular velocities.
You can subscribe to any other message that you find important or refer to the document for list
of messages subscribed to by the reference implementation of this node.

One thing to keep in mind while building this node and the `twist_controller` class is the status
of `dbw_enabled`. While in the simulator, its enabled all the time, in the real car, that will
not be the case. This may cause your PID controller to accumulate error because the car could
temporarily be driven by a human instead of your controller.

We have provided two launch files with this node. Vehicle specific values (like vehicle_mass,
wheel_base) etc should not be altered in these files.

We have also provided some reference implementations for PID controller and other utility classes.
You are free to use them or build your own.

Once you have the proposed throttle, brake, and steer values, publish it on the various publishers
that we have created in the `__init__` function.

'''
class Param_State(object):
    def __init__(self):
        self.vehicle_mass = None
        self.fuel_capacity = None
        self.brake_deadband = None
        self.decel_limit = None
        self.accel_limit = None
        self.wheel_radius = None
        self.wheel_base = None
        self.steer_ratio = None
        self.max_lat_accel = None
        self.max_steer_angle = None

class DBWNode(object):
    def __init__(self):
        rospy.init_node('dbw_node')

        ps = Param_State()

        ps.vehicle_mass = rospy.get_param('~vehicle_mass', 1736.35)
        ps.fuel_capacity = rospy.get_param('~fuel_capacity', 13.5)
        ps.brake_deadband = rospy.get_param('~brake_deadband', .1)
        ps.decel_limit = rospy.get_param('~decel_limit', -5)
        ps.accel_limit = rospy.get_param('~accel_limit', 1.)
        ps.wheel_radius = rospy.get_param('~wheel_radius', 0.2413)
        ps.wheel_base = rospy.get_param('~wheel_base', 2.8498)
        ps.steer_ratio = rospy.get_param('~steer_ratio', 14.8)
        ps.max_lat_accel = rospy.get_param('~max_lat_accel', 3.)
        ps.max_steer_angle = rospy.get_param('~max_steer_angle', 8.)

        self.steer_pub = rospy.Publisher('/vehicle/steering_cmd',
                                         SteeringCmd, queue_size=1)
        self.throttle_pub = rospy.Publisher('/vehicle/throttle_cmd',
                                            ThrottleCmd, queue_size=1)
        self.brake_pub = rospy.Publisher('/vehicle/brake_cmd',
                                         BrakeCmd, queue_size=1)

        # Create `TwistController` object
        self.dbw_enabled = True
        self.t_0 = rospy.get_time()
        self.reset = True
        self.twist_cmd = None
        self.current_velocity = None
	self.pose = None
	self.waypoints = None
        self.controller = Controller(ps)


        #Subscribe to needed topics
        rospy.Subscriber('/twist_cmd', TwistStamped, self.twist_cmd_cb, queue_size=1)
        rospy.Subscriber('/current_velocity', TwistStamped, self.current_velocity_cb, queue_size=1)
        rospy.Subscriber('/vehicle/dbw_enabled', Bool, self.dbw_enabled_cb, queue_size=1)
	rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb, queue_size=1)
	rospy.Subscriber('/final_waypoints', Lane, self.waypoints_cb, queue_size=1)
        self.loop()

    def loop(self):
        rate = rospy.Rate(10) # 50Hz
        while not rospy.is_shutdown():
            # Get predicted throttle, brake, and steering using `twist_controller`
            # You should only publish the control commands if dbw is enabled
            t_1 = rospy.get_time()
            t_delta = t_1 - self.t_0
            self.t_0 = t_1

	    #No zero division!
	    t_delta = t_delta + 1e-6
	    
	    #Must have enough waypoints!
	    waypoints_recieved = self.waypoints is not None
	    if not waypoints_recieved:
		continue

	    if (len(self.waypoints) < 10):
		continue

            if self.dbw_enabled and self.twist_cmd is not None and self.current_velocity is not None:
                #Reset for manual/stop
                if self.reset:
                    self.controller.reset()
                    self.reset = False
                
                #Get commands from twist controller
                throttle, brake, steering = self.controller.control(self.twist_cmd, self.current_velocity, t_delta, self.pose, self.waypoints)
                self.publish(throttle, brake, steering)
            else:
                self.reset = True
            rate.sleep()

    def publish(self, throttle, brake, steer):
        tcmd = ThrottleCmd()
        tcmd.enable = True
        tcmd.pedal_cmd_type = ThrottleCmd.CMD_PERCENT
        tcmd.pedal_cmd = throttle
        self.throttle_pub.publish(tcmd)

        scmd = SteeringCmd()
        scmd.enable = True
        scmd.steering_wheel_angle_cmd = steer
        self.steer_pub.publish(scmd)

        bcmd = BrakeCmd()
        bcmd.enable = True
        bcmd.pedal_cmd_type = BrakeCmd.CMD_TORQUE
        bcmd.pedal_cmd = brake
        self.brake_pub.publish(bcmd)

    def current_velocity_cb(self, msg):
        self.current_velocity = msg.twist

    def twist_cmd_cb(self, msg):
        self.twist_cmd = msg

    def dbw_enabled_cb(self, msg):
        self.dbw_enabled = msg.data

    def pose_cb(self, msg):
	self.pose = msg.pose

    def waypoints_cb(self, msg):
	self.waypoints = msg.waypoints


if __name__ == '__main__':
    DBWNode()
