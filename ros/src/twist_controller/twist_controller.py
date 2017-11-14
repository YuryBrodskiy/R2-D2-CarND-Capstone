import rospy
from pid import PID
from lowpass import LowPassFilter
from yaw_controller import YawController
from math import cos, sin
import numpy as np
import tf

GAS_DENSITY = 2.858
ONE_MPH = 0.44704

def localize_coords(pose, waypoints):
    roll, pitch, yaw = tf.transformations.euler_from_quaternion([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
    x_localized = []
    y_localized = []
    localX = pose.position.x
    localY = pose.position.y

    #perform reference shift for number of relevant waypoints
    rel_waypoints = len(waypoints)

    for i in range(rel_waypoints):
	offsetX = waypoints[i].pose.pose.position.x - localX
	offsetY = waypoints[i].pose.pose.position.y - localY
	new_x = offsetX * cos(0 - yaw) - offsetY * sin(0 - yaw)
	new_y = offsetX * sin(0 - yaw) + offsetY * cos(0 - yaw)
	x_localized.append(new_x)
	y_localized.append(new_y)
    return x_localized, y_localized

def get_cte(pose, waypoints):
    x_local, y_local = localize_coords(pose, waypoints)
    coefs = np.polyfit(x_local, y_local, 3)
    error = np.polyval(coefs, 5.0)
    return error

class Controller(object):
    def __init__(self, ps):
        self.yaw_controller = YawController(
            ps.wheel_base, ps.steer_ratio,
            0.1, ps.max_lat_accel, ps.max_steer_angle)
        #Init params, PID, and filters
        self.ps = ps
	self.driverless_mass = self.ps.vehicle_mass + self.ps.fuel_capacity * GAS_DENSITY
	self.pid_steer = PID(.5, .01, .25, -ps.max_steer_angle, ps.max_steer_angle)
        
	#Removed LPF - legacy
	#self.s_lpf = LowPassFilter(3, .5)
        #self.a_lpf = LowPassFilter(3, .5)

    def control(self, twist_cmd, c_v, t_delta, pose, waypoints):
	cte = get_cte(pose, waypoints)

	#Velocity update for accel/steer
        l_v = abs(twist_cmd.twist.linear.x)
        a_v = twist_cmd.twist.angular.z
        velocity_error = l_v - c_v.linear.x
        
        #Steer prediction
        steer = self.yaw_controller.get_steering(l_v, a_v, c_v.linear.x)
        #steer = self.s_lpf.filt(steer)
        
	#Steer correction
	steer_cor = self.pid_steer.step(cte, t_delta)
	#steer = self.s_lpf.filt(steer_cor + steer)
	steer = steer_cor + steer

        #Throttle/brake prediction
	accel = velocity_error / t_delta
  
  	#Update sensitive to direction
  	throttle = 0
  	brake = 0
        
  	if accel > 0:
	    accel = min(accel, self.ps.accel_limit)
  	else:
            accel = max(accel, self.ps.decel_limit)
        torque = self.driverless_mass * accel * self.ps.wheel_radius

	#accel/decel until preference or max torque
	if accel > 0:
	    throttle = min(1, torque/(self.driverless_mass * self.ps.accel_limit * self.ps.wheel_radius))
	else:
	    brake = min(abs(torque), (self.driverless_mass * abs(self.ps.decel_limit) * self.ps.wheel_radius))

	#Zero-out for deadband
	if abs(accel) < self.ps.brake_deadband:
	    throttle, brake = 0, 0
        
	# Return throttle, brake, steer
        return throttle, brake, steer
    
    #Reset for post-manual control/Stop
    def reset(self):
	self.pid_steer.reset()
