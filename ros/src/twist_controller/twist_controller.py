import rospy
from pid import PID
from lowpass import LowPassFilter
from yaw_controller import YawController

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, ps):
        self.yaw_controller = YawController(
            ps.wheel_base, ps.steer_ratio,
            0.1, ps.max_lat_accel, ps.max_steer_angle)
        #Init params, PID, and filters
        self.ps = ps
        self.pid = PID(5, .5, .5, ps.decel_limit, ps.accel_limit)
        self.s_lpf = LowPassFilter(3, .5)
        self.a_lpf = LowPassFilter(3, .5)

    def control(self, twist_cmd, c_v, t_delta):
        #Velocity update for accel/steer
        l_v = abs(twist_cmd.twist.linear.x)
        a_v = twist_cmd.twist.angular.z
        velocity_error = l_v - c_v.twist.linear.x
        
        #Steer prediction
        steer = self.yaw_controller.get_steering(l_v, a_v, c_v.twist.linear.x)
        steer = self.s_lpf.filt(steer)
        
        #Throttle/brake prediction
        accel = self.pid.step(velocity_error, t_delta)
        accel = self.a_lpf.filt(accel)

        #Update sensitive to direction
        throttle = 0
        brake = 0
        
        if accel > 0:
            throttle = accel
        else:
            if -accel < self.ps.brake_deadband:
                accel = 0
            brake = -accel * (self.ps.vehicle_mass + self.ps.fuel_capacity * GAS_DENSITY) * self.ps.wheel_radius

        # Return throttle, brake, steer
        return throttle, brake, steer
    
    #Reset for post-manual control/Stop
    def reset(self):
        self.pid.reset()
