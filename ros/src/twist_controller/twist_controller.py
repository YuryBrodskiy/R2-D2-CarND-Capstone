from yaw_controller import YawController
from pid import PID

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class TwistController(object):
    def __init__(self, wheel_base, steer_ratio,
                 min_speed, max_lat_accel, max_steer_angle,
                 decel_limit, accel_limit,
                 brake_deadband):
        self.brake_deadband = brake_deadband
        self.yaw_controller = YawController(
            wheel_base, steer_ratio,
            min_speed, max_lat_accel, max_steer_angle)
        self.velocity_controller = PID(
            kp=1.0, ki=0.0, kd=0.0, mn=decel_limit, mx=accel_limit)
        pass

    def control(self, set_vel_linear, set_vel_angular,
                curr_vel_linear, dbw_enabled, delta_t):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        req_steer = self.yaw_controller.get_steering(
            set_vel_linear, set_vel_angular, curr_vel_linear)
        req_throttle = self.velocity_controller.step(
            set_vel_linear - curr_vel_linear, delta_t)
        if req_throttle > 0.0:
            req_brake = 0.0
        else:
            req_brake = -req_throttle
            req_throttle = 0.0
            if req_brake < self.brake_deadband:
                req_brake = 0.0

        return req_throttle, req_brake, req_steer
