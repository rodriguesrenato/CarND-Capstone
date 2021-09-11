from pid import PID
from lowpass import LowPassFilter
from yaw_controller import YawController
import rospy
GAS_DENSITY = 2.858
ONE_MPH = 0.44704

# TODO: to PARAMS
MAX_DECEL = 10.0
TARGET_DECEL_LIMIT = 1.0

class Controller(object):
    def __init__(self, vehicle_mass, fuel_capacity, brake_deadband, decel_limit,
                 accel_limit, wheel_radius, wheel_base, steer_ratio, max_lat_accel, max_steer_angle):

        self.yaw_controller = YawController(
            wheel_base, steer_ratio, 0.1, max_lat_accel, max_steer_angle)

        kp = 0.3  # TODO_TEST
        ki = 0.1
        kd = 0.0
        mn = 0.0  # Minimum throttle value
        mx = 0.6  # Maximum throttle value
        self.throttle_controller = PID(kp, ki, kd, mn, mx)

        tau = 0.5  # 1/(2pi*tau) = cutoff frequency
        ts = 0.02  # Sample time
        self.vel_lpf = LowPassFilter(tau, ts)

        self.vehicle_mass = vehicle_mass
        self.fuel_capacity = fuel_capacity
        brake_deadband = brake_deadband
        self.decel_limit = decel_limit
        self.accel_limit = accel_limit
        self.wheel_radius = wheel_radius

        self.last_time = rospy.get_time()

    def control(self, current_vel, dbw_enable, linear_vel, angular_vel):

        if not dbw_enable:
            self.throttle_controller.reset()
            return 0., 0., 0.

        # rospy.logwarn("Angular vel: {0}".format(angular_vel))
        # rospy.logwarn("Target vel: {0}".format(linear_vel))
        # rospy.logwarn("Current vel: {0}".format(current_vel))

        current_vel = self.vel_lpf.filt(current_vel)

        steering = self.yaw_controller.get_steering(
            linear_vel, angular_vel, current_vel)

        vel_error = linear_vel - current_vel
        self.last_vel = current_vel

        current_time = rospy.get_time()
        sample_time = current_time - self.last_time
        self.last_time = current_time

        throttle = self.throttle_controller.step(vel_error, sample_time)
        brake = 0

        if linear_vel == 0.0 and current_vel < 0.2:
            throttle = 0.0
            brake = 700  # N.m

        elif throttle < 0.1 and vel_error < 0.0:
            throttle = 0
            # decel = max(vel_error, self.decel_limit)
            # decel = max(vel_error, TARGET_DECEL_LIMIT)
            decel = min(vel_error,MAX_DECEL)
            # decel = TARGET_DECEL_LIMIT
            brake = abs(decel) * self.vehicle_mass * self.wheel_radius  # torque N*m

        # rospy.logwarn("Filtered vel: {0}".format(self.vel_lpf.get()))

        # Return throttle, brake, steer
        # rospy.logwarn("[TwistCon] Angular_vel: {:.2f}\tLinear_vel: {:.2f}\tCurrent_vel: {:.2f} | throttle: {:.2f}  brake: {:.2f}  steering: {:0.3f}".format(
            # angular_vel, linear_vel, current_vel, throttle, brake, steering))
        return throttle, brake, steering
