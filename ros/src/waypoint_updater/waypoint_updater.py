#!/usr/bin/env python
import numpy as np
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, TwistStamped
from styx_msgs.msg import Lane, Waypoint
from scipy.spatial import KDTree
from lowpass import LowPassFilter

import math

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
MAX_DECEL = 10.0
MIN_DECEL = 0.5
TARGET_DECEL = 2.0
LOOKAHEAD_WPS = 50  # Number of waypoints we will publish. You can change this number


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        self.base_lane = None
        self.pose = None
        self.stopline_wp_idx = -1
        self.waypoints_2d = None
        self.waypoint_tree = None
        self.current_vel = -1
        self.current_vel_filtered = -1
        self.decel_flag = False
        self.decel_stopline_ref = None
        self.decel_coeff = None
        self.decel_target = -1
        self.decel_params = []
        self.lookahead_wps = LOOKAHEAD_WPS

        tau = 0.5  # 1/(2pi*tau) = cutoff frequency
        ts = 0.02  # Sample time
        self.vel_lpf = LowPassFilter(tau, ts)

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        rospy.Subscriber('/current_velocity', TwistStamped, self.velocity_cb)
        self.debug = rospy.get_param('~debug_str')
        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        # rospy.Subscriber('/traffic_waypoint', PoseStamped, self.pose_cb)
        # rospy.Subscriber('/obstacle_waypoint', Lane, self.waypoints_cb)

        self.final_waypoints_pub = rospy.Publisher(
            'final_waypoints', Lane, queue_size=1)

        self.loop()

    def loop(self):
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            if self.pose and self.base_lane and self.waypoint_tree:
                self.publish_waypoints()
            rate.sleep()

    def get_closest_waypoint_idx(self):
        x = self.pose.pose.position.x
        y = self.pose.pose.position.y
        closest_idx = self.waypoint_tree.query([x, y], 1)[1]

        # Check if the closest waypoint is ahead or behind vehicle
        closest_coord = self.waypoints_2d[closest_idx]
        prev_coord = self.waypoints_2d[closest_idx - 1]

        # Equation for hyperplane through closest_coords
        cl_vect = np.array(closest_coord)
        prev_vect = np.array(prev_coord)
        pos_vect = np.array([x, y])

        val = np.dot(cl_vect-prev_vect, pos_vect-cl_vect)

        if val > 0:
            closest_idx = (closest_idx + 1) % len(self.waypoints_2d)
        return closest_idx

    def publish_waypoints(self):
        final_lane = self.generate_lane()
        self.final_waypoints_pub.publish(final_lane)

    def generate_lane(self):
        lane = Lane()

        closest_idx = self.get_closest_waypoint_idx()
        farthest_idx = closest_idx + self.lookahead_wps
        base_waypoints = self.base_lane.waypoints[closest_idx:farthest_idx]
        if farthest_idx > len(self.base_lane.waypoints):
            aux_wp = self.base_lane.waypoints[:(farthest_idx-len(self.base_lane.waypoints))]
            for wp in aux_wp: 
                base_waypoints.append(wp)
        lane.header = self.base_lane.header

        # Get the minimum target speed of the trajectory of waypoints
        # waypoint_vel = min(base_waypoints[0].twist.twist.linear.x,base_waypoints[-1].twist.twist.linear.x)

        # debug_str = "BASE [{:02.1} / {}]|".format(self.current_vel, (self.stopline_wp_idx - closest_idx))
        # for i, wp in enumerate(base_waypoints):
        #     debug_str += "{:02.1f}|".format(wp.twist.twist.linear.x)

        # rospy.logwarn(debug_str+"\n")

        # if (self.current_vel is not None) and (abs(waypoint_vel-self.current_vel) > 1.0):
        # rospy.logwarn("[W_U] Speed change trigger: {:.2f} -> {:.2f}".format(self.current_vel, waypoint_vel))

        # Check if the car should immediately accelerate
        # if waypoint_vel > self.current_vel:
        #     base_waypoints = self.accelerate_waypoints(base_waypoints)

        # TODO: Implement decelerate_waypoints in case th waypoint target speed is lower than
        # the current speed. In this project, this situation should not happen as we have a
        # fixed waypoint target speed and the car wouldn't drive faster than that.

        if self.stopline_wp_idx == -1 or (self.stopline_wp_idx >= farthest_idx):
            lane.waypoints = base_waypoints[:LOOKAHEAD_WPS]
        else:
            # if self.decel_stopline_ref != self.stopline_wp_idx:
            #     self.decel_coeff = None
            #     self.decel_target = -1
            lane.waypoints = self.decelerate_waypoints_tl(
                base_waypoints, closest_idx)[:LOOKAHEAD_WPS]

        if self.debug:
            debug_str = "[WU] spd:{:02.1f}/{:02.1f}, decel_target:{:02.2f}, wpID/to_tl:{:4.0f}/{:4.0f}] |".format(
                self.current_vel,self.current_vel_filtered, self.decel_target, closest_idx, (self.stopline_wp_idx - closest_idx))

            for i, wp in enumerate(lane.waypoints):
                debug_str += "{:2.1f}|".format(wp.twist.twist.linear.x)
                if i >= 40:
                    break

            rospy.logwarn(debug_str)
        return lane

    # def accelerate_waypoints(self, waypoints):
    #     temp = []
    #     reach_target_wp_speed = False
    #     for i, wp in enumerate(waypoints):
    #         p = Waypoint()
    #         p.pose = wp.pose

    #         # If the acceleration curve reaches the waypoint target speed, then just copy the rest of the waypoints
    #         if reach_target_wp_speed:
    #             p.twist.twist.linear.x = wp.twist.twist.linear.x

    #         else:
    #             # Calculate the distance from the first waypoint to the current iteration waypoint
    #             dist = self.distance_from_current_pose(waypoints,i)
    #             # Torricelli Equation with v_final = current_vel + 2*MAX_ACCEL*dist
    #             vel = math.sqrt(
    #                 math.pow(self.current_vel, 2) + 2.0 * MAX_ACCEL*dist)
    #             # if vel < 1.1:
    #             #     vel = 1.1
    #             # when reaches the waypoint target speed, set the flag
    #             if vel > wp.twist.twist.linear.x:
    #                 reach_target_wp_speed = True
    #                 p.twist.twist.linear.x = wp.twist.twist.linear.x
    #             else:
    #                 p.twist.twist.linear.x = vel

    #         temp.append(p)

    #     return temp

    # def decelerate_waypoints_tl(self, waypoints, closest_idx):
    #     temp = []
    #     for i, wp in enumerate(waypoints):

    #         p = Waypoint()
    #         p.pose = wp.pose

    #         # Three waypoints back from line so front of car stops at line
    #         stop_idx = max(self.stopline_wp_idx - closest_idx - 2, 0)
    #         dist = self.distance(waypoints, i, stop_idx)

    #         # Torricelli Equation with v_final = 0
    #         # TODO: Improve this create a new decel curve, continuous derivative -> S curve with accel ramp?
    #         vel = math.sqrt(2 * MAX_DECEL*dist)
    #         if vel < 0.1:
    #             vel = 0.0
    #         # if dist < 2.0:
    #         #     p.twist.twist.linear.x = min(vel, wp.twist.twist.linear.x,self.current_vel)
    #         # else:
    #         p.twist.twist.linear.x = min(vel, wp.twist.twist.linear.x)
    #         temp.append(p)

    #     return temp[:LOOKAHEAD_WPS]

    def decelerate_waypoints_tl(self, waypoints, closest_idx):
        # Three waypoints back from line so front of car stops at line
        stop_idx = max(self.stopline_wp_idx - closest_idx - 2, 0)
        # dist_tl = self.distance(waypoints, 0, stop_idx)
        dist_tl = self.distance_from_current_pose(waypoints, stop_idx)

        # Get the deceleration needed to stop the car before traffic lights, by Torricelli equation
        decel = math.pow(self.current_vel_filtered, 2) / (2*dist_tl)

        # if self.decel_coeff == None:
        #     self.decel_coeff = max(0.3, self.current_vel_filtered/dist_tl)
        #     self.decel_stopline_ref = self.stopline_wp_idx
        
        if self.decel_stopline_ref != self.stopline_wp_idx:
            self.decel_target = max(min(decel, MAX_DECEL),MIN_DECEL)
            if self.decel_target > MIN_DECEL:
                self.decel_stopline_ref = self.stopline_wp_idx

        # if self.decel_target < 0:
        #     self.decel_target = max(min(decel, MAX_DECEL),MIN_DECEL)
        #     self.decel_stopline_ref = self.stopline_wp_idx

        if decel > MAX_DECEL:
            rospy.logerr("The car won't have enough time to break, crossing traffic light: stop_idx={}, dist={:2.1f}, decel={:2.2f}, decel_coeff={}".format(
                stop_idx, dist_tl, decel, self.decel_coeff))
            return waypoints

        else:
            temp = []
            # Two waypoints back from line so front of car stops at line
            stop_idx = max(self.stopline_wp_idx - closest_idx - 2, 0)
            for i, wp in enumerate(waypoints):

                p = Waypoint()
                p.pose = wp.pose

                dist = self.distance(waypoints, i, stop_idx)

                # Torricelli Equation with v_final = 0
                # TODO: Improve this create a new decel curve, continuous derivative -> S curve with accel ramp?
                # vel = math.sqrt(2 * MAX_DECEL*dist)
                vel = math.sqrt(2 * self.decel_target * dist)
                if vel < 0.2 or (self.current_vel < 0.2 and dist < 3.0):
                    vel = 0.0
                # if dist < 2.0:
                #     p.twist.twist.linear.x = min(vel, wp.twist.twist.linear.x,self.current_vel)
                # else:
                p.twist.twist.linear.x = min(vel, wp.twist.twist.linear.x)
                temp.append(p)
                # rospy.logwarn("[WU]Decelerating: stop_idx={}, dist={:2.1f}, decel={:2.2f}, decel_coeff={:2.2f}".format(
                #     stop_idx, dist_tl, decel, self.decel_coeff))
            return temp

    # def decelerate_waypoints_tl_ramp(self, waypoints, closest_idx):

    #     # Three waypoints back from line so front of car stops at line
    #     stop_idx = max(self.stopline_wp_idx - closest_idx - 2, 0)
    #     # dist_tl = self.distance(waypoints, 0, stop_idx)
    #     dist_tl = self.distance_from_current_pose(waypoints, stop_idx)

    #     # Get the deceleration needed to stop the car before traffic lights, by Torricelli equation
    #     decel = math.pow(self.current_vel_filtered, 2) / (2*dist_tl)

    #     if self.decel_coeff == None:
    #         self.decel_coeff = max(0.3, self.current_vel_filtered/dist_tl)
    #         self.decel_stopline_ref = self.stopline_wp_idx

    #     if decel > MAX_DECEL:
    #         rospy.logerr("The car won't have enough time to break, crossing traffic light: stop_idx={}, dist={:2.1f}, decel={:2.2f}, decel_coeff={:2.2f}".format(
    #             stop_idx, dist_tl, decel, self.decel_coeff))
    #         return waypoints

    #     else:
    #         temp = []
    #         for i, wp in enumerate(waypoints):

    #             p = Waypoint()
    #             p.pose = wp.pose

    #             # dist = self.distance_from_current_pose(waypoints, i)
    #             dist = self.distance(waypoints, i, stop_idx)
    #             # Torricelli Equation with v_final = 0
    #             # TODO: Improve this create a new decel curve, continuous derivative -> S curve with accel ramp?
    #             # vel = math.sqrt(2 * MAX_DECEL*dist)

    #             # Linear deceleration
    #             # vel = 0.5*(dist_tl - dist)+1.0
    #             vel = self.decel_coeff*dist
    #             if vel < 0.2 or i >= stop_idx:
    #                 vel = 0.0
    #             # if dist < 2.0:
    #             #     p.twist.twist.linear.x = min(vel, wp.twist.twist.linear.x,self.current_vel)
    #             # else:
    #             p.twist.twist.linear.x = min(vel, wp.twist.twist.linear.x)
    #             temp.append(p)

    #         rospy.logwarn("[WU]Decelerating: stop_idx={}, dist={:2.1f}, decel={:2.2f}, decel_coeff={:2.2f}".format(
    #             stop_idx, dist_tl, decel, self.decel_coeff))
    #         return temp

    def pose_cb(self, msg):
        self.pose = msg

    def velocity_cb(self, msg):
        self.current_vel = msg.twist.linear.x
        self.current_vel_filtered = self.vel_lpf.filt(msg.twist.linear.x)
        self.lookahead_wps = LOOKAHEAD_WPS + int(self.current_vel_filtered*3)

    def waypoints_cb(self, waypoints):
        self.base_lane = waypoints
        if not self.waypoints_2d:
            self.waypoints_2d = [[waypoint.pose.pose.position.x,
                                  waypoint.pose.pose.position.y] for waypoint in waypoints.waypoints]
            self.waypoint_tree = KDTree(self.waypoints_2d)

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        self.stopline_wp_idx = msg.data

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0

        def dl(a, b): return math.sqrt(
            (a.x-b.x)**2 + (a.y-b.y)**2 + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position,
                       waypoints[i].pose.pose.position)
            wp1 = i
        return dist

    def distance_from_current_pose(self, waypoints, idx):
        dist = 0

        def dl(a, b): return math.sqrt(
            (a.x-b.x)**2 + (a.y-b.y)**2 + (a.z-b.z)**2)

        dist += dl(self.pose.pose.position,
                   waypoints[0].pose.pose.position)
        wp1 = 0
        for i in range(wp1, idx+1):
            dist += dl(waypoints[wp1].pose.pose.position,
                       waypoints[i].pose.pose.position)
            wp1 = i
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
