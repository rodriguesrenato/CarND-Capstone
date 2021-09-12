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
        self.decel_stopline_ref = None
        
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

        # Handle when the base_waypoints cross the end of the lane_waypoints vector
        if farthest_idx > len(self.base_lane.waypoints):
            aux_wp = self.base_lane.waypoints[:(
                farthest_idx-len(self.base_lane.waypoints))]
            for wp in aux_wp:
                base_waypoints.append(wp)
        lane.header = self.base_lane.header

        # Check if there is a red light ahead in trajectory len to start decelerating, otherwise keep ahead.
        # Even though the trajectory is built under the whole lookahead_wps length, the lane waypoint length is fixed at LOOKAHEAD_WPS
        if self.stopline_wp_idx == -1 or (self.stopline_wp_idx >= farthest_idx):
            lane.waypoints = base_waypoints[:LOOKAHEAD_WPS]
        else:
            lane.waypoints = self.decelerate_waypoints_tl(
                base_waypoints, closest_idx)[:LOOKAHEAD_WPS]

        # Show trajectory information and speeds on each following waypoints
        if self.debug:
            debug_str = "[WU] spd:{:02.1f}/{:02.1f}, decel_target:{:02.2f}, wpID/to_tl:{:4.0f}/{:4.0f}] |".format(
                self.current_vel, self.current_vel_filtered, self.decel_target, closest_idx, (self.stopline_wp_idx - closest_idx))

            for i, wp in enumerate(lane.waypoints):
                debug_str += "{:2.1f}|".format(wp.twist.twist.linear.x)
                if i >= 40:
                    break

            rospy.logwarn(debug_str)
        return lane

    def decelerate_waypoints_tl(self, waypoints, closest_idx):
        # Three waypoints back from line so front of car stops at line
        stop_idx = max(self.stopline_wp_idx - closest_idx - 2, 0)

        # Calculate the distance from car to the target stop waypoint
        dist_tl = self.distance_from_current_pose(waypoints, stop_idx)

        # Get the deceleration needed to stop the car before traffic lights, by Torricelli equation
        decel = math.pow(self.current_vel_filtered, 2) / (2*dist_tl)

        # Update the target deceleration once when find a new trafficlight
        if self.decel_stopline_ref != self.stopline_wp_idx:
            self.decel_target = max(min(decel, MAX_DECEL), MIN_DECEL)
            if self.decel_target > MIN_DECEL:
                self.decel_stopline_ref = self.stopline_wp_idx

        # Check if the dynamic calculated deceleration need to break the car on time will exceed the maximum limit, if so then keep ahead
        if decel > MAX_DECEL:
            rospy.logerr("The car won't have enough time to break, crossing traffic light: stop_idx={}, dist_tl={:2.1f}, decel_needed={:2.2f}, decel_target={}".format(
                stop_idx, dist_tl, decel, self.decel_target))
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
                vel = math.sqrt(2 * self.decel_target * dist)

                # Set this waypoint velocity to zero when calculate speed is too low or when 
                # the current speed is too low and it is close to the traffic light. This helps 
                # avoiding the resulting overshooting of the speed controller at the set point
                if vel < 0.2 or (self.current_vel < 0.2 and dist < 5.0):
                    vel = 0.0

                p.twist.twist.linear.x = min(vel, wp.twist.twist.linear.x)
                temp.append(p)
            return temp

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

    # Calculate Distance between two waypoints
    def distance(self, waypoints, wp1, wp2):
        dist = 0

        def dl(a, b): return math.sqrt(
            (a.x-b.x)**2 + (a.y-b.y)**2 + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position,
                       waypoints[i].pose.pose.position)
            wp1 = i
        return dist

    # Calculate Distance from current pose to the given waypoint
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
