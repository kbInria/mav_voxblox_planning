#!/usr/bin/env python3
# coding: utf-8

import rospy
import numpy as np

from std_srvs.srv import Empty
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseArray, PoseStamped, Pose
from mav_planning_msgs.srv import PlannerService, PlannerServiceRequest

##########################  Helper functions ######################

def EuclideanDistance(pose1, pose2):
    p1 = np.array([pose1.position.x, pose1.position.y, pose1.position.z])
    p2 = np.array([pose2.position.x, pose2.position.y, pose2.position.z])
    return np.linalg.norm(p2 - p1)

###################################################################

class GoalPoseHandler:
    """The goal pose handler receive a target pose in a reference frame. It then listen to the odometry of the agent 
        which is supposed to reached this goal and compute the path between the current pose of the agent and the goal (using mav-voxblox-planning)
        Then the waypoints of the path are sent one by one to the node responsible to convert a pose target into a velocity command to be sent to the actuators.
        NB: the odom, the best next viepoint and the waypoints are expected to be all in the same frame (usually the world).
            It is then transformed into the frame that the velocity command's is expecting (usually agent/map)
    """

    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('goal_pose_handler', anonymous=True)
        self._rate = rospy.Rate(10)  # 10 Hz

        # Initialize the class's attributes
        self._last_odom_pose = None
        self._next_best_viewpoint = None
        self._current_waypoints = None
        self._sent_waypoint = None
        self._distance_threshold_for_goal_reach = 0.1 # in m

        # Create a publisher
        self._next_waypoint_pub = rospy.Publisher("next_waypoint", Pose, queue_size=10)

        # Create subscribers
        rospy.Subscriber("groundtruth/odom", Odometry, self._OdometryCb)
        rospy.Subscriber("waypoint_list", PoseArray, self._WayPointListCb)
        rospy.Subscriber("next_best_viewpoint", PoseStamped, self._NextBestViewpointCb)

        # Create service clients
        service_timeout = 30
        try:
            rospy.wait_for_service('voxblox_rrt_planner/plan', service_timeout)
            rospy.wait_for_service('voxblox_rrt_planner/publish_path', service_timeout)
        except rospy.ROSException:
            rospy.logerror("failed to connect to services")
        self._planner_plan_srv = rospy.ServiceProxy('voxblox_rrt_planner/plan', PlannerService)
        self._planner_publish_path_srv = rospy.ServiceProxy('voxblox_rrt_planner/publish_path', Empty)

        rospy.loginfo("Finishing goal_pose_handler initialization.")

    def _OdometryCb(self, msg):
        # TODO: check if odom pose is in world frame
        self._last_odom_pose = msg

    def _WayPointListCb(self, msg):
        self._current_waypoints = msg

    def _NextBestViewpointCb(self, msg):
        """ Callback of the odometry msg. When receiving the odometry msg, 
            immediately call voxblox_rrt_planner to plan and publish the path between start and goal pose.

        Args:
            msg (nav_msgs.Odometry): Odometry of the agent that should go to the next best viewpoint
        """
        # Check if the waypoints can be computed
        if self._last_odom_pose is None:
            rospy.logdebug_throttle(60, "Did not receive sufficient information to compute waypoints.")
            return

        self._next_best_viewpoint = msg
        # Fill up start and goal pose in the path's mav_planning_msgs sevice request
        plannerRequest = PlannerServiceRequest()
        # plannerRequest.header.stamp = rospy.Time.now()
        # plannerRequest.header.frame_id = "world" # TODO: should be a parameter
        plannerRequest.start_pose.header = self._last_odom_pose.header
        plannerRequest.start_pose.pose = self._last_odom_pose.pose.pose # TODO: Should do twist as well?
        plannerRequest.goal_pose = self._next_best_viewpoint # TODO: should do goal_velocity as well?

        # Send the start and goal pose to the path's mav_planning_msgs service
        plannerResponse = self._planner_plan_srv(plannerRequest)
        # If a path could be planned between the start and the goal pose
        if plannerResponse.success:
            hasPublished = self._planner_publish_path_srv()
            if hasPublished:
                # Store the computed waypoints
                rospy.logdebug("planner request succeeded. Adding the following points: %r", self._current_waypoints)
        else:
            rospy.logwarn("The following request failed to generate a path: %r. \n Could not handle the next best view pose %r.", plannerRequest, self._next_best_viewpoint)

    def _UpdateWaypointToBeSent(self):
        """ The waypoints are sent one by one to the receiver so this function ensure that the current waypoint
            is reached before sending the next one. In the meantime, it resend the current target at a rate equal to self._rate. 
        """
        # Check if the waypoints can be sent
        if self._last_odom_pose is None or self._current_waypoints is None or not self._current_waypoints.poses:
            rospy.logdebug_throttle(60, "Did not receive sufficient information to send waypoints.")
            return
        
        # If it's the first spin after receiving the next best viewpoint, start sending the waypoints
        # or if the current goal (first element of self._current_waypoints) has been reached
        if not self._sent_waypoint or EuclideanDistance(self._last_odom_pose.pose.pose, self._sent_waypoint) < self._distance_threshold_for_goal_reach:
            # Update the waypoint to be sent
            self._sent_waypoint = self._current_waypoints.poses.pop(0)

        self._next_waypoint_pub.publish(self._sent_waypoint)

    def Spin(self):
        while not rospy.is_shutdown():
            self._UpdateWaypointToBeSent()
            self._rate.sleep()

if __name__ == "__main__":
    goalPoseHandler = GoalPoseHandler()
    goalPoseHandler.Spin()