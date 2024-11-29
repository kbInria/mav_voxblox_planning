#!/usr/bin/env python3
# coding: utf-8

import rospy

from geometry_msgs.msg import Pose
from trajectory_msgs.msg import MultiDOFJointTrajectory

##########################  Helper functions ######################

def ConvertTrajectoryPointToPose(point):
    """Convert a MultiDOFJointTrajectoryPoint to a Pose
    Cf https://docs.ros.org/en/noetic/api/trajectory_msgs/html/msg/MultiDOFJointTrajectory.html
    
    Args:
        point (MultiDOFJointTrajectoryPoint): a trajectory point is composed of transforms for each of its DOF (here 1), a velocity and an acceleration.
    """
    pointTransform = point.transforms[0] # Because we only have 1DOF here
    convertPose = Pose()
    convertPose.position.x = pointTransform.translation.x
    convertPose.position.y = pointTransform.translation.y
    convertPose.position.z = pointTransform.translation.z
    convertPose.orientation = pointTransform.rotation

    return convertPose

###################################################################

class GoalPoseHandler:

    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('goal_pose_handler', anonymous=True)
        self._rate = rospy.Rate(10)  # 10 Hz

        # Initialize the class's attributes
        self._trajectory_waypoints = None  # List of waypoints that compose the trajectory
        self._distance_threshold_for_reaching_waypoint = 0.1 # in m TODO: should be in angle as well

        # Create a publisher
        self._next_waypoint_pub = rospy.Publisher("next_waypoint", Pose, queue_size=10)

        # Create a subscriber
        rospy.Subscriber("command/trajectory", MultiDOFJointTrajectory, self._TrajectoryCB)

        rospy.loginfo("Finishing goal_pose_handler initialization.")

    def _TrajectoryCB(self, msg):
        self._trajectory_waypoints = msg

        if self._trajectory_waypoints is None or not self._trajectory_waypoints.points:
            return

        for point in self._trajectory_waypoints.points:
            waypoint = ConvertTrajectoryPointToPose(point)
            self._next_waypoint_pub.publish(waypoint)

    def Spin(self):
        while not rospy.is_shutdown():
            self._rate.sleep()

if __name__ == "__main__":
    goalPoseHandler = GoalPoseHandler()
    goalPoseHandler.Spin()