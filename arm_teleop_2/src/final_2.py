#!/usr/bin/env python3

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from moveit_msgs.msg import RobotState, Constraints, OrientationConstraint
import time

# Init node
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface_tutorial', anonymous=True)

# Define Interfaces
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()

# Define Move Group
group_name = "arm_body"
move_group = moveit_commander.MoveGroupCommander(group_name)

# Get reference frame
planning_frame = move_group.get_planning_frame()

# Get end-effector link
eef_link = move_group.get_end_effector_link()

# Get all groups in robot
group_names = robot.get_group_names()

# Get entire state
robot_state = robot.get_current_state()

# Get current pose
current_pose = move_group.get_current_pose().pose

# Create Goal Pose
pose_goal = geometry_msgs.msg.Pose()
pose_goal.orientation.w = current_pose.orientation.w
pose_goal.orientation.x = current_pose.orientation.x
pose_goal.orientation.y = current_pose.orientation.y
pose_goal.orientation.z = current_pose.orientation.z

# Move slightly far from current position
pose_goal.position.x = current_pose.position.x + 0.01  # Move 10 cm in x direction
pose_goal.position.y = current_pose.position.y + 0.01  # Move 10 cm in y direction
pose_goal.position.z = current_pose.position.z + 0.01  # Move 10 cm in z direction

# Set Goal Pose
move_group.set_pose_target(pose_goal)

# Obtain plan
plan = move_group.plan()

# Execute the plan
start_time = time.time()
# move_group.execute(plan[0], wait=True)

move_group.go(wait=True)
end_time = time.time()

# Stop group (to prevent any residual movement)
move_group.stop()

# Clear targets
move_group.clear_pose_targets()

# Verify the execution
print("Execution complete. Final pose:")
print(move_group.get_current_pose().pose)
print("Execution time:", end_time - start_time, "seconds")