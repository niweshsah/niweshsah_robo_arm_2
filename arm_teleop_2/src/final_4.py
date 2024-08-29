#! /usr/bin/env python3

# Include the necessary libraries 
import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
from math import pi

class MyRobot:

    # Default Constructor
    def __init__(self, Group_Name):
        # Initialize the moveit_commander and rospy node
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('node_set_nearest_pose', anonymous=True)

        # Instantiate a RobotCommander object
        self._robot = moveit_commander.RobotCommander()
        # Instantiate a PlanningSceneInterface object
        self._scene = moveit_commander.PlanningSceneInterface()

        # Define the move group for the robot
        self._planning_group = Group_Name
        # Instantiate a MoveGroupCommander Object
        self._group = moveit_commander.MoveGroupCommander(self._planning_group)

        # Set planning time and number of attempts
        self._group.set_planning_time(10.0)
        self._group.set_num_planning_attempts(5)

        # Create action client for the Execute Trajectory action server
        self._execute_trajectory_client = actionlib.SimpleActionClient('execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
        self._execute_trajectory_client.wait_for_server()

        # Get the planning frame, end effector link and the robot group names
        self._planning_frame = self._group.get_planning_frame()
        self._eef_link = self._group.get_end_effector_link()
        self._group_names = self._robot.get_group_names()

        rospy.loginfo('\033[95m' + "Planning Group: {}".format(self._planning_frame) + '\033[0m')
        rospy.loginfo('\033[95m' + "End Effector Link: {}".format(self._eef_link) + '\033[0m')
        rospy.loginfo('\033[95m' + "Group Names: {}".format(self._group_names) + '\033[0m')
        rospy.loginfo('\033[95m' + " >>> MyRobot initialization is done." + '\033[0m')

    def move_to_nearest_pose(self, offset_x=0.1, offset_y=0.1, offset_z=0.1):
        # Get the current pose of the robot
        current_pose = self._group.get_current_pose().pose

        # Create a new goal pose slightly different from the current pose
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation = current_pose.orientation
        pose_goal.position.x = current_pose.position.x + offset_x
        pose_goal.position.y = current_pose.position.y + offset_y
        pose_goal.position.z = current_pose.position.z + offset_z

        # Set the goal pose and allow replanning if the initial plan fails
        self._group.set_pose_target(pose_goal)

        # Plan to the nearest possible pose
        plan_success, plan, planning_time, error_code = self._group.plan()
        if plan_success:
            # Create a goal message object for the action server
            goal = moveit_msgs.msg.ExecuteTrajectoryGoal()
            goal.trajectory = plan[1]  # The trajectory is the second element in the tuple returned by plan()

            # Send the goal to the action server
            self._execute_trajectory_client.send_goal(goal)
            self._execute_trajectory_client.wait_for_result()
            rospy.loginfo('\033[32m' + "Moved to the nearest possible pose." + '\033[0m')
        else:
            rospy.logwarn('\033[93m' + "Failed to find a valid plan to the target pose." + '\033[0m')

        rospy.loginfo(self._group.get_current_pose().pose)

    # Class Destructor
    def __del__(self):
        # Shutdown the moveit commander
        moveit_commander.roscpp_shutdown()
        rospy.loginfo('\033[95m' + "Object of class MyRobot Deleted." + '\033[0m')


def main():
    # Create a new arm object from the MyRobot class
    arm = MyRobot("arm_body")

    # Loop to move the arm to the nearest possible positions repeatedly
    while not rospy.is_shutdown():
        arm.move_to_nearest_pose(offset_x=0.0, offset_y=0.0, offset_z=0.01)
        rospy.sleep(3)

    # Delete the arm object at the end of the code
    del arm


if __name__ == '__main__':
    main()
