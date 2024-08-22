#!/usr/bin/env python3

# from __future__ import print_function
# from six.moves import input

# import sys
# import copy
# import rospy
# import moveit_commander
# import moveit_msgs.msg#!/usr/bin/env python3
# import geometry_msgs.msg
# from pynput import keyboard
# import time

# from math import pi, tau, dist, fabs, cos

# from std_msgs.msg import String
# from moveit_commander.conversions import pose_to_list

# def all_close(goal, actual, tolerance):
#     if type(goal) is list:
#         for index in range(len(goal)):
#             if abs(actual[index] - goal[index]) > tolerance:
#                 return False

#     elif type(goal) is geometry_msgs.msg.PoseStamped:
#         return all_close(goal.pose, actual.pose, tolerance)

#     elif type(goal) is geometry_msgs.msg.Pose:
#         x0, y0, z0, qx0, qy0, qz0, qw0 = pose_to_list(actual)
#         x1, y1, z1, qx1, qy1, qz1, qw1 = pose_to_list(goal)
#         d = dist((x1, y1, z1), (x0, y0, z0))
#         cos_phi_half = fabs(qx0 * qx1 + qy0 * qy1 + qz0 * qz1 + qw0 * qw1)
#         return d <= tolerance and cos_phi_half >= cos(tolerance / 2.0)

#     return True
#!/usr/bin/env python3

# class MoveGroupPythonInterfaceTutorial(object):

#     def __init__(self):
#         moveit_commander.roscpp_initialize(sys.argv)
#         rospy.init_node('move_group_python_interface_tutorial',
#                         anonymous=True)

#         robot = moveit_commander.RobotCommander()
#         scene = moveit_commander.PlanningSceneInterface()
#         group_name = "arm_body"
#         move_group = moveit_commander.MoveGroupCommander(group_name)

#         planning_frame = move_group.get_planning_frame()
#         eef_link = move_group.get_end_effector_link()
#         group_names = robot.get_group_names()

#         self.box_name = ''
#         self.robot = robot
#         self.scene = scene
#         self.move_group = move_group
#         self.planning_frame = planning_frame
#         self.eef_link = eef_link
#         self.group_names = group_names

#     def set_goal_pose_from_keyboard(self, dx=0, dy=0, dz=0):
#         pose_goal = geometry_msgs.msg.Pose()
#         pose_goal.orientation.w = 1.0

#         current_pose = self.move_group.get_current_pose().pose

#         pose_goal.position.x = current_pose.position.x + dx
#         pose_goal.position.y = current_pose.position.y + dy
#         pose_goal.position.z #!/usr/bin/env python3= current_pose.position.z + dz

#         return pose_goal

#     def update_goal_pose(self):
#         move_group = self.move_group

#         while True:
#             try:
#                 keys = []
#                 def on_press(key):
#                     keys.append(key)
#                 with keyboard.Listener(on_press=on_press) as listener:
#                     listener.join(timeout=1)

#                 dx, dy, dz = 0, 0, 0
#                 if keyboard.KeyCode(char='w') in keys:
#                     dz = 1
#                     rospy.loginfo("Pressed 'w' key")
#                 elif keyboard.KeyCode(char='s') in keys:
#                     dz = -1
#                     rospy.loginfo("Pressed 's' key")
#                 if keyboard.KeyCode(char='a') in keys:
#                     dx = -1
#                     rospy.loginfo("Pressed 'a' key")
#                 elif keyboard.KeyCode(char='d') in keys:
#                     dx = 1
#                     rospy.loginfo("Pressed 'd' key")
#                 if keyboard.KeyCode(char='q') in keys:
#                     dy = -1
#                     rospy.loginfo("Pressed 'q' key")
#                 elif keyboard.KeyCode(char='e') in keys:
#                     dy = 1
#                     rospy.loginfo("Pressed 'e' key")

#                 pose_goal = self.set_goal_pose_from_keyboard(dx, dy, dz)
#                 move_group.set_pose_target(pose_goal)

#                  # Increase planning time to 10 seconds
#                 move_group.set_planning_time(10.0)

#                 move_group.go(wait=True)
#                 move_group.stop()
#                 move_group.clear_pose_targets()

#                 time.sleep(1)
#             except KeyboardInterrupt:
#                 print("Exiting...")
#                 break

#             time.sleep(1)


# tutorial = MoveGroupPythonInterfaceTutorial()
# tutorial.update_goal_pose()




# from __future__ import print_function
# from six.moves import input

# import sys
# import rospy
# import moveit_commander
# import moveit_msgs.msg
# import geometry_msgs.msg
# from pynput import keyboard
# import time
# from math import dist, fabs, cos
# from moveit_commander.conversions import pose_to_list

# def all_close(goal, actual, tolerance):
#     if type(goal) is list:
#         for index in range(len(goal)):
#             if abs(actual[index] - goal[index]) > tolerance:
#                 return False
#     elif type(goal) is geometry_msgs.msg.PoseStamped:
#         return all_close(goal.pose, actual.pose, tolerance)
#     elif type(goal) is geometry_msgs.msg.Pose:
#         x0, y0, z0, qx0, qy0, qz0, qw0 = pose_to_list(actual)
#         x1, y1, z1, qx1, qy1, qz1, qw1 = pose_to_list(goal)
#         d = dist((x1, y1, z1), (x0, y0, z0))
#         cos_phi_half = fabs(qx0 * qx1 + qy0 * qy1 + qz0 * qz1 + qw0 * qw1)
#         return d <= tolerance and cos_phi_half >= cos(tolerance / 2.0)
#     return True

# class MoveGroupPythonInterfaceTutorial(object):

#     def __init__(self):
#         moveit_commander.roscpp_initialize(sys.argv)
#         rospy.init_node('move_group_python_interface_tutorial', anonymous=True)

#         robot = moveit_commander.RobotCommander()
#         scene = moveit_commander.PlanningSceneInterface()
#         group_name = "arm_body"  # Update this with your actual MoveIt group name
#         self.move_group = moveit_commander.MoveGroupCommander(group_name)

#         self.robot = robot
#         self.scene = scene

#     def set_goal_pose_from_keyboard(self, dx=0, dy=0, dz=0):
#         pose_goal = geometry_msgs.msg.Pose()
#         pose_goal.orientation.w = 1.0

#         current_pose = self.move_group.get_current_pose().pose

#         pose_goal.position.x = current_pose.position.x + dx
#         pose_goal.position.y = current_pose.position.y + dy
#         pose_goal.position.z = current_pose.position.z + dz

#         return pose_goal

#     def update_goal_pose(self):
#         move_group = self.move_group

#         while True:
#             try:
#                 keys = []
#                 def on_press(key):
#                     keys.append(key)
#                 with keyboard.Listener(on_press=on_press) as listener:
#                     listener.join(timeout=1)

#                 dx, dy, dz = 0, 0, 0
#                 if keyboard.KeyCode(char='w') in keys:
#                     dz = 1
#                     rospy.loginfo("Pressed 'w' key")
#                 elif keyboard.KeyCode(char='s') in keys:
#                     dz = -1
#                     rospy.loginfo("Pressed 's' key")
#                 if keyboard.KeyCode(char='a') in keys:
#                     dx = -1
#                     rospy.loginfo("Pressed 'a' key")
#                 elif keyboard.KeyCode(char='d') in keys:
#                     dx = 1
#                     rospy.loginfo("Pressed 'd' key")
#                 if keyboard.KeyCode(char='q') in keys:
#                     dy = -1
#                     rospy.loginfo("Pressed 'q' key")
#                 elif keyboard.KeyCode(char='e') in keys:
#                     dy = 1
#                     rospy.loginfo("Pressed 'e' key")

#                 pose_goal = self.set_goal_pose_from_keyboard(dx, dy, dz)
#                 move_group.set_pose_target(pose_goal)

#                 # Increase planning time
#                 move_group.set_planning_time(10.0)

#                 rospy.loginfo(f"Sending goal pose: {pose_goal}")

#                 success = move_group.go(wait=True)

#                 if success:
#                     rospy.loginfo("Move success!")
#                 else:
#                     rospy.logwarn("Move failed!")

#                 move_group.stop()
#                 move_group.clear_pose_targets()

#                 time.sleep(1)
#             except KeyboardInterrupt:
#                 print("Exiting...")
#                 break

#             time.sleep(1)

# tutorial = MoveGroupPythonInterfaceTutorial()
# tutorial.update_goal_pose()


















# from __future__ import print_function
# from six.moves import input

# import sys
# import rospy
# import moveit_commander
# import moveit_msgs.msg
# import geometry_msgs.msg
# from random import uniform
# import time

# from math import dist, fabs, cos
# from moveit_commander.conversions import pose_to_list

# def all_close(goal, actual, tolerance):
#     if type(goal) is list:
#         for index in range(len(goal)):
#             if abs(actual[index] - goal[index]) > tolerance:
#                 return False
#     elif type(goal) is geometry_msgs.msg.PoseStamped:
#         return all_close(goal.pose, actual.pose, tolerance)
#     elif type(goal) is geometry_msgs.msg.Pose:
#         x0, y0, z0, qx0, qy0, qz0, qw0 = pose_to_list(actual)
#         x1, y1, z1, qx1, qy1, qz1, qw1 = pose_to_list(goal)
#         d = dist((x1, y1, z1), (x0, y0, z0))
#         cos_phi_half = fabs(qx0 * qx1 + qy0 * qy1 + qz0 * qz1 + qw0 * qw1)
#         return d <= tolerance and cos_phi_half >= cos(tolerance / 2.0)
#     return True

# class MoveGroupPythonInterfaceTutorial(object):

#     def __init__(self):
#         moveit_commander.roscpp_initialize(sys.argv)
#         rospy.init_node('move_group_python_interface_tutorial', anonymous=True)

#         robot = moveit_commander.RobotCommander()
#         scene = moveit_commander.PlanningSceneInterface()
#         group_name = "arm_body"  # Update this with your actual MoveIt group name
#         self.move_group = moveit_commander.MoveGroupCommander(group_name)

#         self.robot = robot
#         self.scene = scene

#     def get_random_pose(self):
#         pose_goal = geometry_msgs.msg.Pose()
#         pose_goal.orientation.w = 1.0

#         # Define limits based on the expected workspace of your arm
#         pose_goal.position.x = uniform(-0.05, 0.05)  # Adjust the range as needed
#         pose_goal.position.y = uniform(-0.05, 0.05)  # Adjust the range as needed
#         pose_goal.position.z = uniform(0.0, 0.05)   # Adjust the range as needed

#         return pose_goal

#     def send_random_goal(self):
#         move_group = self.move_group

#         while True:
#             try:
#                 pose_goal = self.get_random_pose()
#                 move_group.set_pose_target(pose_goal)

#                 # Increase planning time
#                 move_group.set_planning_time(10.0)

#                 rospy.loginfo(f"Sending random goal pose: {pose_goal}")

#                 success = move_group.go(wait=True)

#                 if success:
#                     rospy.loginfo("Move success!")
#                 else:
#                     rospy.logwarn("Move failed!")

#                 move_group.stop()
#                 move_group.clear_pose_targets()

#                 time.sleep(5)  # Wait before sending the next goal
#             except KeyboardInterrupt:
#                 print("Exiting...")
#                 break

#             time.sleep(1)

# tutorial = MoveGroupPythonInterfaceTutorial()
# tutorial.send_random_goal()












from __future__ import print_function
from six.moves import input

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt

    tau = 2.0 * pi

    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))


class MoveGroupPythonInterfaceTutorial(object):
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("move_group_python_interface_tutorial", anonymous=True)

        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()
        group_name = "arm_body"
        move_group = moveit_commander.MoveGroupCommander(group_name)
        display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )

        planning_frame = move_group.get_planning_frame()
        eef_link = move_group.get_end_effector_link()
        group_names = robot.get_group_names()

        self.box_name = ""
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names

    def all_close(self, goal, actual, tolerance):
        if type(goal) is list:
            for index in range(len(goal)):
                if abs(actual[index] - goal[index]) > tolerance:
                    return False

        elif type(goal) is geometry_msgs.msg.PoseStamped:
            return self.all_close(goal.pose, actual.pose, tolerance)

        elif type(goal) is geometry_msgs.msg.Pose:
            x0, y0, z0, qx0, qy0, qz0, qw0 = pose_to_list(actual)
            x1, y1, z1, qx1, qy1, qz1, qw1 = pose_to_list(goal)
            d = dist((x1, y1, z1), (x0, y0, z0))
            cos_phi_half = fabs(qx0 * qx1 + qy0 * qy1 + qz0 * qz1 + qw0 * qw1)
            return d <= tolerance and cos_phi_half >= cos(tolerance / 2.0)

        return True

    def move_arm(self):
        current_pose = self.move_group.get_current_pose().pose

        new_pose = geometry_msgs.msg.Pose()
        new_pose.orientation = current_pose.orientation
        new_pose.position.x = current_pose.position.x + 0.1
        new_pose.position.y = current_pose.position.y + 0.1
        new_pose.position.z = current_pose.position.z

        self.move_group.set_pose_target(new_pose)

        success = self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()

        return success


def main():
    tutorial = MoveGroupPythonInterfaceTutorial()
    success = tutorial.move_arm()
    print("Success:", success)


if __name__ == "__main__":
    main()