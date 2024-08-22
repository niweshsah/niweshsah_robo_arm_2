#!/usr/bin/env python3

import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import sys, select, termios, tty

class TeleopArmController:
    def __init__(self):
        # Define the keys and their corresponding joint indices
        self.joint_names = ['joint_0', 'joint_1', 'joint_2', 'joint_3', 'joint_4']
        # Initial joint angles
        self.joint_angles = [0.0] * len(self.joint_names)
        self.joint_step = 0.01  # Step size for changing joint angles
        # Initialize the selected joint
        self.selected_joint = 0

        self.settings = termios.tcgetattr(sys.stdin)

        rospy.init_node('teleop_arm')
        self.pub = rospy.Publisher('/robot_arm_controller/command', JointTrajectory, queue_size=10)
        self.rate = rospy.Rate(10)  # 10 Hz

        rospy.loginfo("Teleoperation node started. Select joint (0-4) and adjust using 'w' (increase) or 's' (decrease).")

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def run(self):
        while not rospy.is_shutdown():
            key = self.get_key()
            command_executed = False

            if key in '01234':
                self.selected_joint = int(key)
                rospy.loginfo(f"Joint {self.selected_joint} selected.")

            elif key == 'w':  # Increase joint angle
                self.joint_angles[self.selected_joint] += self.joint_step
                command_executed = True

            elif key == 's':  # Decrease joint angle
                self.joint_angles[self.selected_joint] -= self.joint_step
                command_executed = True

            elif key == '\x03':  # Ctrl+C
                rospy.loginfo("Shutting down teleoperation node.")
                break

            if command_executed:
                # Clamp the angles within a range if needed
                self.joint_angles[self.selected_joint] = max(-3.14, min(3.14, self.joint_angles[self.selected_joint]))
                rospy.loginfo(f"Adjusted joint {self.selected_joint} to {self.joint_angles[self.selected_joint]} radians.")

                # Create the JointTrajectory message
                trajectory_msg = JointTrajectory()
                trajectory_msg.joint_names = self.joint_names

                # Create a single point in the trajectory
                point = JointTrajectoryPoint()
                point.positions = self.joint_angles
                point.time_from_start = rospy.Duration(0.1)  # Small duration to indicate immediate movement

                trajectory_msg.points = [point]

                # Publish the trajectory message
                self.pub.publish(trajectory_msg)

                rospy.loginfo(f"Published joint angles: {self.joint_angles}")

            self.rate.sleep()

    def shutdown(self):
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)


if __name__ == '__main__':
    teleop_arm = TeleopArmController()
    try:
        teleop_arm.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        teleop_arm.shutdown()


















# import rospy
# from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
# import sys, select, termios, tty
# import threading

# class TeleopArmController:
#     def __init__(self):
#         # Define the keys and their corresponding joint indices
#         self.joint_names = ['joint_0', 'joint_1', 'joint_2', 'joint_3', 'joint_4']
#         # Initial joint angles
#         self.joint_angles = [0.0] * len(self.joint_names)
#         self.joint_step = 0.05  # Step size for changing joint angles
#         # Initialize the selected joint
#         self.selected_joint = 0

#         self.settings = termios.tcgetattr(sys.stdin)

#         rospy.init_node('teleop_arm')
#         self.pub = rospy.Publisher('/robot_arm_controller/command', JointTrajectory, queue_size=10)
#         self.rate = rospy.Rate(10)  # 10 Hz

#         rospy.loginfo("Teleoperation node started. Select joint (0-4) and adjust using 'w' (increase) or 's' (decrease). Press space to stop movement.")

#         self.key_pressed = None
#         self.lock = threading.Lock()

#     def get_key(self):
#         tty.setraw(sys.stdin.fileno())
#         select.select([sys.stdin], [], [], 0)
#         key = sys.stdin.read(1)
#         termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
#         return key

#     def listen_for_key_release(self):
#         while True:
#             key = self.get_key()
#             if key == '' or key == '\x03':  # Key release or Ctrl+C
#                 with self.lock:
#                     self.key_pressed = None
#             elif key == ' ':  # Space bar to stop movement
#                 with self.lock:
#                     self.key_pressed = 'stop'
#             else:
#                 with self.lock:
#                     self.key_pressed = key

#     def run(self):
#         key_release_thread = threading.Thread(target=self.listen_for_key_release)
#         key_release_thread.daemon = True
#         key_release_thread.start()

#         while not rospy.is_shutdown():
#             with self.lock:
#                 key = self.key_pressed
#                 self.key_pressed = None

#             if key is not None:
#                 if key == 'stop':  # Stop movement
#                     self.joint_angles = [0.0] * len(self.joint_names)
#                     rospy.loginfo("Movement stopped.")
#                 elif key in '01234':
#                     self.selected_joint = int(key)
#                     rospy.loginfo(f"Joint {self.selected_joint} selected.")

#                 elif key == 'w':  # Increase joint angle
#                     self.joint_angles[self.selected_joint] += self.joint_step

#                 elif key == 's':  # Decrease joint angle
#                     self.joint_angles[self.selected_joint] -= self.joint_step

#                 # Clamp the angles within a range if needed
#                 self.joint_angles[self.selected_joint] = max(-3.14, min(3.14, self.joint_angles[self.selected_joint]))

#                 # Create the JointTrajectory message
#                 trajectory_msg = JointTrajectory()
#                 trajectory_msg.joint_names = self.joint_names

#                 # Create a single point in the trajectory
#                 point = JointTrajectoryPoint()
#                 point.positions = self.joint_angles
#                 point.time_from_start = rospy.Duration(0.1)  # Small duration to indicate immediate movement

#                 trajectory_msg.points = [point]

#                 # Publish the trajectory message
#                 self.pub.publish(trajectory_msg)

#                 rospy.loginfo(f"Published joint angles: {self.joint_angles}")

#             # Add a small delay to allow continuous key presses
#             rospy.sleep(0.05)

#         self.shutdown()

#     def shutdown(self):
#         with self.lock:
#             self.key_pressed = None
#         termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)


# if __name__ == '__main__':
#     teleop_arm = TeleopArmController()
#     try:
#         teleop_arm.run()
#     except rospy.ROSInterruptException:
#         pass
#     except KeyboardInterrupt:
#         teleop_arm.shutdown()
#         print("Ctrl+C detected. Exiting...")