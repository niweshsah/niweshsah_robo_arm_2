# niweshsah_robo_arm_2

```bash
sudo apt-get install ros*controller*
sudo apt install ros-noetic-trac-ik
```

Install the above packages



## Launching moveit package file

1.  **Make workspace (e.g. catkin_ws)**

     ```bash
    cd ~
    mkdir catkin_ws
    cd catkin_ws/
    
    ```

2.  **Clone the Repository**

    ```bash
    git clone https://github.com/niweshsah/niweshsah_robo_arm_2.git
    ```

3. **Build the Package**

    Ensure you are in the workspace root directory (e.g., `catkin_ws`).

    ```bash
    catkin init
    catkin build
    ```
4. **Sourcing and setting the environment variables**:

     Type " nano ~/.bashrc " in terminal and add following lines at the end:

     ```bash
    source /opt/ros/noetic/setup.bash
     export TURTLEBOT3_MODEL=burger
     source ~/catkin_ws/devel/setup.bash
     ```

     Save the file and open the terminal:
      ```bash
    source ~/.bashrc
    ```

5. **Launch the file**

   Go to below launch file and uncomment the rviz portion from the launch file, if you also want to launch rviz

   ```bash
    roslaunch moveit_pkg_2 full_robot_arm_launch.launch
   ```


## Teleop code

 Open a new terminal and rosrun the codes in /arm_teleop_2/src folder. Work is under progress so they are giving errors
