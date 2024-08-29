# niweshsah_robo_arm_2

```bash
sudo apt-get install ros*controller*
```

Install the above command if ros controller is not moving



## Launching moveit package file

1.  **Make workspace (e.g. catkin_ws)**

     ```bash
    cd ~
    mkdir catkin_ws
    cd catkin_ws/
    
    ```

2.  **Clone the Repository**

    ```bash
    git clone https://github.com/niweshsah/Task2_Deimos_NiweshSah.git
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

   ```bash
    roslaunch moveit_pkg_2 full_robot_arm_launch.launch
   ```

