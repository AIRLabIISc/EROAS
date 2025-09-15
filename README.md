# EROAS
3D obstacle avoidance for AUV using 2.5D Forward Looking Sonar 




https://github.com/user-attachments/assets/0b70529d-3543-488e-9d04-d4fe210640c7






Refer to submitted paper - [Paper](https://arxiv.org/abs/2411.05516)

## Setup Instructions

1. **Create a ROS Workspace**
   - Clone this repository into the `src` folder of your ROS workspace.
   - The UUV_simulator in this repo is a modified version of [UUV Simulator](https://github.com/uuvsimulator/uuv_simulator) which has new worlds, and works for ROS, Ubuntu 20.04
   - Clone the Dave Simulator and the sonar plugin into the `src` folder as well.

2. **Install Dave Simulator**
   - Follow the [Dave Simulator Installation Guide](https://field-robotics-lab.github.io/dave.doc/).
   - Install the Multibeam Forward Looking Sonar plugin:
     - Documentation available [here](https://field-robotics-lab.github.io/dave.doc/contents/dave_sensors/Multibeam-Forward-Looking-Sonar/).
   - **Note**: A GPU and CUDA are required. We have tested with CUDA 12.4.
     - Follow the CUDA installation documentation on their page.
   - You may also download rviz_sonar_image package to visualise sonar data in RVIZ

3. **Manual Package Installation**
   - If Dave doesn't install all the packages on its own, manually download the following repositories:

     ```yaml
     dockwater:
       type: git
       url: git@github.com:Field-Robotics-Lab/dockwater.git
       version: main
     ds_msgs:
       type: git
       url: git@github.com:Field-Robotics-Lab/ds_msgs.git
       version: nps_dev
     ds_sim:
       type: git
       url: git@github.com:Field-Robotics-Lab/ds_sim.git
       version: nps_dev
     eca_a9:
       type: git
       url: git@github.com:uuvsimulator/eca_a9.git
       version: master
     rexrov2:
       type: git
       url: git@github.com:uuvsimulator/rexrov2.git
       version: master
     uuv_manipulators:
       type: git
       url: git@github.com:field-robotics-lab/uuv_manipulators
       version: master
     uuv_simulator:
       type: git
       url: git@github.com:field-robotics-lab/uuv_simulator
       version: master
     ```

   - After cloning, your `src` directory should look like:

     ```bash
     user@user:~/dave_ws/src$ ls
     marine_msgs
     dave
     navigator_auv
     rviz_sonar_image
     dockwater
     nps_uw_multibeam_sonar
     ds_msgs
     nps_uw_sensors_gazebo-master
     uuv_manipulators
     ds_sim
     uuv_simulator
     eca_a9
     rexrov2
     ```

4. **Build the ROS Workspace**
   - Build the workspace:

     ```bash
     cd ~/dave_ws
     catkin_make
     ```

   - Source the workspace:

     ```bash
     source devel/setup.bash
     ```

5. **Running the Algorithm**
   - To run the algorithm:

     ```bash
     roslaunch rexrov2_gazebo start_EROAS_demo.launch
     ```

   - To run a teleoperation node with CBF:

     ```bash
     roslaunch rexrov2_gazebo start_teleop_cbf.launch teleop_on:=true
     ```

6. **World Selection**
   - There are mainly **three worlds** you can open: **World A**, **World B**, and **Coral World**.
   - You can select the world by commenting or uncommenting lines in the `start_EROAS_demo.launch` file located at:

     ```
     rexrov2/rexrov2_gazebo/launch/start_EROAS_demo.launch (lines 6-24)
     ```

   - You will also need to comment or uncomment the corresponding world in the `ocean_waves.launch` file at:

     ```
     uuv_simulator/uuv_gazebo_worlds/launch/ocean_waves.launch (lines 9-16)
     ```

   - **Important**: Make sure to uncomment only one world at a time.

7. **Modifying Waypoints**
   - You can modify the waypoints or goals in the script:

     ```
     navigator_auv/scripts/only_gap.py
     ```

   - Edit line **59** (`self.waypoints`) to set your desired waypoints.
  
8. **Additional Tools**
   - The repository also contains a lot of useful code, such as:
     - **Location Spawners**
     - **Pose Data/Image Savers**
     - **Plotters**
     - **Point Cloud Generators**
     - **Sonar Motion Generator**
     - **Detection Modules**
     - **Topic Rate Modifier**

⭐ If you find this project helpful, please consider giving it a star! ⭐

   - These tools can be utilized for various tasks such as data collection, visualization, and enhancing the simulation environment.



