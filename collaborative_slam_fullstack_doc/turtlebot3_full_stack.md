This document aims to introduce how to build full stack collaborative slam system. In this document, it will first introduce the hardware setup and software setup of the system. You can build your own full stack system based on the guide of these two sections. Then, it will introduce the all-in-one ROS2 launch file in detail. You can have your own launch file based on it. We use TurtleBot3 (waffle_pi) Robot in our experiment.
### 1 Hardware setup ###
For hardware setup, there is only one thing you need to do: **place the sensor in the appropriate position and determine the extrinsic matrix according to the position.**
We use [TurtleBot3 (waffle_pi)](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/) as the test platform as shown in the figure 1. Comparing with the original robot, we make the following two changes:
- Add NUC: We place a [NUC](https://ark.intel.com/content/www/us/en/ark/products/205073/intel-nuc-11-performance-kit-nuc11pahi7.html) with an Intel i7 2.8GHz CPU, 16G RAM, used for running collaborative SLAM and navigation algorithm. (Of course, we also place a portable battery used for supplying power for NUC)

- Add-on camera: [RealSense D455](https://www.intelrealsense.com/depth-camera-d455/) is used to replace the original camera on TurtleBot3. D455 is mannually sticked on the position of the original camera so that the extrinsic matrix is similiar to the original one. 

Note: 
- The extrinsic matrix here refers to the transformation matrix between "/camera_link" and "/base_link". The definitions of the two different coordinates are explained in the figure 1. You need to manually measure it and publish it onto the TF tree when launching the entire system.

<p align="center">
<img style="border-radius: 0.3125em;
    box-shadow: 0 2px 4px 0 rgba(34,36,38,.12),0 2px 10px 0 rgba(34,36,38,.08);" 
    src="https://github.com/zhangyang-intel/turtlebot3_modified/blob/foxy-devel/collaborative_slam_fullstack_doc/coordinate_defination.png ">
    <br>
    <div align="center" style="color:orange; border-bottom: 1px solid #d9d9d9;
    display: inline-block;
    color: #999;
    padding: 2px;">Figure 1. turtlebot3 coordinate defination</div>
</p>

### 2 Software setup ###
For software setup, there are totally 4 components need to be installed: TurtleBot3 robot driver, RealSense camera driver, Collaborative SLAM and Navigation.
#### 2.1 TurtleBot3 robot driver ####
Follow the [instructions for TurtleBot3 setup](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/#pc-setup).

- Note:
The original source code of package "ros-foxy-turtlebot3" is [here](https://github.com/ROBOTIS-GIT/turtlebot3). Our patch for the original TurtleBot3 repo is [here](./0001-fix-the-problem-that-the-odometry-may-publish-an-out.patch). This patch mainly includes two modifications: one is adding additional logic to filter the outlier odometry data; and the other is to modify the description file in order not to publish transformation between "camera_link" (default camera of TurtleBot3 robot) and "base_link" to tf_tree.

```
cd YOUR_COLCON_WORKSPACE/src
# place repos under the src foler
git clone -b foxy-devel https://github.com/ROBOTIS-GIT/DynamixelSDK.git
git clone -b foxy-devel https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
git clone -b foxy-devel https://github.com/ROBOTIS-GIT/turtlebot3.git
cd ./turtlebot3
git apply 0001-fix-the-problem-that-the-odometry-may-publish-an-out.patch
cd ..
source /opt/ros/foxy/setup.bash
colcon build
```

#### 2.2 RealSense camera driver ####

Follow the [instructions for ros-realsense setup](https://github.com/IntelRealSense/realsense-ros/tree/ros2) and we choose the method 2 to build.

To test the connection, you can use commands like ```dmesg``` or ```lsusb```. You may further verify using ```realsense-viewer```. You need to use USB3.0 cable to connect RealSense camera to the USB3.0 interface of your machine. 

#### 2.3 Collaborative SLAM ####
Follow the [github readme](https://github.com/intel-innersource/applications.robotics.mobile.collaborative-slam/tree/main) to build.

#### 2.4 Navigation ####
Follow the [github readme](https://github.com/ros-planning/navigation2/tree/main/nav2_bringup) and [installation guide](https://navigation.ros.org/getting_started/index.html#installation) to build.

### 3 Launch file introduction ###
This [ROS2 launch file](./full_stack_turtlebot3.launch.py) is what we use in our experiment and you can have your own ROS2 launch file to meet your specific requirements. This launch file launch the following 5 parts:  
#### 3.1 Collaborative SLAM ####
For the tracker node, we use this [launch file](https://github.com/intel-innersource/applications.robotics.mobile.collaborative-slam/blob/main/tracker/launch/tracker.launch.py). The following parameters need to be noted:
- tracker_rviz: This parameter is recommended to set to true in mapping mode, you can set a navigation goal for the robot on the tracker rviz window.
- traj_store_path/octree_store_path: The default values of these parameters are empty and corresponding results will not be saved. So you need to specify your own path to store the trajectory of frames and Octree map result.

For the server node, we use this [launch file](https://github.com/intel-innersource/applications.robotics.mobile.collaborative-slam/blob/main/server/launch/server.launch.py). The following parameters need to be noted:
- server_rviz: The default value of this parameter is true and you can set a navigation goal for the robot on the server rviz window in localization mode.
- fix_scale: It should set to true when using RGBD camera setup and the scale will not be optimized during optimization.
- save_map_path/save_traj_folder: The default values of these parameters are empty and corresponding results will not be saved. So you need to specify your own path to store the trajectory of keyframes and point cloud map result.

#### 3.2 Turtlebot3 robot driver ####
This part is used to control the movement of TurtleBot3 robot. It publishes the TF tree as shown in the blue box in the figure 2. But for collaborative slam algorithm, only odom and base_link are compulsory while the other TF frames are not essential.

-Note: For other robots, there are only two things you need to do: first, find the robot driver; second, publish the odometry data onto TF tree as a transformation matrix between "/odom" and "/base_link" frames.

#### 3.3 Static_tf ####
Here, we use "tf2_ros" package to publish transform between "/base_link" and "/camera_link" as shown in the red box in the figure 2. The transform is the extrinsic matrix determined by the position of the camera on the robot.
- Note: If you use other sensors, such as lidar, you should also publish transform between "/lidar_link" and "/camera_link".

#### 3.4 Realsense camera driver ####
We use the official launch file. Please refer to [here](https://github.com/IntelRealSense/realsense-ros) for more details. This part publishes the tf tree as shown in the green box in the the figure 2.  There are three parameters needed to be noted:
- align_depth: Set to true to publish the depth image aligned to the color image on the topic "/camera".
- device_type: Set the type according to the camera you use (we use RealSense D455, so choose d455)
- initial_reset: It is recommended to set to true, which will allow the RealSense to reset on every start.

#### 3.5 Navigation ####
The original launch file "navigation_launch.py" is [here](https://github.com/ros-planning/navigation2/blob/main/nav2_bringup/launch/navigation_launch.py). We add an interface for setting paremeter "remap_map_id". Please refer to [this](./navigation_launch.py). The corresponding parameter configuration file is [here](waffle_pi.yaml). The following parameters need to be noted:
- start_navigation_node: It should set to true if you want launch navigation module.
- nav_params_file: The path of parameter configuration file of navigation module.
- remap_map_id: Navigation related nodes defaultly subscribe topic "/map" for occupancy map information. In mapping mode, the tracker publish "/map" topic. But in localization mode, the server publish topic "/univloc_tracker_0/map", so this parameter should be set to "/univloc_tracker_0/map" in this situation.

<p align="center">
<img style="border-radius: 0.3125em;
    box-shadow: 0 2px 4px 0 rgba(34,36,38,.12),0 2px 10px 0 rgba(34,36,38,.08);" 
    src="https://github.com/zhangyang-intel/turtlebot3_modified/blob/foxy-devel/collaborative_slam_fullstack_doc/tf_tree.png">
    <br>
    <div align="center" style="color:orange; border-bottom: 1px solid #d9d9d9;
    display: inline-block;
    color: #999;
    padding: 2px;">Figure 2. Recommended TF Tree</div>
</p>

### 4 Example ###
- In mapping mode, launch the entire system and save all results.
```
ros2 launch full_stack_turtlebot3.launch.py traj_store_path:=/YOUR_PATH_TO_SAVE/ save_traj_folder:=/YOUR_PATH_TO_SAVE/ save_map_path:=/YOUR_PATH_TO_SAVE/pointcloud_map.msg octree_store_path:=/YOUR_PATH_TO_SAVE/octree_map.bin
```
You will save the following results.

<p align="center">
<img style="border-radius: 0.3125em;
    box-shadow: 0 2px 4px 0 rgba(34,36,38,.12),0 2px 10px 0 rgba(34,36,38,.08);" 
    src="https://github.com/zhangyang-intel/turtlebot3_modified/blob/foxy-devel/collaborative_slam_fullstack_doc/result1.png">
    <br>
    <div align="center" style="color:orange; border-bottom: 1px solid #d9d9d9;
    display: inline-block;
    color: #999;
    padding: 2px;">Figure 3. results saved using command1</div>
</p>

- In localization mode, launch the entire system to load prior map and navigate.
```
ros2 launch full_stack_turtlebot3.launch.py octree_load_path:=/YOUR_PATH_TO_LOAD/octree_map.bin load_map_path:=/YOUR_PATH_TO_LOAD/pointcloud_map.msg traj_store_path:=/YOUR_PATH_TO_SAVE/ slam_mode:=localization server_mode:=localization remap_map_id:=/univloc_server/map tracker_rviz:=false server_rviz:=true map_frame:="map-0" enable_fast_mapping:=false use_odom:=true
```

<p align="center">
<img style="border-radius: 0.3125em;
    box-shadow: 0 2px 4px 0 rgba(34,36,38,.12),0 2px 10px 0 rgba(34,36,38,.08);" 
    src="https://github.com/zhangyang-intel/turtlebot3_modified/blob/foxy-devel/collaborative_slam_fullstack_doc/result2.png">
    <br>
    <div align="center" style="color:orange; border-bottom: 1px solid #d9d9d9;
    display: inline-block;
    color: #999;
    padding: 2px;">Figure 4. results saved using command2</div>
</p>