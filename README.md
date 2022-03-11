# V-SLAMResources

**Install ROS**

http://wiki.ros.org/melodic/Installation/Ubuntu

**Create Catkin Workspace**

http://wiki.ros.org/catkin/Tutorials/create_a_workspace

**Install RTABMAP**

https://github.com/introlab/rtabmap_ros

**Process for running RTAB-Mapping**

$ cd ~/catkin_ws

$ source devel/setup.bash

$ roslaunch rtabmap_ros rtabmap.launch \rtabmap_args:="--delete_db_on_start" 
\depth_topic:=/camera/aligned_depth_to_color/image_raw 
\rgb_topic:=/camera/color/image_raw 
\camera_info_topic:=/camera/color/camera_info 
\approx_sync:=false localization:=false 
rviz:=true rtabmapviz:=false

**Inside of a separate terminal run this command to launch the Intel Real Sense Camera**

$  roslaunch realsense2_camera rs_camera.launch align_depth:=true
