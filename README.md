# ORB SLAM2 AUTO

To set up this project, do the following:

- Execute the build shell script, allow ORB_SLAM2 to build its components.
- run colcon build on the parent directory.
- Ensure ROS2 Foxy, latest version of OpenCV and Pangolin are installed.
- Python 3.8, numpy and python OpenCV are required.

To run these modules, do the following(ensure **parent-directory** is replaced):

- Start up a new instance of Gazebo (any world can be used).
- Import the 'model_editor_models' directory and generate a new 'TinyBot' robot into the world.
- Source the packages source files via 'source **parent-directory**/install/setup.bash'.
- Run 'ros2 run pointcloud_mapper pointcloud_mapper'.
- Untar 'ORBvoc.txt' inside 'ORB_SLAM2/Vocabulary'.
- Run 'ros2 run ros2_orbslam rgbd **parent-directory**/ORB_SLAM2/Vocabulary/ORBvoc.txt **parent-directory**/ORB_SLAM2-configuration/rgbd/TUM1.yaml'.
- Run 'ros2 run auto_twist_control auto_twist_control'.
