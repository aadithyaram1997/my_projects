#To run the project from scratch, follow the steps

mkdir -p ros_ws/src && cd ros_ws/src
git clone https://github.com/Tinker-Twins/AprilTag  #April Tag repository
git clone https://git.fh-aachen.de/mascor-public/arams/art_gallery -b 3at5 #Art gallery repository
git clone https://git.fh-aachen.de/vh5465s/arams_project_2022.git #Project repository
cd ..
colcon build --symlink-install
sudo apt install ros-foxy-gazebo* ros-foxy-tf-transformations ros-foxy-nav2* -y
sudo pip3 install transforms3d
pip3 install openvino opencv-python

#1st terminal
colcon build
. install/setup.bash
ros2 launch tb3_gazebo arams.launch.py

#2nd terminal
colcon build
. install/setup.bash
ros2 launch my_robot_slam initiate.launch.py

#3rd terminal
colcon build
. install/setup.bash
ros2 launch my_robot_slam begin.launch.py
