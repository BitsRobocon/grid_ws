cd $HOME/grid_ws/src/
wstool init
wstool merge https://raw.github.com/ros-controls/ros_control/noetic-devel/ros_control.rosinstall
wstool update
cd ..
rosdep install --from-paths . --ignore-src --rosdistro noetic -y
cd src
git clone https://github.com/ros-drivers/four_wheel_steering_msgs.git
git clone https://github.com/ros-controls/urdf_geometry_parser.git
catkin build