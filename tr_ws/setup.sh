# Install ROS2
# https://index.ros.org/doc/ros2/Installation/Foxy/Linux-Install-Debians/

# Setup locale
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Setup sources
sudo apt update && sudo apt install curl gnupg2 lsb-release
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'

# Install ROS2 package
sudo apt update
sudo apt install ros-foxy-desktop

# Environment setup
source /opt/ros/foxy/setup.bash
sudo apt install python3-argcomplete

echo "HELLO and goodbye"

# Install rosdep 
sudo apt install python3-rosdep2
rosdep update

# Install colcon
# https://index.ros.org/doc/ros2/Tutorials/Colcon-Tutorial/
sudo apt install python3-colcon-common-extensions

# Make external_pkgs dir
mkdir ./src/external_pkgs
cd src/external_pkgs

# Add external package vision_opencv
# https://github.com/ros-perception/vision_opencv/tree/ros2/cv_bridge#build-and-test
git clone https://github.com/ros-perception/vision_opencv.git
cd vision_opencv
git checkout ros2
cd .. # WORKSPACE/src/external_pkgs

# Use rosdep to install dependencies in src and external_pkgs
cd .. # WORKSPACE/src
cd .. # WORKSPACE
rosdep install -i --from-path src --rosdistro foxy -y

# Install cv_bridge dependencies
# https://github.com/ros-perception/vision_opencv/tree/ros2/cv_bridge#installation

# First install OpenCV
# https://docs.opencv.org/master/d7/d9f/tutorial_linux_install.html
sudo apt-get install build-essential
sudo apt-get install cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev
cd ~/Downloads
wget https://github.com/opencv/opencv/archive/4.2.0.zip
mkdir -p opencv-4.2.0/build
cd opencv-4.2.0/build
cmake -D CMAKE_BUILD_TYPE=Release -D CMAKE_INSTALL_PREFIX=/usr/local ..
make -j7 # runs 7 jobs in parallel
sudo make install

# Other cv_bridge dependencies
sudo apt install python3-numpy
sudo apt install libboost-python1.71.0


# Example commands
# ros2 launch tr_pipeline_manager tr_pipeline.launch.py
# ros2 service call /tr/configure_pipeline tr_pipeline_interfaces/srv/ConfigurePipeline '{pipeline_type: {id:  "bar"}}' && ros2 action send_goal /tr/run_pipeline tr_pipeline_interfaces/action/RunPipeline '{input: 0}'
# ros2 launch tr_camera_driver tr_camera_driver.launch.py


