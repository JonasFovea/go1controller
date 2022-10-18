FROM ros:noetic

SHELL ["/bin/bash", "-c"]

# Finalize ROS setup
RUN apt-get -y update && apt-get -y install git &&  \
    echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc && \
    source ~/.bashrc && \
    apt-get install -y python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential

# initialize catkin workspace
RUN cd ~/.. && \
    source /opt/ros/noetic/setup.bash && \
    apt-get -y update && \
    apt-get -y install python3-catkin-tools && \
    mkdir -p ~/catkin_ws/src/ && \
    cd ~/catkin_ws/ && \
    catkin build && \
    source devel/setup.bash && \
    echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc && \
    source ~/.bashrc

# Install lcm and libglib
RUN apt-get install unzip wget && apt-get install -y libglib2.0-dev
RUN cd && \
    wget https://github.com/lcm-proj/lcm/releases/download/v1.4.0/lcm-1.4.0.zip && \
    unzip lcm-1.4.0.zip &&  mv lcm-1.4.0 lcm &&\
    rm lcm-1.4.0.zip && \
    cd lcm && mkdir build && cd build && cmake ../ && make && make install

# Install msgpack
RUN cd && \
    git clone https://github.com/msgpack/msgpack-c.git && \
    cd msgpack-c && \
    git checkout cpp_master && \
    cmake . && \
    sudo cmake --build . --target install

# Clone forked unitree repositories
RUN cd &&\
    git clone --branch fix https://github.com/JonasFovea/unitree_legged_sdk.git && \
    cd ~/catkin_ws/src/ && \
    git clone --branch chip https://github.com/JonasFovea/unitree_ros_to_real.git

# Fill ~/.bashrc
RUN echo "source /usr/share/gazebo-11/setup.sh" >> ~/.bashrc && \
    echo "export ROS_PACKAGE_PATH=~/catkin_ws:\${ROS_PACKAGE_PATH}" >> ~/.bashrc && \
    echo "export GAZEBO_PLUGIN_PATH=~/catkin_ws/devel/lib:\${GAZEBO_PLUGIN_PATH}" >> ~/.bashrc && \
    echo "export LD_LIBRARY_PATH=~/catkin_ws/devel/lib:\${LD_LIBRARY_PATH}" >> ~/.bashrc && \
    echo "export UNITREE_LEGGED_SDK_PATH=~/unitree_legged_sdk" >> ~/.bashrc && \
    echo 'export UNITREE_PLATFORM="amd64"' >> ~/.bashrc && \
    echo "export ROS_HOSTNAME=localhost" >> ~/.bashrc && \
    echo "export ROS_MASTER_URI=http://\${ROS_HOSTNAME}:11311" >> ~/.bashrc && \
    source ~/.bashrc

# Install ROS package dependencies
RUN rosdep update && \
    apt-get -y install ros-noetic-tf  \
        ros-noetic-ros-control  \
        ros-noetic-joint-state-controller  \
        ros-noetic-robot-state-publisher  \
        ros-noetic-gazebo-ros-pkgs  \
        ros-noetic-gazebo-ros-control \
        ros-noetic-xacro && \
        rm -rf /var/lib/apt/lists/*

# Building the SDK
RUN cd ~/unitree_legged_sdk && \
    mkdir build && \
    cd build && \
    cmake ../ && \
    make

# Building the packages
RUN  cd ~/catkin_ws && source devel/setup.bash && catkin build


# Istalling the repository content
COPY * ~/catkin_ws/src/go1controller
# build the package
RUN cd ~/catkin_ws/ && catkin build



CMD ["/bin/bash"]
