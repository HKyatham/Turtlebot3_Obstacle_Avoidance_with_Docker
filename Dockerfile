# Use the ROS 2 Humble Desktop Full image
FROM osrf/ros:humble-desktop-full

SHELL [ "/bin/bash", "-c" ]

# Install necessary packages
RUN apt-get update && apt-get install -y \
    terminator \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-turtlebot3* \
    python3-colcon-common-extensions \
    ros-humble-turtlebot4-desktop \
    git \
    && apt-get clean

# Install Git (if not installed earlier)
RUN apt-get update && apt-get install -y git

# Clone a ROS 2 package
RUN source /opt/ros/humble/setup.bash && \
    mkdir -p /ros_ws/src && \
    cd /ros_ws/src && \
    git clone https://github.com/HKyatham/Turtlebot3_Obstacle_Avoidance_with_Docker.git

# Set environment variables for ROS 2 and TurtleBot
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

# Build the ROS 2 workspace
RUN source /opt/ros/humble/setup.bash \
    && cd /ros_ws \
    && colcon build 

RUN echo "source /ros_ws/install/setup.bash" >> /root/.bashrc

# Set working directory
WORKDIR /ros_ws

# Allow GUI display through X11
ENV DISPLAY=:0
ENV QT_X11_NO_MITSHM=1

# Ensure X11 forwarding and Gazebo startup
ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && \
    source /ros_ws/install/setup.bash && \
    export DISPLAY=:0 && \
    ros2 launch enpm_turtlebot3_project competition_world.launch.py"]
