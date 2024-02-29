FROM ros:foxy-ros-base-focal

# No keyboard when building
ENV DEBIAN_FRONTEND=noninteractive

# Python and Pip install
# RUN apt-get update && apt-get upgrade -y
RUN apt-get update
RUN apt-get install -y python3-pip
RUN python3 -m pip install -U pip

# Clone Virtuoso in ~/mrg/dev_ws/src
WORKDIR /root
RUN mkdir mrg
WORKDIR /root/mrg
RUN mkdir dev_ws
WORKDIR /root/mrg/dev_ws
RUN mkdir src
WORKDIR /root/mrg/dev_ws/src
RUN git clone -b foxy https://github.com/gt-marine-robotics-group/Virtuoso.git

# Clone ublox (GPS driver) in ~/mrg/dev_ws/src
RUN git clone https://github.com/aussierobots/ublox_dgnss
WORKDIR /root/mrg/dev_ws/src/ublox_dgnss
RUN git checkout 26cc57f

# Clone urg (Hokuyo Lidar driver) in ~/mrg/dev_ws/src
WORKDIR /root/mrg/dev_ws/src
RUN git clone -b foxy-devel https://github.com/ros-drivers/urg_node.git

# Clone microstrain (IMU driver) in ~/mrg/dev_ws/src
WORKDIR /root/mrg/dev_ws/src
RUN /bin/bash -c "git clone https://github.com/LORD-MicroStrain/microstrain_inertial.git"
WORKDIR /root/mrg/dev_ws/src/microstrain_inertial
RUN /bin/bash -c "git checkout a519cd4"

# Clone Depthai (OAK-D driver) in ~/mrg/dev_ws/src
WORKDIR /root/mrg/dev_ws/src
RUN /bin/bash -c "git clone -b foxy https://github.com/gt-marine-robotics-group/depthai-working.git"

# Rosdep install Virtuoso dependencies
WORKDIR /root/mrg/dev_ws
RUN /bin/bash -c "source /opt/ros/foxy/setup.bash && rosdep install --from-paths src --ignore-src -ry"

# Microros
WORKDIR /root/mrg
RUN mkdir microros_ws
WORKDIR /root/mrg/microros_ws
RUN /bin/bash -c "git clone -b foxy https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup"
RUN /bin/bash -c "source /opt/ros/foxy/setup.bash && rosdep install --from-paths src --ignore-src -y"
RUN /bin/bash -c "source /opt/ros/foxy/setup.bash && colcon build"
RUN /bin/bash -c "source /opt/ros/foxy/setup.bash && source install/local_setup.bash && ros2 run micro_ros_setup create_agent_ws.sh && ros2 run micro_ros_setup build_agent.sh"

# BAG-of-tricks
# set this arg to some value to rebuild the docker container starting here,
# useful if changes have been made to bag-of-tricks on github
ARG BAG_OF_TRICKS=unknown
WORKDIR /root/mrg
RUN /bin/bash -c "git clone -b docker https://github.com/gt-marine-robotics-group/bag-of-tricks.git"
RUN /bin/bash -c "cp /root/mrg/bag-of-tricks/tmux/.tmux.conf ~/"

# Virtuoso build
# set this arg to some value to rebuild the docker container starting here,
# useful if changes have been made to Virtuoso on github
ARG VIRTUOSO=unknown
WORKDIR /root/mrg/dev_ws/src/Virtuoso
RUN /bin/bash -c "git pull origin foxy"
WORKDIR /root/mrg/dev_ws
# in theory just using --packages-up-to virtuoso_autonomy should work, but docker was crashing computer
# during build (100% CPU and memory usage even when limiting memory during build)
RUN /bin/bash -c "source /opt/ros/foxy/setup.bash && colcon build --packages-up-to microstrain_inertial_driver"
RUN /bin/bash -c "source /opt/ros/foxy/setup.bash && colcon build --packages-up-to ublox_dgnss_node"
RUN /bin/bash -c "source /opt/ros/foxy/setup.bash && colcon build --packages-up-to depthai_ros_driver"
RUN /bin/bash -c "source /opt/ros/foxy/setup.bash && colcon build --packages-up-to urg_node"
RUN /bin/bash -c "source /opt/ros/foxy/setup.bash && colcon build --packages-up-to virtuoso_autonomy"

WORKDIR /root/mrg/dev_ws

# Some additional utility packages
RUN /bin/bash -c "sudo apt install -y usbutils vim tmux"
RUN /bin/bash -c "pip3 install tmuxp --ignore-installed"
