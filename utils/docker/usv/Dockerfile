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
WORKDIR /root/mrg/dev_ws
RUN /bin/bash -c "source /opt/ros/foxy/setup.bash && colcon build --packages-up-to depthai_ros_driver"
RUN /bin/bash -c "source /opt/ros/foxy/setup.bash && colcon build --packages-select depthai_examples"

# Rosdep install Virtuoso dependencies
WORKDIR /root/mrg/dev_ws
RUN /bin/bash -c "source /opt/ros/foxy/setup.bash && rosdep install --from-paths src --ignore-src -ry"

# Some additional utility packages
RUN /bin/bash -c "sudo apt install -y usbutils vim"
# RUN /bin/bash -c "sudo apt install -y vim"

# Micro-Ros

# RUN mkdir /microros_ws
# WORKDIR /microros_ws
# RUN /bin/bash -c "git clone -b foxy https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup"

# RUN /bin/bash -c "sudo apt install -y python3-rosdep"
# RUN /bin/bash -c "sudo apt update && rosdep update"
# RUN /bin/bash -c "source /opt/ros/foxy/setup.bash && rosdep install --from-paths src --ignore-src -y"

# RUN /bin/bash -c "source /opt/ros/foxy/setup.bash && colcon build"
# RUN /bin/bash -c "source /opt/ros/foxy/setup.bash && source install/local_setup.bash && ros2 run micro_ros_setup create_agent_ws.sh && ros2 run micro_ros_setup build_agent.sh"

# RUN /bin/bash -c "pip3 install tmuxp --ignore-installed"
# RUN /bin/bash -c "sudo apt install tmux"

# # Microstrain

# WORKDIR /dev_ws/src
# RUN /bin/bash -c "git clone https://github.com/LORD-MicroStrain/microstrain_inertial.git"
# WORKDIR /dev_ws/src/microstrain_inertial
# RUN /bin/bash -c "git checkout a519cd4"

# WORKDIR /dev_ws

# RUN /bin/bash -c "source /opt/ros/foxy/setup.bash && colcon build --packages-up-to virtuoso_autonomy --packages-ignore microstrain_inertial microstrain_inertial_driver microstrain_inertial_msgs"

# # DEPTHAI

# ARG DEPTHAI=unknown
# WORKDIR /dev_ws/src
# RUN /bin/bash -c "git clone -b foxy https://github.com/gt-marine-robotics-group/depthai-working.git"
# WORKDIR /dev_ws
# RUN /bin/bash -c "source /opt/ros/foxy/setup.bash && colcon build --packages-up-to depthai_ros_driver"
# RUN /bin/bash -c "source /opt/ros/foxy/setup.bash && colcon build --packages-select depthai_examples"

# # Misc

# RUN /bin/bash -c "sudo apt install -y vim"

# # Microstrain again

# WORKDIR /dev_ws
# RUN /bin/bash -c "source /opt/ros/foxy/setup.bash && rosdep install --from-paths src -ry"
# RUN /bin/bash -c "source /opt/ros/foxy/setup.bash && colcon build --packages-up-to microstrain_inertial_driver"

# # Bag of Tricks stuff

# ARG BAG_OF_TRICKS=unknown
# WORKDIR /
# RUN /bin/bash -c "git clone -b docker https://github.com/gt-marine-robotics-group/bag-of-tricks.git"

# RUN /bin/bash -c "cp /bag-of-tricks/tmux/.tmux.conf ~/"

# # LATEST VIRTUOSO VERSION

# ARG VIRTUOSO=unknown
# WORKDIR /dev_ws/src/Virtuoso
# RUN /bin/bash -c "git pull origin foxy"
# WORKDIR /dev_ws
# RUN /bin/bash -c "source /opt/ros/foxy/setup.bash && colcon build --packages-up-to virtuoso_autonomy"

# WORKDIR /dev_ws
