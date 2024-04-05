FROM osrf/ros:melodic-desktop-bionic

# Set environment 
ARG ROS_DISTRO=melodic 

ENV ROS_DISTRO=$ROS_DISTRO 

ENV ROS_VERSION=2 \ 
    ROS_PYTHON_VERSION=2

# Install Dependencies 
RUN apt update

RUN apt install -y \
    python-rosdep \ 
    python-pip \
    wget

RUN apt-get install -y \ 
    libcanberra-gtk-module dbus \
    dbus-x11 \
    xterm \
    git

# removed gnome-terminal
RUN pip install pyserial \
    pip install setuptools==2.0 \ 
    pip install vcstool \
    pip install pyserial \
    pip install setuptools -U \
    pip install scikit-image \
    pip install ordereddict

RUN python2 -m pip install install protobuf==3.17.3 \
    python2 -m pip install install tensorflow-gpu==1.12.0

# RUN apt install libcanberra-gtk-module libcanberra-gtk3-module -y

# Make directories 
RUN mkdir -p -- ~/ros_ws/src   

# Update rosdeps 
RUN cd ~/ros_ws/src \ 
    rosdep install -y \ 
    --from-paths src \  
    --ignore-src \  
    --skip-keys " \ 
    fastcdr \  
    rti-connext-dds-6.0.1 \ 
    urdfdom_headers"
 
WORKDIR /root/ros_ws 
RUN cd src/ \
    /bin/bash -c 'git clone https://github.com/m-rosette/IMML_cable_follow.git \
    git checkout main \
    cd .. \
    && source /opt/ros/melodic/setup.bash \
    && chmod -R 755 src \
    && catkin_make'
RUN apt install ros-melodic-ur-robot-driver -y
RUN apt upgrade -y 

RUN echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc 
RUN echo "source ~/ros_ws/devel/setup.bash" >> ~/.bashrc
RUN echo "cd ~/ros_ws" >> ~/.bashrc
RUN export ROS_PACKAGE_PATH=/ros_ws/:$ROS_PACKAGE_PATH

RUN echo "echo 'Terminal sourced using /opt/ros/melodic/setup.bash'" >> ~/.bashrc 