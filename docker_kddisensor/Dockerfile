# This is an auto generated Dockerfile for ros:ros-core
# generated from docker_images/create_ros_core_image.Dockerfile.em
FROM ubuntu:xenial

# install packages
RUN apt-get update && apt-get install -q -y \
    dirmngr \
    gnupg2 \
    lsb-release \
    && rm -rf /var/lib/apt/lists/*

# setup keys
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys 421C365BD9FF1F717815A3895523BAEEB01FA116

# setup sources.list
RUN echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list

# install bootstrap tools
RUN apt-get update && apt-get install --no-install-recommends -y \
    python-rosdep \
    python-rosinstall \
    python-vcstools \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# setup environment
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

# bootstrap rosdep
RUN rosdep init \
    && rosdep update

# install ros packages
ENV ROS_DISTRO kinetic
RUN apt-get update && apt-get install -y \
    ros-kinetic-ros-core=1.3.2-0* \
    && rm -rf /var/lib/apt/lists/*

# install kddisensor packages
RUN pip3 install requests==2.9.1 \
    pandas \
    matplotlib==2.2.3

# send_mail source pull from github
RUN git clone https://github.com/ayato-mizutani/send_mail.git

# add user ros
RUN adduser --gecos "ROS User" --disabled-password ros
RUN usermod -a -G dialout ros
USER ros
RUN HOME=/home/ros rosdep update

# create catkin_ws
RUN mkdir -p /home/ros/catkin_ws/src
RUN /bin/bash -c ". /opt/ros/kinetic/setup.bash; catkin_init_workspace /home/ros/catkin_ws/src"
RUN cp -r /send_mail/* /home/ros/catkin_ws/src/
RUN /bin/bash -c ". /opt/ros/kinetic/setup.bash; cd /home/ros/catkin_ws; catkin_make"

USER root

# setup entrypoint
COPY ./ros_entrypoint.sh /

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
