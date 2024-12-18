FROM osrf/ros:humble-desktop  
# nvidia-container-runtime
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=all

#ROS-Domain
ENV ROS_DOMAIN_ID=1
LABEL maintainer="kashg@seas.upenn.edu"

RUN mkdir -p /run/user/1000
RUN chmod 0700 /run/user/1000

RUN apt-get update && apt-get install -y \
    vim \
    python3-pip \
    iproute2 \
    ros-humble-tf2-eigen
RUN mkdir -p ~/ros_ws/src/motion_capture_system/
WORKDIR /root/ros_ws/src/motion_capture_system

# ROS2 Specific
COPY ./mocap_base ./mocap_base
COPY ./mocap_qualisys ./mocap_qualisys
COPY ./mocap_vicon ./mocap_vicon

# Dev Specific
# COPY ./bags ../../../
COPY ./.bashrc ../../../

WORKDIR /root/ros_ws/
#RUN . /opt/ros/humble/setup.sh && colcon build


