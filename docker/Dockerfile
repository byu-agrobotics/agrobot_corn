# Created by Nelson Durrant, Oct 2024
FROM ros:humble-ros-base

ARG DEBIAN_FRONTEND=noninteractive
ARG TARGETARCH

# Define a username and password for the new user
ARG NAME=agrobot
ARG PASS=agrobot

# Update and upgrade
RUN apt update && apt upgrade -y


RUN apt update && apt install -y curl \
    wget \
    unzip \
    python3-pip

# Set up a new user
RUN useradd -ms /bin/bash $NAME
RUN usermod -aG sudo $NAME
RUN echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers
RUN usermod -aG video $NAME
RUN usermod -aG dialout $NAME
RUN groupadd docker
RUN usermod -aG docker $NAME
RUN echo "$NAME:$PASS" | chpasswd
USER $NAME

# APT Dependencies
USER root
WORKDIR /home/$NAME/agrobot_ws
RUN apt update && apt install -y \
    curl \
    rsync \
    git \
    vim \
    nano \
    tmux \
    x11-apps \
    x11-utils \
    x11-xserver-utils \
    xauth \
    ros-humble-rviz2 \
    ros-humble-rqt* \
    libopencv-dev \
    python3-opencv \
    v4l-utils \
    libgtk2.0-dev \
    libgl1-mesa-glx \
    libglib2.0-0 \
    libx11-xcb1 \
    libxrender1 \
    libxext6 \
    libxfixes3 \
    libxcomposite1 \
    libxcursor1 \
    libxdamage1 \
    libxi6 \
    libxtst6

USER $NAME

WORKDIR /home/$NAME

# Set up automatic ROS 2 sourcing and colorized output
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "source /home/$NAME/agrobot_ws/install/setup.bash" >> /home/$NAME/.bashrc
RUN echo "export RCUTILS_COLORIZED_OUTPUT=1" >> ~/.bashrc
RUN touch ~/.hushlogin

#PIP INSTALLS
RUN pip3 install tmuxp serial pyserial simple-pid pyserial

# Add local python binaries to the PATH
ENV PATH="$PATH:/home/$NAME/.local/bin"