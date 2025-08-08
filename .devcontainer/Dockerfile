###########################################
# Base image
###########################################
FROM ubuntu:24.04 AS base

ENV DEBIAN_FRONTEND=noninteractive

# Install language
RUN apt-get update && apt-get install -y \
  locales \
  && locale-gen en_US.UTF-8 \
  && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
  && rm -rf /var/lib/apt/lists/*
ENV LANG en_US.UTF-8

# Install timezone
RUN ln -fs /usr/share/zoneinfo/UTC /etc/localtime \
  && export DEBIAN_FRONTEND=noninteractive \
  && apt-get update \
  && apt-get install -y tzdata \
  && dpkg-reconfigure --frontend noninteractive tzdata \
  && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get -y upgrade \
  && rm -rf /var/lib/apt/lists/*

# Install common programs
RUN apt-get update && apt-get install -y --no-install-recommends \
  curl \
  gnupg2 \
  lsb-release \
  sudo \
  software-properties-common \
  wget \
  && rm -rf /var/lib/apt/lists/*

# Install ROS2

RUN export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}') \
  && curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb" \
  && sudo dpkg -i /tmp/ros2-apt-source.deb

RUN apt-get update && apt-get install -y --no-install-recommends \
  ros-jazzy-desktop \
  python3-argcomplete \
  ros-jazzy-rmw-zenoh-cpp \
  && rm -rf /var/lib/apt/lists/*

ENV ROS_DISTRO=jazzy
ENV AMENT_PREFIX_PATH=/opt/ros/jazzy
ENV COLCON_PREFIX_PATH=/opt/ros/jazzy
ENV LD_LIBRARY_PATH=/opt/ros/jazzy/lib
ENV PATH=/opt/ros/jazzy/bin:$PATH
ENV PYTHONPATH=/opt/ros/jazzy/lib/python3.10/site-packages
ENV ROS_PYTHON_VERSION=3
ENV ROS_VERSION=2
ENV DEBIAN_FRONTEND=

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
  bash-completion \
  build-essential \
  cmake \
  gdb \
  gdbserver \
  git \
  openssh-client \
  python3-argcomplete \
  python3-pip \
  ros-dev-tools \
  ros-jazzy-ament-* \
  vim \
  ranger \
  htop \
  && rm -rf /var/lib/apt/lists/*

RUN rosdep init || echo "rosdep already initialized"

ARG USERNAME=ros
ARG USER_UID=1001
ARG USER_GID=$USER_UID

# Create a non-root user

RUN if ! getent group $USER_GID; then groupadd -g $USER_GID $USERNAME; fi \
  && if ! id -u $USER_UID >/dev/null 2>&1; then \
  useradd -m -u $USER_UID -g $USER_GID -s /bin/bash $USERNAME; \
  fi \
  && apt-get update \
  && apt-get install -y sudo \
  && echo "$USERNAME ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME \
  && chmod 0440 /etc/sudoers.d/$USERNAME \
  && rm -rf /var/lib/apt/lists/*

# Set up autocompletion for user
RUN apt-get update && apt-get install -y git bash-completion \
  && echo "if [ -f /opt/ros/${ROS_DISTRO}/setup.bash ]; then source /opt/ros/${ROS_DISTRO}/setup.bash; fi" >> /home/$USERNAME/.bashrc \
  && echo "if [ -f /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash ]; then source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash; fi" >> /home/$USERNAME/.bashrc \
  && rm -rf /var/lib/apt/lists/*

# Install Mesa for software OpenGL
RUN apt-get update && apt-get install -y \
    mesa-utils \
    libgl1 \
    libglx-mesa0 \
    libgl1-mesa-dri \
    libglu1-mesa \
    && rm -rf /var/lib/apt/lists/*

# Optional: Install TurboVNC for fast rendering
RUN apt-get update && apt-get install -y \
    xvfb \
    x11vnc \
    fluxbox \
    && rm -rf /var/lib/apt/lists/*

RUN adduser ros dialout

ENV DEBIAN_FRONTEND=
ENV AMENT_CPPCHECK_ALLOW_SLOW_VERSIONS=1

ARG WORKSPACE=/home/${USERNAME}

ENV DEPENDENCIES_WS /opt/dependencies_ws
RUN mkdir -p $DEPENDENCIES_WS/src

ENV HOME=/home/${USERNAME}
ENV WS=${WORKSPACE}

ARG CMAKE_BUILD_TYPE=RelWithDebInfo

# Build ROS 2 source dependencies
WORKDIR $DEPENDENCIES_WS
COPY ros_dependencies.repos ros_dependencies.repos
RUN vcs import src < ros_dependencies.repos
RUN apt-get update && \
  rosdep update && \
  rosdep install -q -y --from-paths src --ignore-src && \
  rm -rf /var/likb/apt/lists/*

RUN . /opt/ros/jazzy/setup.sh && colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=$CMAKE_BUILD_TYPE

# Install PIP requirements
COPY requirements.txt /tmp/
RUN python3 -m pip install --no-cache-dir --break-system-packages -r /tmp/requirements.txt

ENV DEBIAN_FRONTEND=dialog

# Create source folder
WORKDIR $WS
RUN mkdir ws

# Install ros dependencies
COPY --chown=$USERNAME . ws

RUN apt-get update && \
  . $DEPENDENCIES_WS/install/setup.sh && \
  rosdep update && \
  rosdep install --from-paths ws/src --ignore-src -y && \
  rm -rf /var/lib/apt/lists/*

USER $USERNAME

RUN sudo chown -R ros:ros /home/ros

# Build ros2 workspace
RUN /bin/bash -c 'source $DEPENDENCIES_WS/install/setup.bash && cd /home/ros && colcon build --merge-install --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=$CMAKE_BUILD_TYPE'

# Add source to bashrc
RUN echo "if [ -f /home/ros/install/setup.bash ]; then source /home/ros/install/setup.bash; fi" >> /home/ros/.bashrc
RUN echo "if [ -f $DEPENDENCIES_WS/install/setup.bash ]; then source $DEPENDENCIES_WS/install/setup.bash; fi" >> /home/ros/.bashrc
RUN echo "if [ -f ${WS}/ws/run-completion ]; then source ${WS}/ws/run-completion; fi" >> ${HOME}/.bashrc

RUN echo "export RCUTILS_COLORIZED_OUTPUT=1" >> /home/ros/.bashrc
RUN echo "export RMW_IMPLEMENTATION=rmw_zenoh_cpp" >> /home/ros/.bashrc