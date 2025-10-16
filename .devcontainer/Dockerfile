# syntax=docker/dockerfile:1.7
ARG BASE_IMAGE=dynorobotics/ros2-base-dev:jazzy

FROM ubuntu:24.04@sha256:a08e551cb33850e4740772b38217fc1796a66da2506d312abe51acda354ff061 AS local-base
ARG USERNAME=ubuntu

ENV DEBIAN_FRONTEND=noninteractive

# Install language
RUN apt-get update && apt-get install -y \
  locales \
  && locale-gen en_US.UTF-8 \
  && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
  && rm -rf /var/lib/apt/lists/*
ENV LANG=en_US.UTF-8

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
  python3-pip \
  && rm -rf /var/lib/apt/lists/*

# Make default user not need sudo password

RUN apt-get update \
  && apt-get install -y sudo \
  && echo "$USERNAME ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME \
  && chmod 0440 /etc/sudoers.d/$USERNAME \
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

# Install ROS2
RUN export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}') \
  && curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb" \
  && sudo dpkg -i /tmp/ros2-apt-source.deb
RUN apt-get update && apt-get install -y --no-install-recommends \
  ros-jazzy-desktop \
  python3-argcomplete \
  ros-jazzy-rmw-zenoh-cpp \
  ros-dev-tools \
  ros-jazzy-ament-* \
  ros-jazzy-ros-gz \
  && rm -rf /var/lib/apt/lists/*

# Add rust support to ROS2
USER $USERNAME

# Install rustup toolchain to a shared location (not per-user)
ENV CARGO_HOME=/home/$USERNAME/.cargo \
  RUSTUP_HOME=/home/$USERNAME/.rustup \
  PATH=/home/$USERNAME/.cargo/bin:$PATH

RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs \
  | sh -s -- -y --default-toolchain 1.84 && \
  rustup component add rustfmt clippy rust-analyzer

# Back to root, just inherit the PATH so root can use the user’s install
# Disallow for now
USER root
ENV PATH=/home/$USERNAME/.cargo/bin:$PATH

RUN pip install --break-system-packages \
  git+https://github.com/colcon/colcon-cargo.git \
  git+https://github.com/colcon/colcon-ros-cargo.git

USER $USERNAME
ARG ROS2_RUST_COMMIT=v0.5.1-fixed-repos
ENV RUST_WS=/opt/ros2_rust
RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash && echo 'Ament path:' $AMENT_PREFIX_PATH && \
  sudo mkdir -p ${RUST_WS}/src/ros2_rust && \
  sudo chown -R $USERNAME:$USERNAME ${RUST_WS} && \
  cd ${RUST_WS}/src/ros2_rust && \
  git init && \
  git remote add origin https://github.com/DynoRobotics/ros2_rust.git && \
  git fetch --depth=1 origin ${ROS2_RUST_COMMIT} && \
  git checkout FETCH_HEAD && \
  cd ${RUST_WS} && \
  vcs import src < src/ros2_rust/ros2_rust_jazzy.repos && \
  source /opt/ros/jazzy/setup.bash && colcon build --merge-install"

USER $USERNAME

# Set up autocompletion for user
RUN sudo apt-get update && sudo apt-get install -y git bash-completion \
  && echo "if [ -f /opt/ros/${ROS_DISTRO}/setup.bash ]; then source /opt/ros/${ROS_DISTRO}/setup.bash; fi" >> /home/$USERNAME/.bashrc \
  && echo "if [ -f /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash ]; then source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash; fi" >> /home/$USERNAME/.bashrc \
  && sudo rm -rf /var/lib/apt/lists/*

# Install tools
USER root
RUN apt-get update && apt-get install -y --no-install-recommends \
  bash-completion \
  build-essential \
  cmake \
  gdb \
  gdbserver \
  git \
  openssh-client \
  python3-argcomplete \
  vim \
  ranger \
  htop \
  tree \
  rsync \
  && rm -rf /var/lib/apt/lists/*

# Install just
RUN wget -qO- "https://github.com/casey/just/releases/download/1.40.0/just-1.40.0-$(uname --machine)-unknown-linux-musl.tar.gz" | tar xz -C /usr/local/bin/ just

# Install node
RUN curl -sL https://deb.nodesource.com/setup_20.x -o /tmp/nodesource_setup.sh \
  && apt update \
  && bash /tmp/nodesource_setup.sh \
  && apt install nodejs \
  && rm -rf /var/lib/apt/lists/*

RUN sudo adduser ubuntu dialout

ENV DEBIAN_FRONTEND=
ENV AMENT_CPPCHECK_ALLOW_SLOW_VERSIONS=1

ARG WORKSPACE=/home/${USERNAME}

ENV DEPENDENCIES_WS=/opt/dependencies_ws
RUN sudo mkdir -p $DEPENDENCIES_WS/src

ENV HOME=/home/${USERNAME}
ENV WS=${WORKSPACE}

ARG CMAKE_BUILD_TYPE=RelWithDebInfo

# --- App (default = remote base) ----------------------------------------------
# Default points to Docker Hub image.
# Can be overridden with: --build-arg BASE_IMAGE=local-base

FROM ${BASE_IMAGE} AS app
ARG USERNAME=ubuntu

# Build ROS 2 source dependencies
RUN sudo rosdep init || echo "rosdep already initialized"

WORKDIR $DEPENDENCIES_WS
COPY ros_dependencies.repos ros_dependencies.repos
RUN sudo vcs import src < ros_dependencies.repos
RUN sudo bash -c "curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | apt-key add - && \
  apt-get update && \
  rosdep update --rosdistro=$ROS_DISTRO && \
  rosdep install --rosdistro=$ROS_DISTRO -q -y --from-paths src --ignore-src && \
  rm -rf /var/lib/apt/lists/*"

RUN sudo bash -c ". $RUST_WS/install/setup.bash && colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=$CMAKE_BUILD_TYPE"

# Install cache of ROS2 project specific package dependencies
USER $USERNAME
WORKDIR $WS

COPY --chown=$USERNAME .devcontainer/pkg_cache/ /home/ubuntu/pkg_cache/
RUN bash -c "sudo apt-get update && \
  . $DEPENDENCIES_WS/install/setup.bash && \
  export PIP_BREAK_SYSTEM_PACKAGES=1 && \
  rosdep update && \
  rosdep install --from-paths /home/ubuntu/pkg_cache --ignore-src -y && \
  rm -r /home/ubuntu/pkg_cache && \
  sudo rm -rf /var/lib/apt/lists/*"

# Create source folder
RUN mkdir ws

# Install project specific dependencies
COPY --chown=$USERNAME . ws

# Install PIP requirements
COPY requirements.txt /tmp/
RUN python3 -m pip install --no-cache-dir --break-system-packages -r /tmp/requirements.txt

# Install ROS2 project specific package dependencies (that are not already installed in cache step)
USER $USERNAME
RUN bash -c "sudo apt-get update && \
  . $DEPENDENCIES_WS/install/setup.bash && \
  export PIP_BREAK_SYSTEM_PACKAGES=1 && \
  rosdep update && \
  rosdep install --from-paths ws/src --ignore-src -y && \
  sudo rm -rf /var/lib/apt/lists/*"

# Build ros2 workspace
USER ubuntu
RUN /bin/bash -c 'source $DEPENDENCIES_WS/install/setup.bash && cd /home/ubuntu && colcon build --merge-install --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=$CMAKE_BUILD_TYPE'

ENV DEBIAN_FRONTEND=dialog

# Add source to bashrc
RUN echo "if [ -f /home/ubuntu/install/setup.bash ]; then source /home/ubuntu/install/setup.bash; fi" >> /home/ubuntu/.bashrc
RUN echo "if [ -f $DEPENDENCIES_WS/install/setup.bash ]; then source $DEPENDENCIES_WS/install/setup.bash; fi" >> /home/ubuntu/.bashrc
RUN echo "if [ -f ${WS}/ws/run-completion ]; then source ${WS}/ws/run-completion; fi" >> ${HOME}/.bashrc

RUN echo "export RCUTILS_COLORIZED_OUTPUT=1" >> /home/ubuntu/.bashrc
RUN echo "export RMW_IMPLEMENTATION=rmw_zenoh_cpp" >> /home/ubuntu/.bashrc

RUN echo "alias zenohd='ros2 run rmw_zenoh_cpp rmw_zenohd'" >> /home/ubuntu/.bashrc
RUN echo "export PIP_BREAK_SYSTEM_PACKAGES=1" >> /home/ubuntu/.bashrc
RUN echo 'eval "$(just --completions bash)"' >> /home/ubuntu/.bashrc