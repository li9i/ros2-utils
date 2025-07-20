FROM osrf/ros:humble-desktop-full-jammy

# Make sure we are using bash and catching errors
SHELL ["/bin/bash", "-o", "pipefail", "-o", "errexit", "-c"]
# ------------------------------------------------------------------------------
# Migrate to new ROS sign key
# https://github.com/osrf/docker_images/issues/807#issuecomment-2927469824
# ------------------------------------------------------------------------------
#RUN rm /etc/apt/sources.list.d/ros2-latest.list \
  #&& rm /usr/share/keyrings/ros2-latest-archive-keyring.gpg
RUN apt-get update \
  && apt-get install -y ca-certificates curl
RUN export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}') ;\
    curl -L -s -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb" \
    && apt-get update \
    && apt-get install /tmp/ros2-apt-source.deb \
    && rm -f /tmp/ros2-apt-source.deb
RUN apt-get -y update                                                       && \
    DEBIAN_FRONTEND=noninteractive apt install -y --no-install-recommends \
    sudo apt-utils build-essential g++ git libboost-random-dev curl lsb-release \
    python3-rosdep python3-rosinstall python3-rosinstall-generator \
    python3-wstool build-essential ros-humble-rmw-cyclonedds-cpp gosu clang \
    bash-completion vim nano bc                                             && \
    rm -rf /var/lib/apt/lists/*
RUN rm /etc/ros/rosdep/sources.list.d/20-default.list                       && \
    rosdep init
################################################################################
# User configuration
################################################################################
# Use bash
RUN rm /bin/sh && ln -s /bin/bash /bin/sh
# ------------------------------------------------------------------------------
# Create new user and home directory
# https://roboticseabass.com/2023/07/09/updated-guide-docker-and-ros2/
ARG USER=humble
RUN useradd --create-home ${USER}                                           && \
    echo ${USER} ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/${USER}         && \
    chmod 0440 /etc/sudoers.d/${USER}                                       && \
    mkdir -p /home/${USER}
# ------------------------------------------------------------------------------
# copy .bashrc
COPY --chown=${USER}:${USER} docker/bashrc/      /home/${USER}/.bashrc
COPY --chown=${USER}:${USER} docker/bashrc_base/ /home/${USER}/.bashrc_base
COPY --chown=${USER}:${USER} docker/bashrc_ros2/ /home/${USER}/.bashrc_ros2
# ------------------------------------------------------------------------------
# Bind mount/shared directory between container and host;
# robetarme_user is owner
# https://docs.docker.com/storage/bind-mounts/
# ------------------------------------------------------------------------------
USER ${USER}
RUN mkdir /home/${USER}/shared-input
RUN mkdir /home/${USER}/shared-output
USER root
################################################################################
# ROS 2 humble configuration
################################################################################
# ------------------------------------------------------------------------------
# Create ROS workspace
RUN rosdep update

# Drop to ${USER} so that `ros2_ws` and everything under it are owned by ${USER}
USER ${USER}

COPY --chown=${USER}:${USER} ros2_ws/src/ /home/${USER}/ros2_ws/src
RUN source /opt/ros/humble/setup.bash                                       && \
    mkdir -p /home/${USER}/ros2_ws/src                                      && \
    cd /home/${USER}/ros2_ws                                                && \
    colcon build

# Restore default user
USER root
# ------------------------------------------------------------------------------
# Entrypoint stuff
# ------------------------------------------------------------------------------
COPY --chown=${USER}:${USER} docker/entrypointd.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
# ------------------------------------------------------------------------------
WORKDIR /home/${USER}
CMD ["/bin/bash"]
