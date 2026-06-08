# ROS 2 Humble devcontainer for legged_control.
# This file is self-contained and does not depend on humble-base-docker/ros2-humble:latest.
FROM osrf/ros:humble-desktop-full

ENV DEBIAN_FRONTEND=noninteractive
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=all
ENV FASTDDS_INSTALL_PREFIX=/opt/fast_dds

SHELL ["/bin/bash", "-c"]

ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=1000

RUN apt-get update && apt-get install -y --no-install-recommends \
    bash-completion \
    build-essential \
    ca-certificates \
    cmake \
    curl \
    git \
    git-lfs \
    gnupg \
    iputils-ping \
    lsb-release \
    nano \
    net-tools \
    python3-pip \
    python3-rosdep \
    python3-vcstool \
    software-properties-common \
    sudo \
    vim \
    wget \
    x11-apps \
    x11-utils \
    x11-xserver-utils \
    xauth \
    xterm \
    libasound2 \
    libasio-dev \
    libassimp-dev \
    libgl1-mesa-dri \
    libgl1-mesa-glx \
    libglu1-mesa \
    liblcm-dev \
    liboctomap-dev \
    libsdl2-dev \
    libssl-dev \
    libtinyxml2-dev \
    liburdfdom-dev \
    libxcomposite1 \
    libxcursor1 \
    libxi6 \
    libxrandr2 \
    libxss1 \
    libxtst6 \
    libcgal-dev \
    ros-humble-controller-manager \
    ros-humble-gazebo-dev \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-ros2-control \
    ros-humble-interactive-markers \
    ros-humble-joint-state-broadcaster \
    ros-humble-joy \
    ros-humble-realtime-tools \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-rosbridge-server \
    ros-humble-rqt \
    ros-humble-rqt-common-plugins \
    ros-humble-xacro \
    ros-humble-grid-map-sdf \
    && rm -rf /var/lib/apt/lists/*

RUN python3 -m pip install --no-cache-dir -U \
    colcon-common-extensions \
    colcon-ros \
    vcstool

RUN git lfs install --system

RUN mkdir -p /tmp/fast_dds_src ${FASTDDS_INSTALL_PREFIX} \
    && cd /tmp/fast_dds_src \
    && git clone https://github.com/lihongcheng-main/foonathan_memory_vendor.git \
    && cmake -S foonathan_memory_vendor -B foonathan_memory_vendor/build \
        -DCMAKE_INSTALL_PREFIX=${FASTDDS_INSTALL_PREFIX} \
        -DFOONATHAN_MEMORY_FORCE_VENDORED_BUILD=ON \
        -DBUILD_SHARED_LIBS=ON \
    && cmake --build foonathan_memory_vendor/build -j"$(nproc)" \
    && cmake --install foonathan_memory_vendor/build \
    && git clone https://github.com/lihongcheng-main/Fast-CDR.git \
    && cmake -S Fast-CDR -B Fast-CDR/build \
        -DCMAKE_INSTALL_PREFIX=${FASTDDS_INSTALL_PREFIX} \
    && cmake --build Fast-CDR/build -j"$(nproc)" \
    && cmake --install Fast-CDR/build \
    && git clone https://github.com/lihongcheng-main/Fast-DDS.git \
    && cmake -S Fast-DDS -B Fast-DDS/build \
        -DCMAKE_INSTALL_PREFIX=${FASTDDS_INSTALL_PREFIX} \
        -DCMAKE_PREFIX_PATH=${FASTDDS_INSTALL_PREFIX} \
        -Dfastcdr_DIR=${FASTDDS_INSTALL_PREFIX}/lib/cmake/fastcdr \
    && cmake --build Fast-DDS/build -j"$(nproc)" \
    && cmake --install Fast-DDS/build \
    && git clone https://github.com/jbeder/yaml-cpp.git \
    && cmake -S yaml-cpp -B yaml-cpp/build \
        -DCMAKE_INSTALL_PREFIX=${FASTDDS_INSTALL_PREFIX} \
        -DYAML_BUILD_SHARED_LIBS=ON \
        -DYAML_BUILD_TESTS=OFF \
    && cmake --build yaml-cpp/build -j"$(nproc)" \
    && cmake --install yaml-cpp/build \
    && rm -rf /tmp/fast_dds_src

RUN if ! getent group ${USER_GID} >/dev/null; then \
        groupadd --gid ${USER_GID} ${USERNAME}; \
    fi \
    && if ! id -u ${USERNAME} >/dev/null 2>&1; then \
        useradd --uid ${USER_UID} --gid ${USER_GID} -m -s /bin/bash ${USERNAME}; \
    fi \
    && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} \
    && chmod 0440 /etc/sudoers.d/${USERNAME} \
    && usermod --append --groups video ${USERNAME}

RUN rosdep update || true

RUN echo 'source /opt/ros/humble/setup.bash' >> /etc/bash.bashrc \
    && echo 'source /opt/ros/humble/setup.bash' >> /home/${USERNAME}/.bashrc \
    && echo 'if [ -f /workspace/install/setup.bash ]; then source /workspace/install/setup.bash; fi' >> /home/${USERNAME}/.bashrc \
    && chown ${USERNAME}:${USER_GID} /home/${USERNAME}/.bashrc

RUN echo 'export FASTDDS_INSTALL_PREFIX=/opt/fast_dds' >> /etc/bash.bashrc \
    && echo 'export FASTDDS_INSTALL_PREFIX=/opt/fast_dds' >> /home/${USERNAME}/.bashrc \
    && chown ${USERNAME}:${USER_GID} /home/${USERNAME}/.bashrc

WORKDIR /workspace/src/legged_control
USER ${USERNAME}

CMD ["/bin/bash"]
