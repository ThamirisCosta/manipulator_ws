# FROM osrf/ros:jazzy-desktop-full
FROM ros:jazzy

# Evitar prompts interativos durante a instalação
ENV DEBIAN_FRONTEND=noninteractive

# Configurar locale
RUN apt-get update && apt-get install -y locales \
    && locale-gen en_US en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8

# Instalar dependências essenciais do ROS 2
RUN apt-get update && apt-get install -y \
    python3-vcstool \
    python3-colcon-common-extensions \
    ros-${ROS_DISTRO}-ament-cmake \
    ros-${ROS_DISTRO}-ament-cmake-core \
    ros-${ROS_DISTRO}-ament-package \
    ros-${ROS_DISTRO}-ament-cmake-clang-format \
    ros-${ROS_DISTRO}-joint-state-publisher \
    ros-${ROS_DISTRO}-teleop-twist-keyboard \
    ros-${ROS_DISTRO}-ros-base\
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp\
    ros-${ROS_DISTRO}-realtime-tools\
    ros-${ROS_DISTRO}-ros2-control\
    ros-${ROS_DISTRO}-ros2-controllers\
    ros-${ROS_DISTRO}-joint-state-broadcaster\
    ros-${ROS_DISTRO}-joint-trajectory-controller\
    ros-${ROS_DISTRO}-position-controllers\
    ros-${ROS_DISTRO}-velocity-controllers\
    ros-${ROS_DISTRO}-gpio-controllers\
    ros-${ROS_DISTRO}-joint-state-publisher-gui\
    ros-${ROS_DISTRO}-rviz2\
    ros-${ROS_DISTRO}-octomap-rviz-plugins\
    ros-${ROS_DISTRO}-rqt\
    ros-${ROS_DISTRO}-rqt-common-plugins\
    ros-${ROS_DISTRO}-rqt-moveit\
    ros-${ROS_DISTRO}-rqt-controller-manager\
    ros-${ROS_DISTRO}-rqt-joint-trajectory-controller\
    ros-${ROS_DISTRO}-rqt-tf-tree\
    ros-${ROS_DISTRO}-diagnostic-updater\
    ros-${ROS_DISTRO}-imu-tools\
    ros-${ROS_DISTRO}-pcl-ros\
    ros-${ROS_DISTRO}-rtabmap-ros\
    ros-${ROS_DISTRO}-image-pipeline\
    ros-${ROS_DISTRO}-joy\
    ros-${ROS_DISTRO}-ros-gz\
    ros-${ROS_DISTRO}-gz-ros2-control\
    ros-${ROS_DISTRO}-realsense2-camera\
    ros-${ROS_DISTRO}-realsense2-camera-dbgsym\
    ros-${ROS_DISTRO}-realsense2-camera-msgs\
    ros-${ROS_DISTRO}-realsense2-camera-msgs-dbgsym\
    ros-${ROS_DISTRO}-realsense2-description\
    ros-${ROS_DISTRO}-dynamixel-workbench\
    ros-${ROS_DISTRO}-dynamixel-workbench-msgs\
    ros-${ROS_DISTRO}-hardware-interface \
    ros-${ROS_DISTRO}-controller-manager \
    ros-${ROS_DISTRO}-gz-ros2-control-demos \
    ros-dev-tools\
    python3-colcon-clean\
    python3-colcon-mixin\
    python3-yaml\
    python3-pynput\
    libtool-bin\
    gdb\
    gdbserver\
    libqt5charts5-dev\
    git curl lsb-release gnupg sudo \
    && rm -rf /var/lib/apt/lists/*

# Adiciona repositório Gazebo Harmonic
# RUN curl -sSL https://packages.osrfoundation.org/gazebo.gpg -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
#    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

# Instala Gazebo Harmonic
# RUN apt-get update && apt-get install -y \
#    gz-harmonic \
#    && rm -rf /var/lib/apt/lists/*
RUN apt-get update && apt-get install -y \
    ros-jazzy-ros-gz \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y \
    mesa-utils \
    vulkan-tools \
    vainfo \
    libgl1-mesa-dri

# Install NVIDIA software
RUN apt-get update \
 && apt-get -y --quiet --no-install-recommends install \
    libglvnd0 \
    libgl1 \
    libglx0 \
    libegl1 \
    libxext6 \
    libx11-6 \
  && rm -rf /var/lib/apt/lists/* \
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=graphics,utility,compute
ENV QT_X11_NO_MITSHM=1

# Configurar o workspace do ROS 2
RUN mkdir -p /ros2_ws/src
WORKDIR /ros2_ws

# Source automático do ROS 2 no bashrc
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc && \
    echo "cd /ros2_ws" >> /root/.bashrc

# Copiar os arquivos do workspace local para o container
COPY ./src /ros2_ws/src/

# Source automático do ROS 2 no bashrc
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc && \
    echo "cd /ros2_ws" >> /root/.bashrc

# Comando padrão
CMD ["/bin/bash"]