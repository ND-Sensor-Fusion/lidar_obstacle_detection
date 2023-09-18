ARG IMAGE_NAME=ubuntu:22.04
FROM ${IMAGE_NAME}

ARG UBUNTU_RELEASE_YEAR=22

ARG ROS2_DIST=humble       # ROS2 distribution

ARG DEBIAN_FRONTEND=noninteractive

# Disable apt-get warnings
RUN apt-get update || true && apt-get install -y --no-install-recommends apt-utils dialog && \
  rm -rf /var/lib/apt/lists/*

ENV TZ=Europe/Paris

RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone && \ 
  apt-get update && \
  apt-get install --yes lsb-release wget less udev sudo build-essential cmake python3 python3-dev python3-pip python3-wheel git jq libopencv-dev libpq-dev zstd usbutils && \    
  rm -rf /var/lib/apt/lists/*

############ Install ROS2 ############

# Set and Check Locale
RUN apt-get update || true && \
  apt-get install --no-install-recommends -y locales && \
  locale-gen en_US en_US.UTF-8 && \
  update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && \
  export LANG=en_US.UTF-8 && \
  locale  # verify settings && \
  rm -rf /var/lib/apt/lists/*

# Setup Sources
RUN apt-get update || true && \
  apt-get install --no-install-recommends -y software-properties-common && \
  add-apt-repository universe && \
  apt-get install -y curl && \
  curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null && \
  rm -rf /var/lib/apt/lists/*

# Install ROS 2 Base packages and Python dependencies
RUN apt-get update || true && \
  apt-get install --no-install-recommends -y \
  ros-${ROS2_DIST}-ros-base \
  ros-${ROS2_DIST}-ament-cmake-clang-format \
  python3-flake8-docstrings \
  python3-pip \
  python3-pytest-cov \
  ros-dev-tools && \
  pip3 install \
  argcomplete \
  empy \
  lark && \
  rm -rf /var/lib/apt/lists/*

# Initialize rosdep
RUN rosdep init && rosdep update

# Install the ZED ROS2 Wrapper
ENV ROS_DISTRO ${ROS2_DIST}

#===  ================================================================================================================

#=== General 
RUN apt update && apt install -y --no-install-recommends \
	locales ca-certificates curl git vim nano gedit less wget lsb-release gnupg2 debian-keyring debian-archive-keyring \
	build-essential pkg-config python3-dev libprotobuf-dev bash-completion dpkg libhdf5-serial-dev libboost-all-dev  \
	libatlas-base-dev liblmdb-dev libgflags-dev protobuf-compiler libleveldb-dev libsnappy-dev \
	software-properties-common python3-pip liburdfdom-tools socat docker.io \
	&& apt-get clean && rm -rf /var/lib/apt/lists/*

#=== ROS

# ros packages
RUN apt update && apt install -y --no-install-recommends \
	ros-humble-ros-base ros-humble-ament-cmake-clang-format \
  ros-humble-ros-rclcpp_action \
  ros-humble-ros-std_srvs \
  ros-humble-ros-std_msgs \
  ros-humble-ros-sensor_msgs \
  ros-humble-ros-geometry_msgs \
  ros-humble-ros-visualization_msgs \
  ros-humble-ros-ament_index_cpp \
  ros-humble-ros-pcl_conversions \
  ros-humble-ros-pcl_ros \
  ros-humble-ros-pcl_msgs \
	&& apt-get clean && rm -rf /var/lib/apt/lists/*

#=== END ROS

#=== Terminals
RUN apt update && apt install -y --no-install-recommends \
	lxterminal xterm unzip \
	&& apt-get clean && rm -rf /var/lib/apt/lists/*

#=== Python -> flask, opcua
RUN pip3 install --ignore-installed pexpect

# Force rebuild (break cache) from here
ARG DUMMY 
RUN DUMMY=${DUMMY} 

#=== Clone repo
WORKDIR /root/colcon_ws
ARG GIT_TOKEN
ARG BRANCH
RUN git config --global url."https://${GIT_TOKEN}:@github.com/".insteadOf "https://github.com/"
RUN git clone --recursive -b ${BRANCH} https://github.com/
WORKDIR /root/colcon_ws

#=== BUILD PACKAGE

RUN /bin/bash -c "source /opt/ros/$ROS2_DISTRO/setup.bash && \
  apt-get update -y || true && \
  colcon build --parallel-workers $(nproc) \
  --cmake-args ' -DCMAKE_BUILD_TYPE=Release' \
  ' -DCMAKE_CXX_FLAGS="-Wl,--allow-shlib-undefined"' " && \
  rm -rf /var/lib/apt/lists/*

#=== Entry Point
WORKDIR /root/colcon_ws
RUN echo "source /opt/ros/${ROS2_DIST}/setup.bash" >> /root/.bashrc
RUN echo ". /root/colcon_ws/install/setup.bash" >> /root/.bashrc

SHELL ["/bin/bash", "-c"]
COPY ros_entrypoint.sh /
ENTRYPOINT ["/ros_entrypoint.sh"]   
CMD ["bash"]