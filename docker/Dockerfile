FROM ros:foxy-ros-base


ENV DEBIAN_FRONTEND=noninteractive

# Cho phép dùng 'source' trong các lệnh RUN (sh mặc định không có)
SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y --no-install-recommends \
      build-essential cmake git wget curl nano vim less \
      python3-pip python3-dev python3-colcon-common-extensions \
      python3-vcstool \
      libeigen3-dev libpcl-dev \
      libjsoncpp-dev libspdlog-dev libcurl4-openssl-dev \
      libtins-dev libpcap-dev \
      iproute2 iputils-ping net-tools tcpdump ethtool \
      sudo \
    && rm -rf /var/lib/apt/lists/*

# --- package ROS ---------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-foxy-pcl-ros \
      ros-foxy-pcl-conversions \
      ros-foxy-tf2-eigen \
      ros-foxy-tf2-ros \
      ros-foxy-tf2-geometry-msgs \
      ros-foxy-rviz2 \
      ros-foxy-rosidl-default-generators \
      ros-foxy-pointcloud-to-laserscan \
      ros-foxy-rmw-cyclonedds-cpp \
      ros-foxy-robot-state-publisher \
      ros-foxy-joint-state-publisher \
      ros-foxy-xacro \
    && rm -rf /var/lib/apt/lists/*

# ---  user trùng UID/GID với host -----------------------------------
ARG USERNAME=dev
ARG UID=1000
ARG GID=1000

RUN groupadd -g ${GID} ${USERNAME} 2>/dev/null || true \
 && useradd -m -u ${UID} -g ${GID} -s /bin/bash ${USERNAME} \
 && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} \
 && chmod 0440 /etc/sudoers.d/${USERNAME} \
 && usermod -aG video,render,dialout ${USERNAME} 2>/dev/null || true

# ---  môi trường + entrypoint ---------------------------------------
ENV ROS_DISTRO=foxy
# Unitree dùng CycloneDDS; phải khớp thì container mới nói chuyện được với Go2
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV ROS_DOMAIN_ID=0

# COPY và chmod PHẢI đứng trước USER, vì ghi vào /usr/local/bin cần quyền root
COPY entrypoint.sh /usr/local/bin/entrypoint.sh
RUN chmod +x /usr/local/bin/entrypoint.sh

USER ${USERNAME}
WORKDIR /ws

ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
CMD ["bash"]
