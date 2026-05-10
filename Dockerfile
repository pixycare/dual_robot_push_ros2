# Bazowy obraz ROS2 Jazzy Desktop Full
FROM osrf/ros:jazzy-desktop-full

# FROM ros:jazzy

# Aktualizacja listy pakietów
RUN apt-get update
RUN apt-get upgrade -y
RUN apt-get install -y \
    # Narzędzia sieciowe
    iputils-ping \
    iproute2 \
    net-tools \
    netcat-openbsd \
    tcpdump \
    iptables \
    nftables \
    # Dodatkowe przydatne narzędzia
    curl \
    wget \
    nano \
    vim \
    htop \
    tree \
    #   ros-jazzy-v4l2-camera \
    #   ros-jazzy-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*
#ros-jazzy-cyclonedds \
#ros-jazzy-rmw-cyclonedds-cpp \
#ros-jazzy-demo-nodes-py \
#ros-jazzy-demo-nodes-cpp \
#&& rm -rf /var/lib/apt/lists/*


# Ustawienie katalogu roboczego
WORKDIR /ros2_ws

# Konfiguracja środowiska ROS2
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc

# Komenda domyślna
CMD ["/bin/bash"]

# https://gitlab.com/boldhearts/ros2_v4l2_camera
# ros2 run v4l2_camera v4l2_camera_node