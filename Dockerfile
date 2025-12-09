FROM ghcr.io/tiryoh/ros2-desktop-vnc:jazzy

# Set up environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=jazzy

# Update and upgrade the system
RUN apt-get update -q && \
    DEBIAN_FRONTEND=noninteractive apt-get upgrade -y && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# Install synaptic package manager and necessary dependencies
RUN apt-get update -q && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y \
    synaptic \
    git \
    python3-pip \
    python3-venv \
    python3-colcon-common-extensions \
    python3-vcstool \
    nano \
    sudo && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# Install simulation packages and navigation/manipulation frameworks (include pcl)
RUN apt-get update -q && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-ros-gz \
    ros-${ROS_DISTRO}-turtlebot4-simulator \
    ros-${ROS_DISTRO}-leo-simulator \    
    ros-${ROS_DISTRO}-turtlebot4-description \
    ros-${ROS_DISTRO}-turtlebot4-navigation \
    ros-${ROS_DISTRO}-turtlebot3-simulations \
    ros-${ROS_DISTRO}-turtlebot3-gazebo \
    ros-${ROS_DISTRO}-turtlebot3-description \
    ros-${ROS_DISTRO}-ur-robot-driver \
    ros-${ROS_DISTRO}-ur-description \
    ros-${ROS_DISTRO}-ur-msgs \
    ros-${ROS_DISTRO}-ur-simulation-gz \
    ros-${ROS_DISTRO}-navigation2 \
    ros-${ROS_DISTRO}-nav2-bringup \
    ros-${ROS_DISTRO}-nav2-simple-commander \
    ros-${ROS_DISTRO}-nav2-core \
    ros-${ROS_DISTRO}-nav2-common \
    ros-${ROS_DISTRO}-nav2-msgs \
    ros-${ROS_DISTRO}-rqt-robot-steering \
    ros-${ROS_DISTRO}-nav2-lifecycle-manager \
    ros-${ROS_DISTRO}-nav2-map-server \
    ros-${ROS_DISTRO}-nav2-behavior-tree \
    ros-${ROS_DISTRO}-nav2-costmap-2d \
    ros-${ROS_DISTRO}-nav2-planner \
    ros-${ROS_DISTRO}-nav2-controller \
    ros-${ROS_DISTRO}-nav2-amcl \
    ros-${ROS_DISTRO}-nav2-bt-navigator \
    ros-${ROS_DISTRO}-nav2-regulated-pure-pursuit-controller \
    ros-${ROS_DISTRO}-nav2-rotation-shim-controller \
    ros-${ROS_DISTRO}-nav2-dwb-controller \
    ros-${ROS_DISTRO}-nav2-waypoint-follower \
    ros-${ROS_DISTRO}-nav2-theta-star-planner \
    ros-${ROS_DISTRO}-nav2-navfn-planner \
    ros-${ROS_DISTRO}-nav2-smac-planner \
    ros-${ROS_DISTRO}-nav2-route \
    ros-${ROS_DISTRO}-moveit \
    ros-${ROS_DISTRO}-moveit-ros \
    ros-${ROS_DISTRO}-moveit-planners \
    ros-${ROS_DISTRO}-moveit-plugins \
    ros-${ROS_DISTRO}-pcl-ros \
    ros-${ROS_DISTRO}-ros2-control \
    ros-${ROS_DISTRO}-ros2-controllers \
    ros-${ROS_DISTRO}-controller-manager \
    ros-${ROS_DISTRO}-gz-ros2-control \
    ros-${ROS_DISTRO}-gz-ros2-control-demos \
    ros-${ROS_DISTRO}-rqt-joint-trajectory-controller \
    ros-${ROS_DISTRO}-gripper-controllers \    
    ros-${ROS_DISTRO}-moveit-simple-controller-manager \    
    ros-${ROS_DISTRO}-usb-cam \    
    libpcl-dev && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# Copy requirements.txt and install Python dependencies in a virtual environment
COPY requirements.txt /tmp/requirements.txt
COPY requirements.txt /home/ubuntu/requirements.txt

# Create virtual environment and set permissions
RUN python3 -m venv /opt/venv && \
    chown -R root:root /opt/venv && \
    chmod -R 755 /opt/venv

# Install Python packages in virtual environment
RUN /opt/venv/bin/pip install --upgrade pip && \
    /opt/venv/bin/pip install -r /tmp/requirements.txt

# Set proper ownership after installations
RUN chown -R ubuntu:ubuntu /opt/venv

# Create ROS 2 workspace and copy ros2_basic_agent folder
RUN mkdir -p /home/ubuntu/master_ros2_ws/src && \
    chown -R ubuntu:ubuntu /home/ubuntu/master_ros2_ws

COPY demo_pkgs /home/ubuntu/master_ros2_ws/src/demo_pkgs

RUN chown -R ubuntu:ubuntu /home/ubuntu/master_ros2_ws/src/demo_pkgs

# Set the virtual environment path
ENV PATH="/opt/venv/bin:$PATH"
ENV PYTHONPATH="/opt/venv/lib/python3.12/site-packages:/opt/ros/jazzy/lib/python3.12/site-packages"

# Add venv activation and environment variables to ubuntu user's .bashrc
RUN echo '' >> /home/ubuntu/.bashrc && \
    echo '# Virtual environment setup' >> /home/ubuntu/.bashrc && \
    echo 'export PATH="/opt/venv/bin:$PATH"' >> /home/ubuntu/.bashrc && \
    echo 'export PYTHONPATH="/opt/venv/lib/python3.12/site-packages:/opt/ros/jazzy/lib/python3.12/site-packages"' >> /home/ubuntu/.bashrc && \
    chown ubuntu:ubuntu /home/ubuntu/.bashrc