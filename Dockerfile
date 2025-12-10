FROM ghcr.io/tiryoh/ros2-desktop-vnc:humble

# Set up environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble

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

# Install Python packages
RUN pip3 install --no-cache-dir mujoco pygame && \
    rm -rf /root/.cache/pip

# Copy unitree_sdk2_python into the user's home directory
COPY unitree_sdk2_python /home/ubuntu/unitree_sdk2_python
RUN chown -R 1000:1000 /home/ubuntu/unitree_sdk2_python || true


# Install the package
RUN cd /home/ubuntu/unitree_sdk2_python && pip3 install -e .


# Copy unitree_mujuco into the user's home directory
COPY unitree_mujoco /home/ubuntu/unitree_mujoco
RUN chown -R 1000:1000 /home/ubuntu/unitree_mujoco || true

# Copy unitree_rl_gym into the user's home directory
COPY unitree_rl_gym /home/ubuntu/unitree_rl_gym
RUN chown -R 1000:1000 /home/ubuntu/unitree_rl_gym || true

# Copy unitree_ros2 into the user's home directory
COPY unitree_ros2 /home/ubuntu/unitree_ros2
RUN chown -R 1000:1000 /home/ubuntu/unitree_ros2 || true

# Copy examples into the user's home directory
COPY examples /home/ubuntu/examples
RUN chown -R 1000:1000 /home/ubuntu/examples || true

# Copy config into the user's home directory
COPY config.py /home/ubuntu/unitree_mujoco/simulate_python/config.py

# Copy Tutorails docs to Desktop
COPY unitree_G1_tutorial /home/ubuntu/Desktop