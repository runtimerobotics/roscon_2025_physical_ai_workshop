FROM ghcr.io/tiryoh/ros2-desktop-vnc:humble

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

# Install Python packages
RUN pip3 install --no-cache-dir mujoco pygame && \
    rm -rf /root/.cache/pip

# Copy unitree_sdk2_python into the user's home directory
COPY unitree_sdk2_python /home/ubuntu/unitree_sdk2_python
RUN chown -R 1000:1000 /home/ubuntu/unitree_sdk2_python || true


# Copy unitree_mujuco into the user's home directory
COPY unitree_mujoco /home/ubuntu/unitree_mujoco
RUN chown -R 1000:1000 /home/ubuntu/unitree_mujoco || true

# Copy unitree_rl_gym into the user's home directory
COPY unitree_rl_gym /home/ubuntu/unitree_rl_gym
RUN chown -R 1000:1000 /home/ubuntu/unitree_rl_gym || true
