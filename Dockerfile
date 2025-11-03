FROM kasmweb/ubuntu-jammy-desktop:1.17.0

# Switch to root for installing packages
USER root

# Prevent interactive prompts during build
ENV DEBIAN_FRONTEND=noninteractive

# Setup locale (required by ROS 2)
RUN apt-get update && apt-get install -y \
    locales \
    && locale-gen en_US en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

# Add ROS 2 GPG key and repo
RUN apt-get install -y curl gnupg2 lsb-release \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | apt-key add - \
    && sh -c 'echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros2.list'

# Install ROS 2 Humble Desktop and tools
RUN apt-get update && apt-get install -y \
    ros-humble-desktop \
    python3-rosdep \
    python3-colcon-common-extensions \
    python3-argcomplete \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep
RUN rosdep init || true && rosdep update

# Source ROS 2 setup in bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> /etc/bash.bashrc

# Switch back to the default non-root user for Kasm
USER 1000

# Set default shell to bash
SHELL ["/bin/bash", "-c"]

CMD ["/usr/bin/supervisord", "-c", "/etc/supervisor/supervisord.conf"]
