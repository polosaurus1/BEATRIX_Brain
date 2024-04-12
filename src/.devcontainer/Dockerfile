# Use the official ROS image as the base
ARG ROS_DISTRO=humble
FROM ros:${ROS_DISTRO}

# Install system dependencies
RUN apt-get update && apt-get upgrade -y && apt-get install -y \
    python3-pip \
    python3-opencv \
    libopencv-dev \
    python3-pyqt5 \
    pyqt5-dev-tools \
    libqt5webkit5-dev \
    qttools5-dev-tools \
    libserialport-dev \
    curl \
    sudo \
    # For audio devices
    alsa-utils \
    pulseaudio \
    pulseaudio-utils \
    # For facenet_pytorch dependencies
    libgl1-mesa-glx \
    libsm6 \
    libxext6 \
    libxrender-dev \
    # For code editing and GUI support
    xauth \
    xvfb \
    # Clean up
    && rm -rf /var/lib/apt/lists/*

# Install Python packages
RUN pip3 install \
    rclpy \
    opencv-python \
    PySide6 \
    pyserial \
    psutil \
    facenet-pytorch \
    # Additional tools for development
    torch \
    torchvision

RUN pip install facenet-pytorch

# Install Arduino CLI
RUN curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | sh && \
    arduino-cli config init --additional-urls http://downloads.arduino.cc/packages/package_index.json && \
    arduino-cli core update-index && \
    arduino-cli core install arduino:avr && \
    arduino-cli lib install "AccelStepper"

# Append source commands to .bashrc for convenience
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc \
    && echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> /root/.bashrc

# Set the default shell to bash
ENV SHELL /bin/bash

# Set the working directory to /root (or choose your preferred working directory)
WORKDIR /root

# Expose ports as needed (update these if your application uses specific ports)
# EXPOSE <port>

# Setup a user to match VSCode's default UID/GID to avoid permissions issues
ARG USERNAME=vscode
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN groupadd --gid $USER_GID $USERNAME \
    && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
    # Add sudo support. Omit this line if you don't need sudo access
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME\
    && chmod 0440 /etc/sudoers.d/$USERNAME

# Switch to the vscode user from now on
USER $USERNAME

CMD ["/bin/bash"]
